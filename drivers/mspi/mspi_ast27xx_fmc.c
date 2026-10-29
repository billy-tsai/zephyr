/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_fmc

#include <soc.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define FMC_CONFIG          0x000U
#define FMC_CE_CTRL         0x004U
#define FMC_INTR_STATUS     0x008U
#define FMC_CE0_CTRL         0x010U
#define FMC_ADDR_CONT        0x054U
#define FMC_DMA_RAM_HIGH     0x07CU
#define FMC_DMA_CTRL         0x080U
#define FMC_DMA_FLASH_ADDR   0x084U
#define FMC_DMA_RAM_ADDR     0x088U
#define FMC_DMA_LEN          0x08CU

#define FMC_CONFIG_CE0_WRITE BIT(16)
#define FMC_CE0_ADDR4        BIT(0)
#define FMC_DMA_DONE         BIT(11)
#define FMC_DMA_ENABLE       BIT(0)
#define FMC_DMA_MAX_LEN      BIT(25)
#define FMC_DMA_MIN_LEN      128U

#define FMC_CTRL_FREQ_HIGH   GENMASK(27, 24)
#define FMC_CTRL_COMMAND     GENMASK(23, 16)
#define FMC_CTRL_DUMMY_HIGH  BIT(14)
#define FMC_CTRL_FREQ_LOW    GENMASK(11, 8)
#define FMC_CTRL_DUMMY_LOW   GENMASK(7, 6)
#define FMC_CTRL_CE_STOP     BIT(2)
#define FMC_CTRL_NORMAL_READ 1U
#define FMC_CTRL_USER        3U

struct mspi_ast27xx_fmc_config {
	uintptr_t base;
	uintptr_t window;
	size_t window_size;
	const struct device *clock;
	clock_control_subsys_t hclk;
	const struct pinctrl_dev_config *pcfg;
	struct mspi_cfg mspi_cfg;
};

struct mspi_ast27xx_fmc_data {
	struct k_mutex lock;
	const struct mspi_dev_id *dev_id;
	struct mspi_dev_cfg dev_cfg;
	uint32_t freq_ctrl;
	uint32_t normal_read;
	uint32_t frequency;
};

static uint32_t fmc_read_control(uint32_t command, uint16_t dummy_bytes)
{
	return FIELD_PREP(FMC_CTRL_COMMAND, command) |
	       FIELD_PREP(FMC_CTRL_DUMMY_LOW, dummy_bytes & 3U) |
	       FIELD_PREP(FMC_CTRL_DUMMY_HIGH, dummy_bytes >> 2) | FMC_CTRL_NORMAL_READ;
}

static void fmc_address_mode(const struct mspi_ast27xx_fmc_config *config, bool addr4)
{
	uint32_t val = sys_read32(config->base + FMC_CE_CTRL);

	val = (val & ~FMC_CE0_ADDR4) | (addr4 ? FMC_CE0_ADDR4 : 0U);
	sys_write32(val, config->base + FMC_CE_CTRL);
}

static int fmc_clock_divider(const struct device *dev, uint32_t requested, uint32_t *control,
			     uint32_t *frequency)
{
	const struct mspi_ast27xx_fmc_config *config = dev->config;
	static const uint8_t low_encoding[] = {15U, 7U, 14U, 6U, 13U, 5U, 12U, 4U,
					     11U, 3U, 10U, 2U, 9U, 1U, 8U, 0U};
	uint32_t hclk;
	uint32_t div;
	int ret;

	if (requested == 0U) {
		return -EINVAL;
	}

	ret = clock_control_get_rate(config->clock, config->hclk, &hclk);
	if (ret != 0) {
		return ret;
	}
	if (hclk == 0U) {
		return -EIO;
	}

	div = MAX(2U, DIV_ROUND_UP(hclk, (uint64_t)requested));
	if (div > 256U) {
		return -ENOTSUP;
	}

	*control = FIELD_PREP(FMC_CTRL_FREQ_HIGH, (div - 1U) / 16U) |
		   FIELD_PREP(FMC_CTRL_FREQ_LOW, low_encoding[(div - 1U) % 16U]);
	*frequency = hclk / div;

	return 0;
}

static int mspi_ast27xx_fmc_configure(const struct mspi_dt_spec *spec)
{
	const struct mspi_cfg *cfg = &spec->config;

	if (cfg->op_mode != MSPI_OP_MODE_CONTROLLER || cfg->duplex != MSPI_HALF_DUPLEX ||
	    cfg->channel_num != 0U || cfg->num_periph > 1U || cfg->dqs_support ||
	    cfg->sw_multi_periph || cfg->num_ce_gpios != 0U || cfg->re_init) {
		return -ENOTSUP;
	}

	return 0;
}

static int fmc_validate_dev_config(const struct mspi_dev_cfg *cfg)
{
	if (cfg->ce_num != 0U || cfg->io_mode != MSPI_IO_MODE_SINGLE ||
	    cfg->data_rate != MSPI_DATA_RATE_SINGLE || cfg->cpp != MSPI_CPP_MODE_0 ||
	    cfg->endian != MSPI_XFER_BIG_ENDIAN || cfg->ce_polarity != MSPI_CE_ACTIVE_LOW ||
	    cfg->dqs_enable || cfg->mem_boundary != 0U || cfg->time_to_break != 0U ||
	    cfg->cmd_length > 1U ||
	    (cfg->addr_length != 0U && cfg->addr_length != 3U && cfg->addr_length != 4U) ||
	    cfg->read_cmd > UINT8_MAX || cfg->write_cmd > UINT8_MAX ||
	    cfg->rx_dummy % 8U != 0U || cfg->tx_dummy % 8U != 0U || cfg->rx_dummy > 56U) {
		return -ENOTSUP;
	}

	return 0;
}

static int mspi_ast27xx_fmc_dev_config(const struct device *dev, const struct mspi_dev_id *dev_id,
				     enum mspi_dev_cfg_mask mask, const struct mspi_dev_cfg *cfg)
{
	const struct mspi_ast27xx_fmc_config *config = dev->config;
	struct mspi_ast27xx_fmc_data *data = dev->data;
	struct mspi_dev_cfg next;
	uint32_t freq_ctrl;
	uint32_t frequency;
	uint32_t command;
	bool acquired;
	int ret;

	if (dev_id == NULL || (mask != MSPI_DEVICE_CONFIG_NONE && cfg == NULL)) {
		return -EINVAL;
	}
	if (dev_id->dev_idx != 0U || dev_id->ce.port != NULL ||
	    ((uint32_t)mask & ~MSPI_DEVICE_CONFIG_ALL) != 0U) {
		return -ENOTSUP;
	}

	ret = k_mutex_lock(&data->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE));
	if (ret != 0) {
		return -EBUSY;
	}

	acquired = data->dev_id == NULL;
	if (!acquired && data->dev_id != dev_id) {
		ret = -ESTALE;
		goto out;
	}

	next = data->dev_cfg;
#define FMC_APPLY_CONFIG(flag, field)                                                              \
	if ((mask & MSPI_DEVICE_CONFIG_##flag) != 0U) {                                            \
		next.field = cfg->field;                                                           \
	}
	FMC_APPLY_CONFIG(CE_NUM, ce_num);
	FMC_APPLY_CONFIG(FREQUENCY, freq);
	FMC_APPLY_CONFIG(IO_MODE, io_mode);
	FMC_APPLY_CONFIG(DATA_RATE, data_rate);
	FMC_APPLY_CONFIG(CPP, cpp);
	FMC_APPLY_CONFIG(ENDIAN, endian);
	FMC_APPLY_CONFIG(CE_POL, ce_polarity);
	FMC_APPLY_CONFIG(DQS, dqs_enable);
	FMC_APPLY_CONFIG(RX_DUMMY, rx_dummy);
	FMC_APPLY_CONFIG(TX_DUMMY, tx_dummy);
	FMC_APPLY_CONFIG(READ_CMD, read_cmd);
	FMC_APPLY_CONFIG(WRITE_CMD, write_cmd);
	FMC_APPLY_CONFIG(CMD_LEN, cmd_length);
	FMC_APPLY_CONFIG(ADDR_LEN, addr_length);
	FMC_APPLY_CONFIG(MEM_BOUND, mem_boundary);
	FMC_APPLY_CONFIG(BREAK_TIME, time_to_break);
#undef FMC_APPLY_CONFIG

	ret = fmc_validate_dev_config(&next);
	if (ret != 0) {
		goto out;
	}

	ret = fmc_clock_divider(dev, next.freq, &freq_ctrl, &frequency);
	if (ret != 0) {
		goto out;
	}

	command = next.read_cmd;
	if (command == 0U) {
		command = next.addr_length == 4U ? 0x13U : 0x03U;
	}

	data->dev_cfg = next;
	data->freq_ctrl = freq_ctrl;
	data->frequency = frequency;
	data->normal_read = freq_ctrl |
		fmc_read_control(command, next.read_cmd == 0U ? 0U : next.rx_dummy / 8U);
	fmc_address_mode(config, next.addr_length == 4U);
	sys_write32(data->normal_read, config->base + FMC_CE0_CTRL);
	data->dev_id = dev_id;

out:
	/* Keep the first acquisition until get_channel_status releases the bus. */
	if (ret != 0 || !acquired) {
		(void)k_mutex_unlock(&data->lock);
	}

	return ret;
}

static int fmc_validate_xfer(const struct mspi_xfer *xfer)
{
	if (xfer->num_packet == 0U || xfer->packets == NULL) {
		return -EINVAL;
	}
	if (xfer->async || xfer->hold_ce || xfer->ce_sw_ctrl.gpio.port != NULL ||
	    xfer->ce_sw_ctrl.delay != 0U || xfer->cmd_length > 1U ||
	    (xfer->addr_length != 0U && xfer->addr_length != 3U && xfer->addr_length != 4U) ||
	    xfer->rx_dummy % 8U != 0U || xfer->tx_dummy % 8U != 0U ||
	    (xfer->xfer_mode != MSPI_PIO && xfer->xfer_mode != MSPI_DMA)) {
		return -ENOTSUP;
	}

	for (uint32_t i = 0U; i < xfer->num_packet; i++) {
		const struct mspi_xfer_packet *packet = &xfer->packets[i];

		if (packet->dir != MSPI_RX && packet->dir != MSPI_TX) {
			return -EINVAL;
		}
		if (packet->num_bytes != 0U && packet->data_buf == NULL) {
			return -EINVAL;
		}
		if (packet->cb_mask != MSPI_BUS_NO_CB ||
		    (xfer->cmd_length == 1U && packet->cmd > UINT8_MAX)) {
			return -ENOTSUP;
		}
		if (xfer->addr_length == 3U && packet->address > 0xFFFFFFU) {
			return -EINVAL;
		}
	}

	return 0;
}

static int64_t fmc_deadline(const struct mspi_ast27xx_fmc_data *data, const struct mspi_xfer *xfer)
{
	uint64_t bytes = 0U;
	uint64_t duration;

	for (uint32_t i = 0U; i < xfer->num_packet; i++) {
		const struct mspi_xfer_packet *packet = &xfer->packets[i];

		bytes += (uint64_t)packet->num_bytes + xfer->cmd_length + xfer->addr_length;
		bytes += (packet->dir == MSPI_RX ? xfer->rx_dummy : xfer->tx_dummy) / 8U;
	}

	duration = DIV_ROUND_UP(bytes * 8U * MSEC_PER_SEC, data->frequency);
	duration += CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

	return k_uptime_get() + MAX(duration, xfer->timeout);
}

static int fmc_pio(const struct device *dev, const struct mspi_xfer *xfer,
		   const struct mspi_xfer_packet *packet, int64_t deadline)
{
	const struct mspi_ast27xx_fmc_config *config = dev->config;
	struct mspi_ast27xx_fmc_data *data = dev->data;
	uint32_t control = data->freq_ctrl | FMC_CTRL_USER;
	uint16_t dummy = (packet->dir == MSPI_RX ? xfer->rx_dummy : xfer->tx_dummy) / 8U;
	uint32_t offset = 0U;
	int ret = 0;

	sys_write32(control | FMC_CTRL_CE_STOP, config->base + FMC_CE0_CTRL);
	sys_write32(control, config->base + FMC_CE0_CTRL);
	(void)sys_read32(config->base + FMC_CE0_CTRL);

	if (xfer->cmd_length != 0U) {
		sys_write8(packet->cmd, config->window);
	}
	for (uint8_t i = xfer->addr_length; i > 0U; i--) {
		sys_write8(packet->address >> ((i - 1U) * 8U), config->window);
	}
	for (uint16_t i = 0U; i < dummy; i++) {
		sys_write8(0U, config->window);
	}
	while (packet->num_bytes - offset >= sizeof(uint32_t)) {
		if ((offset % 256U) == 0U && k_uptime_get() >= deadline) {
			ret = -ETIMEDOUT;
			goto out;
		}
		if (packet->dir == MSPI_RX) {
			sys_put_le32(sys_read32(config->window), packet->data_buf + offset);
		} else {
			sys_write32(sys_get_le32(packet->data_buf + offset), config->window);
		}
		offset += sizeof(uint32_t);
	}
	while (offset < packet->num_bytes) {
		if (packet->dir == MSPI_RX) {
			packet->data_buf[offset] = sys_read8(config->window);
		} else {
			sys_write8(packet->data_buf[offset], config->window);
		}
		offset++;
	}

out:
	sys_write32(control | FMC_CTRL_CE_STOP, config->base + FMC_CE0_CTRL);

	return ret;
}

static bool fmc_use_dma(const struct mspi_ast27xx_fmc_config *config,
			const struct mspi_xfer *xfer, const struct mspi_xfer_packet *packet)
{
	return xfer->xfer_mode == MSPI_DMA && packet->dir == MSPI_RX &&
	       packet->num_bytes > FMC_DMA_MIN_LEN && packet->num_bytes <= FMC_DMA_MAX_LEN &&
	       IS_ALIGNED(packet->address, 4U) && IS_ALIGNED(packet->data_buf, 4U) &&
	       xfer->cmd_length == 1U && (xfer->addr_length == 3U || xfer->addr_length == 4U) &&
	       xfer->rx_dummy / 8U <= 7U && packet->address < config->window_size &&
	       packet->num_bytes <= config->window_size - packet->address;
}

static int fmc_dma_read(const struct device *dev, const struct mspi_xfer *xfer,
			const struct mspi_xfer_packet *packet, int64_t deadline)
{
	const struct mspi_ast27xx_fmc_config *config = dev->config;
	struct mspi_ast27xx_fmc_data *data = dev->data;
	uint64_t address = ast27xx_soc_virt_addr_to_phy_addr((uintptr_t)packet->data_buf);
	uint32_t control = data->freq_ctrl | fmc_read_control(packet->cmd, xfer->rx_dummy / 8U);
	int ret = 0;

	fmc_address_mode(config, xfer->addr_length == 4U);
	sys_write32(control, config->base + FMC_CE0_CTRL);
	sys_write32(0U, config->base + FMC_DMA_CTRL);
	sys_write32(address >> 32, config->base + FMC_DMA_RAM_HIGH);
	/* SPI084 takes the flash offset within the CE0 decode range. */
	sys_write32(packet->address, config->base + FMC_DMA_FLASH_ADDR);
	sys_write32((uint32_t)address, config->base + FMC_DMA_RAM_ADDR);
	sys_write32(packet->num_bytes - 1U, config->base + FMC_DMA_LEN);
	sys_write32(FMC_DMA_ENABLE, config->base + FMC_DMA_CTRL);

	while ((sys_read32(config->base + FMC_INTR_STATUS) & FMC_DMA_DONE) == 0U) {
		if (k_uptime_get() >= deadline) {
			ret = -ETIMEDOUT;
			break;
		}
		k_busy_wait(1U);
	}

	sys_write32(0U, config->base + FMC_DMA_CTRL);
	sys_cache_data_invd_range(packet->data_buf, packet->num_bytes);

	return ret;
}

static int fmc_transfer_packet(const struct device *dev, const struct mspi_xfer *xfer,
			       const struct mspi_xfer_packet *packet, int64_t deadline)
{
	if (IS_ENABLED(CONFIG_MSPI_AST27XX_FMC_DMA) && fmc_use_dma(dev->config, xfer, packet)) {
		return fmc_dma_read(dev, xfer, packet, deadline);
	}
	return fmc_pio(dev, xfer, packet, deadline);
}

static int mspi_ast27xx_fmc_transceive(const struct device *dev, const struct mspi_dev_id *dev_id,
				     const struct mspi_xfer *xfer)
{
	const struct mspi_ast27xx_fmc_config *config = dev->config;
	struct mspi_ast27xx_fmc_data *data = dev->data;
	uint32_t addr_cont;
	int64_t deadline;
	int ret;

	ret = fmc_validate_xfer(xfer);
	if (ret != 0) {
		return ret;
	}
	ret = k_mutex_lock(&data->lock, K_MSEC(xfer->timeout));
	if (ret != 0) {
		return -EBUSY;
	}
	if (dev_id == NULL || dev_id != data->dev_id) {
		ret = -ESTALE;
		goto out;
	}

	deadline = fmc_deadline(data, xfer);
	/*
	 * SPI054 holds the CS address-continuity modes and the 4 KB address
	 * boundary control; transfers run with all of them cleared.
	 */
	addr_cont = sys_read32(config->base + FMC_ADDR_CONT);
	sys_write32(0U, config->base + FMC_ADDR_CONT);

	for (uint32_t i = 0U; i < xfer->num_packet; i++) {
		if (k_uptime_get() >= deadline) {
			ret = -ETIMEDOUT;
			break;
		}
		ret = fmc_transfer_packet(dev, xfer, &xfer->packets[i], deadline);
		if (ret != 0) {
			break;
		}
	}

	fmc_address_mode(config, data->dev_cfg.addr_length == 4U);
	sys_write32(data->normal_read, config->base + FMC_CE0_CTRL);
	sys_write32(addr_cont, config->base + FMC_ADDR_CONT);

out:
	(void)k_mutex_unlock(&data->lock);

	return ret;
}

static int mspi_ast27xx_fmc_get_channel_status(const struct device *dev, uint8_t channel)
{
	struct mspi_ast27xx_fmc_data *data = dev->data;
	bool acquired;
	int ret;

	if (channel != 0U) {
		return -ENOTSUP;
	}
	ret = k_mutex_lock(&data->lock, K_NO_WAIT);
	if (ret != 0) {
		return -EBUSY;
	}

	acquired = data->dev_id != NULL;
	data->dev_id = NULL;
	(void)k_mutex_unlock(&data->lock);
	if (acquired) {
		ret = k_mutex_unlock(&data->lock);
	}

	return ret;
}

static int mspi_ast27xx_fmc_init(const struct device *dev)
{
	const struct mspi_ast27xx_fmc_config *config = dev->config;
	struct mspi_ast27xx_fmc_data *data = dev->data;
	const struct mspi_dt_spec spec = {.bus = dev, .config = config->mspi_cfg};
	int ret;

	ret = mspi_ast27xx_fmc_configure(&spec);
	if (ret != 0) {
		return ret;
	}
	if (!device_is_ready(config->clock)) {
		return -ENODEV;
	}
	if (IS_ENABLED(CONFIG_PINCTRL) && config->pcfg != NULL) {
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			return ret;
		}
	}

	k_mutex_init(&data->lock);
	data->dev_cfg.endian = MSPI_XFER_BIG_ENDIAN;
	data->dev_cfg.cmd_length = 1U;
	data->dev_cfg.addr_length = 3U;
	sys_set_bits(config->base + FMC_CONFIG, FMC_CONFIG_CE0_WRITE);

	return 0;
}

static DEVICE_API(mspi, mspi_ast27xx_fmc_api) = {
	.config = mspi_ast27xx_fmc_configure,
	.dev_config = mspi_ast27xx_fmc_dev_config,
	.get_channel_status = mspi_ast27xx_fmc_get_channel_status,
	.transceive = mspi_ast27xx_fmc_transceive,
};

#define FMC_INIT(n)                                                                                \
	BUILD_ASSERT(DT_NUM_REGS(DT_DRV_INST(n)) == 2U, "FMC requires control and AHB regions");  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0), (PINCTRL_DT_INST_DEFINE(n);), ())         \
	static struct mspi_ast27xx_fmc_data fmc_data_##n;                                         \
	static const struct mspi_ast27xx_fmc_config fmc_config_##n = {                            \
		.base = DT_INST_REG_ADDR_BY_IDX(n, 0),                                            \
		.window = DT_INST_REG_ADDR_BY_IDX(n, 1),                                          \
		.window_size = DT_INST_REG_SIZE_BY_IDX(n, 1),                                     \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, hclk)),                      \
		.hclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, hclk, clkid),       \
		.pcfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),                           \
			(PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL)),                              \
		.mspi_cfg = {                                                                    \
			.op_mode = DT_INST_ENUM_IDX_OR(n, op_mode, MSPI_OP_MODE_CONTROLLER),       \
			.duplex = DT_INST_ENUM_IDX_OR(n, duplex, MSPI_HALF_DUPLEX),                \
			.dqs_support = DT_INST_PROP(n, dqs_support),                               \
			.sw_multi_periph = DT_INST_PROP(n, software_multiperipheral),              \
			.num_periph = DT_INST_CHILD_NUM_STATUS_OKAY(n),                            \
			.num_ce_gpios = DT_INST_PROP_LEN_OR(n, ce_gpios, 0),                       \
		},                                                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, mspi_ast27xx_fmc_init, NULL, &fmc_data_##n, &fmc_config_##n,      \
			      POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY, &mspi_ast27xx_fmc_api);

DT_INST_FOREACH_STATUS_OKAY(FMC_INIT)
