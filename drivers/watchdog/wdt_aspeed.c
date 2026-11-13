/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define WDT_RELOAD       0x04U
#define WDT_RESTART      0x08U
#define WDT_CTRL         0x0cU
#define WDT_STATUS       0x10U
#define WDT_CLEAR_STATUS 0x14U

#define WDT_RESTART_MAGIC     0x4755U
#define WDT_CTRL_RESET_MODE   GENMASK(6, 5)
#define WDT_CTRL_FULL_CHIP    BIT(5)
#define WDT_CTRL_RESET_BY_SOC BIT(4)
#define WDT_CTRL_INT_ENABLE   BIT(2)
#define WDT_CTRL_RESET_SYSTEM BIT(1)
#define WDT_CTRL_ENABLE       BIT(0)
#define WDT_STATUS_TIMEOUT    BIT(0)
#define WDT_TICKS_PER_MS      1000U

#define AST2700_WDT_RESET_MASKS 5U

/* An SCU bit that blocks the watchdog's full-chip reset while it is set */
struct aspeed_wdt_scu_gate {
	uint16_t reg;
	uint32_t bit;
};

struct aspeed_wdt_soc {
	uint8_t num_reset_masks;
	uint8_t reset_mask_reg;
	uint8_t sw_reset_mask_reg;
	struct aspeed_wdt_scu_gate scu0_gate;
	struct aspeed_wdt_scu_gate scu1_gate;
};

struct aspeed_wdt_config {
	uintptr_t base;
	const struct aspeed_wdt_soc *soc;
	const struct device *scu0;
	const struct device *scu1;
	const uint32_t *reset_mask;
	uint8_t num_reset_mask;
	void (*irq_config)(void);
};

struct aspeed_wdt_data {
	struct k_spinlock lock;
	wdt_callback_t callback;
	uint32_t reload;
	uint8_t flags;
	bool timeout_installed;
};

static const struct aspeed_wdt_soc aspeed_wdt_ast2700 = {
	.num_reset_masks = AST2700_WDT_RESET_MASKS,
	.reset_mask_reg = 0x1cU,
	.sw_reset_mask_reg = 0x34U,
	.scu0_gate = {.reg = 0xc8U, .bit = BIT(9)},
	.scu1_gate = {.reg = 0xc8U, .bit = BIT(7)},
};

static int aspeed_wdt_scu_ungate(const struct device *scu, const struct aspeed_wdt_scu_gate *gate)
{
	uint32_t val;
	int ret;

	if (scu == NULL) {
		return 0;
	}

	ret = syscon_read_reg(scu, gate->reg, &val);
	if (ret != 0) {
		return ret;
	}

	return syscon_write_reg(scu, gate->reg, val & ~gate->bit);
}

static int aspeed_wdt_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int ret = 0;

	if ((sys_read32(config->base + WDT_CTRL) & WDT_CTRL_ENABLE) != 0U) {
		ret = -EBUSY;
		goto out;
	}

	if (data->timeout_installed) {
		ret = -ENOMEM;
		goto out;
	}

	if (cfg->window.min != 0U || cfg->window.max == 0U ||
	    cfg->window.max > UINT32_MAX / WDT_TICKS_PER_MS) {
		ret = -EINVAL;
		goto out;
	}

	switch (cfg->flags) {
	case WDT_FLAG_RESET_SOC:
	case WDT_FLAG_RESET_CPU_CORE:
		break;
	case WDT_FLAG_RESET_NONE:
		if (cfg->callback == NULL) {
			ret = -EINVAL;
			goto out;
		}
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	/* The timeout callback is delivered by the watchdog interrupt. */
	if (cfg->callback != NULL && config->irq_config == NULL) {
		ret = -ENOTSUP;
		goto out;
	}

	data->reload = cfg->window.max * WDT_TICKS_PER_MS;
	data->flags = cfg->flags;
	data->callback = cfg->callback;
	data->timeout_installed = true;

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int aspeed_wdt_setup(const struct device *dev, uint8_t options)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t ctrl = sys_read32(config->base + WDT_CTRL);
	int ret = 0;

	if ((ctrl & WDT_CTRL_ENABLE) != 0U) {
		ret = -EBUSY;
		goto out;
	}

	if (!data->timeout_installed) {
		ret = -EINVAL;
		goto out;
	}

	/* The counter keeps running while the CPU is halted by a debugger. */
	if (options != 0U) {
		ret = -ENOTSUP;
		goto out;
	}

	ctrl &= ~(WDT_CTRL_RESET_MODE | WDT_CTRL_RESET_BY_SOC | WDT_CTRL_INT_ENABLE |
		  WDT_CTRL_RESET_SYSTEM);

	switch (data->flags) {
	case WDT_FLAG_RESET_SOC:
		ret = aspeed_wdt_scu_ungate(config->scu0, &config->soc->scu0_gate);
		if (ret == 0) {
			ret = aspeed_wdt_scu_ungate(config->scu1, &config->soc->scu1_gate);
		}
		if (ret != 0) {
			goto out;
		}
		ctrl |= WDT_CTRL_FULL_CHIP | WDT_CTRL_RESET_BY_SOC | WDT_CTRL_RESET_SYSTEM;
		break;
	case WDT_FLAG_RESET_CPU_CORE:
		/* Reset mode 0 resets the SoC blocks selected by the reset mask registers. */
		ctrl |= WDT_CTRL_RESET_BY_SOC | WDT_CTRL_RESET_SYSTEM;
		break;
	default:
		break;
	}

	if (data->callback != NULL) {
		ctrl |= WDT_CTRL_INT_ENABLE;
	}

	sys_write32(data->reload, config->base + WDT_RELOAD);
	sys_write32(WDT_RESTART_MAGIC, config->base + WDT_RESTART);
	/* A pending timeout must be cleared before enabling system reset. */
	sys_write32(WDT_STATUS_TIMEOUT, config->base + WDT_CLEAR_STATUS);
	sys_write32(ctrl | WDT_CTRL_ENABLE, config->base + WDT_CTRL);

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int aspeed_wdt_feed(const struct device *dev, int channel_id)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	if (channel_id != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	if (!data->timeout_installed ||
	    (sys_read32(config->base + WDT_CTRL) & WDT_CTRL_ENABLE) == 0U) {
		ret = -EINVAL;
		goto out;
	}

	sys_write32(WDT_RESTART_MAGIC, config->base + WDT_RESTART);

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int aspeed_wdt_disable(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t ctrl = sys_read32(config->base + WDT_CTRL);
	int ret = 0;

	data->timeout_installed = false;
	data->callback = NULL;

	if ((ctrl & WDT_CTRL_ENABLE) == 0U) {
		ret = -EFAULT;
		goto out;
	}

	ctrl &= ~(WDT_CTRL_ENABLE | WDT_CTRL_INT_ENABLE | WDT_CTRL_RESET_SYSTEM);
	sys_write32(ctrl, config->base + WDT_CTRL);
	sys_write32(WDT_STATUS_TIMEOUT, config->base + WDT_CLEAR_STATUS);

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static void __maybe_unused aspeed_wdt_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *data = dev->data;

	if ((sys_read32(config->base + WDT_STATUS) & WDT_STATUS_TIMEOUT) == 0U) {
		return;
	}

	sys_write32(WDT_STATUS_TIMEOUT, config->base + WDT_CLEAR_STATUS);

	if (data->callback != NULL) {
		data->callback(dev, 0);
	}
}

static int aspeed_wdt_init(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	uint32_t ctrl;

	if (IS_ENABLED(CONFIG_WDT_DISABLE_AT_BOOT)) {
		ctrl = sys_read32(config->base + WDT_CTRL);
		ctrl &= ~(WDT_CTRL_ENABLE | WDT_CTRL_INT_ENABLE | WDT_CTRL_RESET_SYSTEM);
		sys_write32(ctrl, config->base + WDT_CTRL);
	}

	/* Without aspeed,reset-mask the masks are left as the boot firmware set them. */
	for (uint8_t i = 0U; i < config->num_reset_mask; i++) {
		sys_write32(config->reset_mask[i],
			    config->base + config->soc->reset_mask_reg + i * sizeof(uint32_t));
		sys_write32(config->reset_mask[i],
			    config->base + config->soc->sw_reset_mask_reg + i * sizeof(uint32_t));
	}

	if (config->irq_config != NULL) {
		config->irq_config();
	}

	return 0;
}

static DEVICE_API(wdt, aspeed_wdt_api) = {
	.install_timeout = aspeed_wdt_install_timeout,
	.setup = aspeed_wdt_setup,
	.feed = aspeed_wdt_feed,
	.disable = aspeed_wdt_disable,
};

#define ASPEED_WDT_IRQ_CONFIG(node)                                                                \
	static void aspeed_wdt_irq_config_##node(void)                                             \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(node), DT_IRQ(node, priority), aspeed_wdt_isr,                 \
			    DEVICE_DT_GET(node), 0);                                               \
		irq_enable(DT_IRQN(node));                                                         \
	}

#define ASPEED_WDT_SCU(node, prop)                                                                 \
	COND_CODE_1(DT_NODE_HAS_PROP(node, prop), (DEVICE_DT_GET(DT_PHANDLE(node, prop))), (NULL))

#define ASPEED_WDT_DEFINE(node, soc_data, max_reset_masks)                                         \
	BUILD_ASSERT(DT_PROP_LEN_OR(node, aspeed_reset_mask, 0) <= (max_reset_masks),              \
		     "too many aspeed,reset-mask entries");                                        \
	IF_ENABLED(DT_IRQ_HAS_IDX(node, 0), (ASPEED_WDT_IRQ_CONFIG(node)))                        \
	IF_ENABLED(DT_NODE_HAS_PROP(node, aspeed_reset_mask),                                      \
		   (static const uint32_t aspeed_wdt_reset_mask_##node[] =                         \
			    DT_PROP(node, aspeed_reset_mask);))                                    \
	static const struct aspeed_wdt_config aspeed_wdt_config_##node = {                         \
		.base = DT_REG_ADDR(node),                                                         \
		.soc = &(soc_data),                                                                \
		.scu0 = ASPEED_WDT_SCU(node, aspeed_scu0),                                         \
		.scu1 = ASPEED_WDT_SCU(node, aspeed_scu1),                                         \
		.reset_mask = COND_CODE_1(DT_NODE_HAS_PROP(node, aspeed_reset_mask),               \
					  (aspeed_wdt_reset_mask_##node), (NULL)),                 \
		.num_reset_mask = DT_PROP_LEN_OR(node, aspeed_reset_mask, 0),                      \
		.irq_config = COND_CODE_1(DT_IRQ_HAS_IDX(node, 0),                                 \
					  (aspeed_wdt_irq_config_##node), (NULL)),                 \
	};                                                                                         \
	static struct aspeed_wdt_data aspeed_wdt_data_##node;                                      \
                                                                                                   \
	DEVICE_DT_DEFINE(node, aspeed_wdt_init, NULL, &aspeed_wdt_data_##node,                     \
			 &aspeed_wdt_config_##node, PRE_KERNEL_1,                                  \
			 CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &aspeed_wdt_api);

DT_FOREACH_STATUS_OKAY_VARGS(aspeed_ast2700_wdt, ASPEED_WDT_DEFINE, aspeed_wdt_ast2700,
			     AST2700_WDT_RESET_MASKS)
