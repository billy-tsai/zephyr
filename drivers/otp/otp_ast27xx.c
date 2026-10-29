/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_otp

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/otp.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(otp_ast27xx, CONFIG_OTP_LOG_LEVEL);

#define AST27XX_OTP_KEY        0x000U
#define AST27XX_OTP_CMD        0x004U
#define AST27XX_OTP_STATUS     0x018U
#define AST27XX_OTP_ADDR       0x01cU
#define AST27XX_OTP_RDATA      0x020U
#define AST27XX_OTP_DBG01      0x0c8U
#define AST27XX_OTP_ECCBRP_EN  0x0d4U
#define AST27XX_OTP_PMC_CQ     0x0e4U
#define AST27XX_OTP_DAP_CFG_RQ 0x538U

#define AST27XX_OTP_UNLOCK_KEY 0x349fe38aU
#define AST27XX_OTP_CMD_READ   0x23b1e361U
#define AST27XX_OTP_BUSY       BIT(0)
#define AST27XX_OTP_ERROR      GENMASK(7, 4)
#define AST27XX_OTP_SYNDROME   GENMASK(4, 0)
#define AST27XX_OTP_CORRECTED  BIT(5)
#define AST27XX_OTP_ECC_ENABLE BIT(0)
#define AST27XX_OTP_BYPASS_DAP BIT(2)
#define AST27XX_OTP_DAP_ECC    0x40008U

#define AST27XX_OTP_SIZE       16384U
#define AST27XX_OTP_WORD_SIZE  2U
#define AST27XX_OTP_TIMEOUT_US 10000U

#define AST27XX_OTP_RBP_START   0x3e0U
#define AST27XX_OTP_RBP_END     0x3ffU
#define AST27XX_OTP_STRAP_START 0x420U
#define AST27XX_OTP_STRAP_END   0x42fU
#define AST27XX_OTP_STRAP14     0x42eU
#define AST27XX_OTP_PUF_START   0x1f80U
#define AST27XX_OTP_GLOBAL_ECC  BIT(0)

struct ast27xx_otp_config {
	DEVICE_MMIO_ROM;
};

struct ast27xx_otp_data {
	DEVICE_MMIO_RAM;
	struct k_mutex lock;
	bool global_ecc;
};

static bool ast27xx_otp_ecc_enabled(const struct device *dev, uint32_t word_addr)
{
	const struct ast27xx_otp_data *data = dev->data;

	/* RBP and STRAP do not support ECC or bit repair. */
	if ((word_addr >= AST27XX_OTP_RBP_START && word_addr <= AST27XX_OTP_RBP_END) ||
	    (word_addr >= AST27XX_OTP_STRAP_START && word_addr <= AST27XX_OTP_STRAP_END)) {
		return false;
	}

	return data->global_ecc || word_addr < AST27XX_OTP_RBP_START ||
	       word_addr >= AST27XX_OTP_PUF_START;
}

static int ast27xx_otp_wait_complete(const struct device *dev, uint32_t word_addr)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint32_t status;
	bool done;

	done = WAIT_FOR(((status = sys_read32(base + AST27XX_OTP_STATUS)) & AST27XX_OTP_BUSY) == 0U,
			AST27XX_OTP_TIMEOUT_US, k_busy_wait(1));
	if (!done) {
		LOG_ERR("Read timeout at word 0x%04x: status 0x%08x", word_addr, status);
		return -ETIMEDOUT;
	}

	if (FIELD_GET(AST27XX_OTP_ERROR, status) == 0U) {
		return 0;
	}

	LOG_ERR("Read failed at word 0x%04x: status 0x%08x", word_addr, status);

	switch (FIELD_GET(AST27XX_OTP_ERROR, status)) {
	case 3U:
	case 4U:
		return -EACCES;
	case 1U:
	case 2U:
	default:
		return -EIO;
	}
}

static int ast27xx_otp_check_ecc(const struct device *dev, uint32_t word_addr)
{
	uint32_t status = sys_read32(DEVICE_MMIO_GET(dev) + AST27XX_OTP_DBG01);

	if ((status & AST27XX_OTP_SYNDROME) == 0U) {
		return 0;
	}

	if ((status & AST27XX_OTP_CORRECTED) != 0U) {
		LOG_DBG("Corrected ECC error at word 0x%04x: status 0x%08x", word_addr, status);
		return 0;
	}

	LOG_ERR("Uncorrectable ECC error at word 0x%04x: status 0x%08x", word_addr, status);
	return -EIO;
}

static int ast27xx_otp_read_word(const struct device *dev, uint32_t word_addr, bool ecc,
				uint16_t *value)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	int ret;

	sys_write32(word_addr, base + AST27XX_OTP_ADDR);
	sys_write32(ecc ? AST27XX_OTP_ECC_ENABLE : 0U, base + AST27XX_OTP_ECCBRP_EN);
	sys_write32(ecc ? AST27XX_OTP_BYPASS_DAP : 0U, base + AST27XX_OTP_PMC_CQ);
	if (ecc) {
		/* The read FSM must be bypassed to apply the DAP ECC/bit repair setting. */
		sys_write32(AST27XX_OTP_DAP_ECC, base + AST27XX_OTP_DAP_CFG_RQ);
	}

	sys_write32(AST27XX_OTP_CMD_READ, base + AST27XX_OTP_CMD);
	ret = ast27xx_otp_wait_complete(dev, word_addr);
	if (ret == 0) {
		*value = (uint16_t)sys_read32(base + AST27XX_OTP_RDATA);
		if (ecc) {
			ret = ast27xx_otp_check_ecc(dev, word_addr);
		}
	}

	/* Clearing PMC CQ also restores DAP CFG RQ automatically, including on errors. */
	sys_write32(0U, base + AST27XX_OTP_PMC_CQ);

	return ret;
}

static int ast27xx_otp_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	struct ast27xx_otp_data *data = dev->data;
	uint8_t *out = buf;
	int ret;
	int unlock_ret;

	if (offset < 0) {
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	if (offset > (off_t)AST27XX_OTP_SIZE || len > AST27XX_OTP_SIZE - (size_t)offset ||
	    buf == NULL) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&data->lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	while (len > 0U) {
		uint32_t word_addr = (uint32_t)offset / AST27XX_OTP_WORD_SIZE;
		size_t byte_in_word = (size_t)offset % AST27XX_OTP_WORD_SIZE;
		size_t count = MIN(AST27XX_OTP_WORD_SIZE - byte_in_word, len);
		uint8_t bytes[AST27XX_OTP_WORD_SIZE];
		uint16_t value;

		ret = ast27xx_otp_read_word(dev, word_addr,
					  ast27xx_otp_ecc_enabled(dev, word_addr), &value);
		if (ret != 0) {
			break;
		}

		sys_put_le16(value, bytes);
		memcpy(out, &bytes[byte_in_word], count);
		out += count;
		offset += count;
		len -= count;
	}

	unlock_ret = k_mutex_unlock(&data->lock);

	return ret != 0 ? ret : unlock_ret;
}

static int ast27xx_otp_init(const struct device *dev)
{
	struct ast27xx_otp_data *data = dev->data;
	uint16_t strap14;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = k_mutex_init(&data->lock);
	if (ret != 0) {
		return ret;
	}

	sys_write32(AST27XX_OTP_UNLOCK_KEY, DEVICE_MMIO_GET(dev) + AST27XX_OTP_KEY);
	ret = ast27xx_otp_read_word(dev, AST27XX_OTP_STRAP14, false, &strap14);
	if (ret != 0) {
		return ret;
	}

	data->global_ecc = (strap14 & AST27XX_OTP_GLOBAL_ECC) != 0U;

	return 0;
}

static DEVICE_API(otp, ast27xx_otp_api) = {
	.read = ast27xx_otp_read,
};

#define AST27XX_OTP_DEFINE(inst)                                                                   \
	static const struct ast27xx_otp_config ast27xx_otp_config_##inst = {                       \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                        \
	static struct ast27xx_otp_data ast27xx_otp_data_##inst;                                    \
	DEVICE_DT_INST_DEFINE(inst, ast27xx_otp_init, NULL, &ast27xx_otp_data_##inst,              \
			      &ast27xx_otp_config_##inst, POST_KERNEL, CONFIG_OTP_INIT_PRIORITY,   \
			      &ast27xx_otp_api);

DT_INST_FOREACH_STATUS_OKAY(AST27XX_OTP_DEFINE)
