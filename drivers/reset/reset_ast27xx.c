/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast27xx_reset

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/util.h>

#define RESET_CTRL0        0x200U
#define RESET_CTRL1        0x220U
#define RESET_CLEAR_OFFSET 0x004U
#define RESET_BANK_SIZE    32U
#define RESET_COUNT        64U

struct reset_ast27xx_config {
	const struct device *syscon;
};

static int ast27xx_reset_reg(uint32_t id, uint32_t *reg, uint32_t *bit)
{
	if (id >= RESET_COUNT) {
		return -EINVAL;
	}

	if (id < RESET_BANK_SIZE) {
		*reg = RESET_CTRL0;
		*bit = BIT(id);
	} else {
		*reg = RESET_CTRL1;
		*bit = BIT(id - RESET_BANK_SIZE);
	}

	return 0;
}

static int ast27xx_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct reset_ast27xx_config *config = dev->config;
	uint32_t reg;
	uint32_t bit;
	int ret;

	ret = ast27xx_reset_reg(id, &reg, &bit);
	if (ret != 0) {
		return ret;
	}

	/* Reading and writing back RW1S bits would assert other reset lines. */
	return syscon_write_reg(config->syscon, reg, bit);
}

static int ast27xx_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct reset_ast27xx_config *config = dev->config;
	uint32_t reg;
	uint32_t bit;
	int ret;

	ret = ast27xx_reset_reg(id, &reg, &bit);
	if (ret != 0) {
		return ret;
	}

	return syscon_write_reg(config->syscon, reg + RESET_CLEAR_OFFSET, bit);
}

static int ast27xx_reset_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct reset_ast27xx_config *config = dev->config;
	uint32_t reg;
	uint32_t bit;
	uint32_t val;
	int ret;

	ret = ast27xx_reset_reg(id, &reg, &bit);
	if (ret != 0) {
		return ret;
	}

	ret = syscon_read_reg(config->syscon, reg, &val);
	if (ret == 0) {
		*status = (val & bit) != 0U;
	}

	return ret;
}

static int ast27xx_reset_line_toggle(const struct device *dev, uint32_t id)
{
	int ret;

	ret = ast27xx_reset_line_assert(dev, id);
	if (ret != 0) {
		return ret;
	}

	return ast27xx_reset_line_deassert(dev, id);
}

static DEVICE_API(reset, ast27xx_reset_api) = {
	.status = ast27xx_reset_status,
	.line_assert = ast27xx_reset_line_assert,
	.line_deassert = ast27xx_reset_line_deassert,
	.line_toggle = ast27xx_reset_line_toggle,
};

#define AST27XX_RESET_INIT(n)                                                                      \
	static const struct reset_ast27xx_config reset_ast27xx_config_##n = {                      \
		.syscon = DEVICE_DT_GET(DT_INST_PARENT(n)),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &reset_ast27xx_config_##n, PRE_KERNEL_1,        \
			      CONFIG_RESET_INIT_PRIORITY, &ast27xx_reset_api);

DT_INST_FOREACH_STATUS_OKAY(AST27XX_RESET_INIT)
