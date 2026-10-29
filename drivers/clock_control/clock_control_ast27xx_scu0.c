/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast27xx_scu0_clock

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
#include <zephyr/sys/util.h>

#define SCU0_HW_STRAP     0x010U
#define SCU0_CLK_STOP     0x240U
#define SCU0_CLK_STOP_CLR 0x244U
#define SCU0_CLK_SEL1     0x280U
#define SCU0_HPLL_PARAM   0x300U
#define SCU0_MPLL_PARAM   0x310U

#define SCU0_CLKIN_RATE MHZ(25)

#define HW_STRAP_CPU_CLK GENMASK(4, 2)
#define PLL_MULT         GENMASK(12, 0)
#define PLL_DIV          GENMASK(18, 13)
#define PLL_POST_DIV     GENMASK(22, 19)
#define PLL_BYPASS       BIT(24)
#define CLK_SEL1_EMMC_HPLL BIT(11)
#define CLK_SEL1_EMMC_DIV  GENMASK(14, 12)

struct clock_ast27xx_scu0_config {
	const struct device *syscon;
};

static int ast27xx_scu0_clock_gate(const struct device *dev, clock_control_subsys_t sys,
				 uint32_t reg)
{
	const struct clock_ast27xx_scu0_config *config = dev->config;
	uint32_t clk_id = (uint32_t)(uintptr_t)sys;

	if (clk_id >= SCU0_CLK_GATE_NUM) {
		return -EINVAL;
	}

	/* This gate has the opposite polarity to the other gates and is not supported yet. */
	if (clk_id == SCU0_CLK_GATE_PORTAUSB2CLK) {
		return -ENOTSUP;
	}

	/* Write only the requested bit so other RW1S clock gates are unaffected. */
	return syscon_write_reg(config->syscon, reg, BIT(clk_id));
}

static int ast27xx_scu0_clock_control_on(const struct device *dev, clock_control_subsys_t sys)
{
	return ast27xx_scu0_clock_gate(dev, sys, SCU0_CLK_STOP_CLR);
}

static int ast27xx_scu0_clock_control_off(const struct device *dev, clock_control_subsys_t sys)
{
	return ast27xx_scu0_clock_gate(dev, sys, SCU0_CLK_STOP);
}

static int ast27xx_scu0_pll_rate(const struct device *syscon, uint32_t param, uint64_t *rate)
{
	uint32_t mult;
	uint32_t div;
	uint32_t post;
	uint32_t val;
	int ret;

	ret = syscon_read_reg(syscon, param, &val);
	if (ret != 0) {
		return ret;
	}

	if ((val & PLL_BYPASS) != 0U) {
		*rate = SCU0_CLKIN_RATE;
		return 0;
	}

	mult = FIELD_GET(PLL_MULT, val);
	div = FIELD_GET(PLL_DIV, val) + 1U;
	post = FIELD_GET(PLL_POST_DIV, val) + 1U;
	if (param == SCU0_HPLL_PARAM) {
		mult += 1U;
		div *= 2U;
	}
	*rate = (uint64_t)SCU0_CLKIN_RATE * mult / div / post;

	return 0;
}

static int ast27xx_scu0_hpll_rate(const struct device *syscon, uint64_t *rate)
{
	uint32_t strap;
	int ret;

	ret = syscon_read_reg(syscon, SCU0_HW_STRAP, &strap);
	if (ret != 0) {
		return ret;
	}

	/* These strap modes fix HPLL independently of the programmable PLL parameters. */
	switch (FIELD_GET(HW_STRAP_CPU_CLK, strap)) {
	case 2U:
		*rate = MHZ(1800);
		break;
	case 3U:
		*rate = MHZ(1700);
		break;
	case 6U:
		*rate = MHZ(1200);
		break;
	case 7U:
		*rate = MHZ(800);
		break;
	default:
		return ast27xx_scu0_pll_rate(syscon, SCU0_HPLL_PARAM, rate);
	}

	return 0;
}

static int ast27xx_scu0_emmc_rate(const struct device *syscon, uint64_t *rate)
{
	uint32_t sel;
	int ret;

	ret = syscon_read_reg(syscon, SCU0_CLK_SEL1, &sel);
	if (ret != 0) {
		return ret;
	}

	if ((sel & CLK_SEL1_EMMC_HPLL) != 0U) {
		ret = ast27xx_scu0_hpll_rate(syscon, rate);
	} else {
		ret = ast27xx_scu0_pll_rate(syscon, SCU0_MPLL_PARAM, rate);
	}
	if (ret == 0) {
		*rate /= 4U * ((FIELD_GET(CLK_SEL1_EMMC_DIV, sel) + 1U) * 2U);
	}

	return ret;
}

static int ast27xx_scu0_clock_control_get_rate(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t *rate)
{
	const struct clock_ast27xx_scu0_config *config = dev->config;
	uint32_t clk_id = (uint32_t)(uintptr_t)sys;
	uint64_t pll;
	int ret;

	switch (clk_id) {
	case SCU0_CLK_HPLL:
		ret = ast27xx_scu0_hpll_rate(config->syscon, &pll);
		break;
	case SCU0_CLK_MPLL:
		ret = ast27xx_scu0_pll_rate(config->syscon, SCU0_MPLL_PARAM, &pll);
		break;
	case SCU0_CLK_GATE_EMMCCLK:
		ret = ast27xx_scu0_emmc_rate(config->syscon, &pll);
		break;
	default:
		return -ENOTSUP;
	}

	if (ret == 0) {
		*rate = (uint32_t)pll;
	}

	return ret;
}

static DEVICE_API(clock_control, ast27xx_scu0_clock_api) = {
	.on = ast27xx_scu0_clock_control_on,
	.off = ast27xx_scu0_clock_control_off,
	.get_rate = ast27xx_scu0_clock_control_get_rate,
};

#define AST27XX_SCU0_CLOCK_INIT(n)                                                                 \
	static const struct clock_ast27xx_scu0_config clock_ast27xx_scu0_config_##n = {            \
		.syscon = DEVICE_DT_GET(DT_INST_PARENT(n)),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &clock_ast27xx_scu0_config_##n, PRE_KERNEL_1,   \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &ast27xx_scu0_clock_api);

DT_INST_FOREACH_STATUS_OKAY(AST27XX_SCU0_CLOCK_INIT)
