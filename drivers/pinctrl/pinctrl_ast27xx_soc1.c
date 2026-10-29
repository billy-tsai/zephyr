/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_soc1_pinctrl

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/dt-bindings/pinctrl/ast27xx-soc1-pinctrl.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#define SCU1_MUX_U       0x450U
#define SCU1_PULL_DOWN_U 0x494U
#define SCU1_FWSPI_DRIVE 0x4E0U

#define PINMUX_BALL_MASK GENMASK(15U, 8U)
#define PINMUX_FUNC_MASK GENMASK(2U, 0U)
#define PINCFG_MASK                                                                             \
	(AST27XX_SOC1_PINCFG_BIAS_DISABLE | AST27XX_SOC1_PINCFG_DRIVE_VALID |                     \
	 AST27XX_SOC1_PINCFG_DRIVE_MASK)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1U,
	     "Only one AST2700 SoC1 pin controller is supported");

struct ast27xx_soc1_field {
	uint16_t offset;
	uint32_t mask;
};

struct ast27xx_soc1_pin_desc {
	struct ast27xx_soc1_field mux;
	struct ast27xx_soc1_field bias;
	struct ast27xx_soc1_field drive;
	uint8_t functions;
};

static const struct ast27xx_soc1_pin_desc pin_descs[] = {
	[AST27XX_SOC1_BALL_C25] = {
		.drive = {SCU1_FWSPI_DRIVE, GENMASK(1U, 0U)},
		.functions = BIT(0U),
	},
	[AST27XX_SOC1_BALL_C24] = {
		.drive = {SCU1_FWSPI_DRIVE, GENMASK(7U, 6U)},
		.functions = BIT(0U),
	},
	[AST27XX_SOC1_BALL_A17] = {
		.drive = {SCU1_FWSPI_DRIVE, GENMASK(9U, 8U)},
		.functions = BIT(0U),
	},
	[AST27XX_SOC1_BALL_A16] = {
		.drive = {SCU1_FWSPI_DRIVE, GENMASK(11U, 10U)},
		.functions = BIT(0U),
	},
	[AST27XX_SOC1_BALL_M16] = {
		.mux = {SCU1_MUX_U, GENMASK(26U, 24U)},
		.bias = {SCU1_PULL_DOWN_U, BIT(6U)},
		.drive = {SCU1_FWSPI_DRIVE, GENMASK(13U, 12U)},
		.functions = BIT(0U) | BIT(1U),
	},
	[AST27XX_SOC1_BALL_A15] = {
		.mux = {SCU1_MUX_U, GENMASK(30U, 28U)},
		.bias = {SCU1_PULL_DOWN_U, BIT(7U)},
		.drive = {SCU1_FWSPI_DRIVE, GENMASK(15U, 14U)},
		.functions = BIT(0U) | BIT(1U),
	},
};

static const struct device *const syscon = DEVICE_DT_GET(DT_INST_PARENT(0));
static struct k_spinlock pinctrl_lock;

static int ast27xx_soc1_validate_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t ball = FIELD_GET(PINMUX_BALL_MASK, pin->pinmux);
	uint32_t func = FIELD_GET(PINMUX_FUNC_MASK, pin->pinmux);
	const struct ast27xx_soc1_pin_desc *desc;

	if ((pin->pinmux & ~(PINMUX_BALL_MASK | PINMUX_FUNC_MASK)) != 0U ||
	    (pin->pincfg & ~PINCFG_MASK) != 0U || ball >= ARRAY_SIZE(pin_descs)) {
		return -EINVAL;
	}

	desc = &pin_descs[ball];
	if ((desc->functions & BIT(func)) == 0U) {
		return -EINVAL;
	}

	if (((pin->pincfg & AST27XX_SOC1_PINCFG_BIAS_DISABLE) != 0U && desc->bias.mask == 0U) ||
	    ((pin->pincfg & AST27XX_SOC1_PINCFG_DRIVE_VALID) != 0U && desc->drive.mask == 0U)) {
		return -ENOTSUP;
	}

	return 0;
}

static int ast27xx_soc1_write_field(const struct ast27xx_soc1_field *field, uint32_t value)
{
	uint32_t val;
	int ret;

	if (field->mask == 0U) {
		return 0;
	}

	ret = syscon_read_reg(syscon, field->offset, &val);
	if (ret != 0) {
		return ret;
	}

	val = (val & ~field->mask) | FIELD_PREP(field->mask, value);

	return syscon_write_reg(syscon, field->offset, val);
}

static int ast27xx_soc1_configure_pin(const pinctrl_soc_pin_t *pin)
{
	const struct ast27xx_soc1_pin_desc *desc =
		&pin_descs[FIELD_GET(PINMUX_BALL_MASK, pin->pinmux)];
	int ret;

	ret = ast27xx_soc1_write_field(&desc->mux, FIELD_GET(PINMUX_FUNC_MASK, pin->pinmux));
	if (ret != 0) {
		return ret;
	}

	if ((pin->pincfg & AST27XX_SOC1_PINCFG_BIAS_DISABLE) != 0U) {
		ret = ast27xx_soc1_write_field(&desc->bias, 1U);
		if (ret != 0) {
			return ret;
		}
	}

	if ((pin->pincfg & AST27XX_SOC1_PINCFG_DRIVE_VALID) != 0U) {
		ret = ast27xx_soc1_write_field(
			&desc->drive, FIELD_GET(AST27XX_SOC1_PINCFG_DRIVE_MASK, pin->pincfg));
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	k_spinlock_key_t key;
	int ret = 0;

	ARG_UNUSED(reg);

	if (pins == NULL && pin_cnt > 0U) {
		return -EINVAL;
	}

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		ret = ast27xx_soc1_validate_pin(&pins[i]);
		if (ret != 0) {
			return ret;
		}
	}

	if (!device_is_ready(syscon)) {
		return -ENODEV;
	}

	/* Keep consumers from interleaving the read-modify-write operations of a state. */
	key = k_spin_lock(&pinctrl_lock);
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		ret = ast27xx_soc1_configure_pin(&pins[i]);
		if (ret != 0) {
			break;
		}
	}
	k_spin_unlock(&pinctrl_lock, key);

	return ret;
}
