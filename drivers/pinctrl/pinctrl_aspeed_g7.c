/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_soc1_pinctrl

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#define FIELD_REG   GENMASK(19, 0)
#define FIELD_BIT   GENMASK(27, 20)
#define FIELD_UPPER GENMASK(31, 28)

#define MUX_WIDTH 3U

#define NUM_BALLS        DT_INST_PROP(0, aspeed_num_balls)
#define DRIVE_STEP_MA    DT_INST_PROP(0, aspeed_drive_strength_step_ma)
#define DRIVE_FIRST_STEP DT_INST_PROP(0, aspeed_drive_strength_first_step)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "one pin controller is supported");
BUILD_ASSERT(DRIVE_STEP_MA > 0, "aspeed,drive-strength-step-ma must be non-zero");

static const struct device *const syscon = DEVICE_DT_GET(DT_INST_PARENT(0));
static struct k_spinlock lock;

#ifdef CONFIG_PINCTRL_ASPEED_G7_CONFLICT_CHECK
/* Function selected on each ball, offset by one so that 0 means unclaimed */
static uint32_t ball_owner[NUM_BALLS];
#endif

static int drive_strength_code(const pinctrl_soc_pin_t *pin, uint32_t *code)
{
	uint32_t width = FIELD_GET(FIELD_UPPER, pin->drive_strength_field);
	uint32_t steps;

	if (pin->drive_strength_field == 0U || width == 0U ||
	    (pin->drive_strength % DRIVE_STEP_MA) != 0U) {
		return -EINVAL;
	}

	steps = pin->drive_strength / DRIVE_STEP_MA;
	if (steps < DRIVE_FIRST_STEP || (steps - DRIVE_FIRST_STEP) > GENMASK(width - 1U, 0U)) {
		return -EINVAL;
	}

	*code = steps - DRIVE_FIRST_STEP;

	return 0;
}

static int validate_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t code;

	if (pin->ball >= NUM_BALLS) {
		return -EINVAL;
	}

	if (pin->bias_disable && pin->bias_disable_field == 0U) {
		return -ENOTSUP;
	}

	if (pin->has_drive_strength && drive_strength_code(pin, &code) != 0) {
		return -EINVAL;
	}

#ifdef CONFIG_PINCTRL_ASPEED_G7_CONFLICT_CHECK
	if (ball_owner[pin->ball] != 0U && ball_owner[pin->ball] != pin->mux + 1U) {
		return -EBUSY;
	}
#endif

	return 0;
}

static int update_field(uint32_t field, uint32_t width, uint32_t value)
{
	uint32_t reg = FIELD_GET(FIELD_REG, field);
	uint32_t mask = GENMASK(width - 1U, 0U) << FIELD_GET(FIELD_BIT, field);
	uint32_t val;
	int ret;

	ret = syscon_read_reg(syscon, reg, &val);
	if (ret != 0) {
		return ret;
	}

	val = (val & ~mask) | ((value << FIELD_GET(FIELD_BIT, field)) & mask);

	return syscon_write_reg(syscon, reg, val);
}

static int configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t code;
	int ret;

	if (pin->mux != 0U) {
		ret = update_field(pin->mux, MUX_WIDTH, FIELD_GET(FIELD_UPPER, pin->mux));
		if (ret != 0) {
			return ret;
		}
	}

	if (pin->bias_disable) {
		ret = update_field(pin->bias_disable_field,
				   FIELD_GET(FIELD_UPPER, pin->bias_disable_field), 1U);
		if (ret != 0) {
			return ret;
		}
	}

	if (pin->has_drive_strength) {
		ret = drive_strength_code(pin, &code);
		if (ret != 0) {
			return ret;
		}

		ret = update_field(pin->drive_strength_field,
				   FIELD_GET(FIELD_UPPER, pin->drive_strength_field), code);
		if (ret != 0) {
			return ret;
		}
	}

#ifdef CONFIG_PINCTRL_ASPEED_G7_CONFLICT_CHECK
	ball_owner[pin->ball] = pin->mux + 1U;
#endif

	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	k_spinlock_key_t key;
	int ret = 0;

	ARG_UNUSED(reg);

	key = k_spin_lock(&lock);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		ret = validate_pin(&pins[i]);
		if (ret != 0) {
			goto out;
		}
	}

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		ret = configure_pin(&pins[i]);
		if (ret != 0) {
			goto out;
		}
	}

out:
	k_spin_unlock(&lock, key);

	return ret;
}
