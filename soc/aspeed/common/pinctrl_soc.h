/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ASPEED_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ASPEED_COMMON_PINCTRL_SOC_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/devicetree.h>

/* One pin function, from a node of the ASPEED G7 pin controller */
typedef struct pinctrl_soc_pin {
	uint32_t mux;
	uint32_t bias_disable_field;
	uint32_t drive_strength_field;
	uint16_t ball;
	uint16_t drive_strength;
	bool bias_disable;
	bool has_drive_strength;
} pinctrl_soc_pin_t;

#define Z_PINCTRL_ASPEED_G7_PIN_INIT(pin)                                                          \
	{                                                                                          \
		.mux = DT_PROP_OR(pin, aspeed_mux, 0),                                             \
		.bias_disable_field = DT_PROP_OR(pin, aspeed_bias_disable_field, 0),               \
		.drive_strength_field = DT_PROP_OR(pin, aspeed_drive_strength_field, 0),           \
		.ball = DT_PROP(pin, aspeed_ball),                                                 \
		.drive_strength = DT_PROP_OR(pin, drive_strength, 0),                              \
		.bias_disable = DT_PROP(pin, bias_disable),                                        \
		.has_drive_strength = DT_NODE_HAS_PROP(pin, drive_strength),                       \
	},

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	Z_PINCTRL_ASPEED_G7_PIN_INIT(DT_PHANDLE_BY_IDX(node_id, prop, idx))

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

#endif /* ZEPHYR_SOC_ASPEED_COMMON_PINCTRL_SOC_H_ */
