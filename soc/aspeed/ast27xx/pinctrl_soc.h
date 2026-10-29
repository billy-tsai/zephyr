/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ASPEED_AST27XX_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ASPEED_AST27XX_PINCTRL_SOC_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#define AST27XX_SOC1_PINCFG_BIAS_DISABLE BIT(0U)
#define AST27XX_SOC1_PINCFG_DRIVE_VALID  BIT(1U)
#define AST27XX_SOC1_PINCFG_DRIVE_SHIFT  2U
#define AST27XX_SOC1_PINCFG_DRIVE_MASK   GENMASK(3U, 2U)

typedef struct {
	uint32_t pinmux;
	uint32_t pincfg;
} pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.pinmux = DT_PROP_BY_IDX(node_id, prop, idx),                                      \
		.pincfg = (DT_PROP(node_id, bias_disable) * AST27XX_SOC1_PINCFG_BIAS_DISABLE) |    \
			  (DT_NODE_HAS_PROP(node_id, aspeed_drive_strength) *                      \
			   AST27XX_SOC1_PINCFG_DRIVE_VALID) |                                      \
			  (DT_PROP_OR(node_id, aspeed_drive_strength, 0U) <<                       \
			   AST27XX_SOC1_PINCFG_DRIVE_SHIFT),                                       \
	},

#define Z_PINCTRL_STATE_GROUP_INIT(node_id, prop, idx)                                             \
	DT_FOREACH_CHILD_VARGS(DT_PHANDLE_BY_IDX(node_id, prop, idx), DT_FOREACH_PROP_ELEM,        \
			       pinmux, Z_PINCTRL_STATE_PIN_INIT)

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM_SEP(node_id, prop, Z_PINCTRL_STATE_GROUP_INIT, ())}

#endif /* ZEPHYR_SOC_ASPEED_AST27XX_PINCTRL_SOC_H_ */
