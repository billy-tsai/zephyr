/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ASPEED_G7_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ASPEED_G7_PINCTRL_H_

/*
 * Register field of an ASPEED G7 pin control setting, an offset in the SCU
 * syscon of the pin controller:
 * bits [19:0]  register offset
 * bits [27:20] bit offset of the field
 * bits [31:28] function to select (ASPEED_G7_PINMUX) or field width in bits
 *              (ASPEED_G7_PINCFG)
 */
#define ASPEED_G7_PINMUX(reg, bit, func)   ((reg) | ((bit) << 20) | ((func) << 28))
#define ASPEED_G7_PINCFG(reg, bit, width) ((reg) | ((bit) << 20) | ((width) << 28))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ASPEED_G7_PINCTRL_H_ */
