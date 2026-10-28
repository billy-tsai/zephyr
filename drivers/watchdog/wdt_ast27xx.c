/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_wdt

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define AST27XX_WDT_RELOAD        0x04U
#define AST27XX_WDT_RESTART       0x08U
#define AST27XX_WDT_CTRL          0x0cU
#define AST27XX_WDT_CLEAR_STATUS  0x14U

#define AST27XX_WDT_RESTART_MAGIC 0x4755U
#define AST27XX_WDT_RESET_MODE    GENMASK(6, 5)
#define AST27XX_WDT_RESET_BY_SOC  BIT(4)
#define AST27XX_WDT_RESET_SYSTEM  BIT(1)
#define AST27XX_WDT_ENABLE        BIT(0)
#define AST27XX_WDT_CLEAR_TIMEOUT BIT(0)
#define AST27XX_WDT_TICKS_PER_MS  1000U

struct ast27xx_wdt_config {
	uintptr_t base;
};

struct ast27xx_wdt_data {
	struct k_spinlock lock;
	uint32_t reload;
	bool timeout_installed;
};

static int ast27xx_wdt_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	const struct ast27xx_wdt_config *config = dev->config;
	struct ast27xx_wdt_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int ret = 0;

	if ((sys_read32(config->base + AST27XX_WDT_CTRL) & AST27XX_WDT_ENABLE) != 0U) {
		ret = -EBUSY;
		goto out;
	}

	if (data->timeout_installed) {
		ret = -ENOMEM;
		goto out;
	}

	if (cfg->window.min != 0U || cfg->window.max == 0U ||
	    cfg->window.max > UINT32_MAX / AST27XX_WDT_TICKS_PER_MS) {
		ret = -EINVAL;
		goto out;
	}

	if (cfg->callback != NULL || cfg->flags != WDT_FLAG_RESET_SOC) {
		ret = -ENOTSUP;
		goto out;
	}

	data->reload = cfg->window.max * AST27XX_WDT_TICKS_PER_MS;
	data->timeout_installed = true;

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int ast27xx_wdt_setup(const struct device *dev, uint8_t options)
{
	const struct ast27xx_wdt_config *config = dev->config;
	struct ast27xx_wdt_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t ctrl = sys_read32(config->base + AST27XX_WDT_CTRL);
	int ret = 0;

	if ((ctrl & AST27XX_WDT_ENABLE) != 0U) {
		ret = -EBUSY;
		goto out;
	}

	if (!data->timeout_installed) {
		ret = -EINVAL;
		goto out;
	}

	if (options != 0U) {
		ret = -ENOTSUP;
		goto out;
	}

	sys_write32(data->reload, config->base + AST27XX_WDT_RELOAD);
	sys_write32(AST27XX_WDT_RESTART_MAGIC, config->base + AST27XX_WDT_RESTART);

	/* A pending timeout must be cleared before enabling system reset. */
	sys_write32(AST27XX_WDT_CLEAR_TIMEOUT, config->base + AST27XX_WDT_CLEAR_STATUS);
	/* Reset mode 0 resets the SoC blocks selected by the reset mask registers. */
	ctrl &= ~AST27XX_WDT_RESET_MODE;
	ctrl |= AST27XX_WDT_RESET_BY_SOC | AST27XX_WDT_RESET_SYSTEM | AST27XX_WDT_ENABLE;
	sys_write32(ctrl, config->base + AST27XX_WDT_CTRL);

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int ast27xx_wdt_feed(const struct device *dev, int channel_id)
{
	const struct ast27xx_wdt_config *config = dev->config;
	struct ast27xx_wdt_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	if (channel_id != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	if (!data->timeout_installed ||
	    (sys_read32(config->base + AST27XX_WDT_CTRL) & AST27XX_WDT_ENABLE) == 0U) {
		ret = -EINVAL;
		goto out;
	}

	sys_write32(AST27XX_WDT_RESTART_MAGIC, config->base + AST27XX_WDT_RESTART);

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int ast27xx_wdt_disable(const struct device *dev)
{
	const struct ast27xx_wdt_config *config = dev->config;
	struct ast27xx_wdt_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t ctrl = sys_read32(config->base + AST27XX_WDT_CTRL);
	int ret = 0;

	data->timeout_installed = false;

	if ((ctrl & AST27XX_WDT_ENABLE) == 0U) {
		ret = -EFAULT;
		goto out;
	}

	ctrl &= ~(AST27XX_WDT_ENABLE | AST27XX_WDT_RESET_SYSTEM);
	sys_write32(ctrl, config->base + AST27XX_WDT_CTRL);
	sys_write32(AST27XX_WDT_CLEAR_TIMEOUT, config->base + AST27XX_WDT_CLEAR_STATUS);

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int ast27xx_wdt_init(const struct device *dev)
{
	const struct ast27xx_wdt_config *config = dev->config;
	uint32_t ctrl;

	if (IS_ENABLED(CONFIG_WDT_DISABLE_AT_BOOT)) {
		ctrl = sys_read32(config->base + AST27XX_WDT_CTRL);
		ctrl &= ~(AST27XX_WDT_ENABLE | AST27XX_WDT_RESET_SYSTEM);
		sys_write32(ctrl, config->base + AST27XX_WDT_CTRL);
	}

	return 0;
}

static DEVICE_API(wdt, ast27xx_wdt_api) = {
	.install_timeout = ast27xx_wdt_install_timeout,
	.setup = ast27xx_wdt_setup,
	.feed = ast27xx_wdt_feed,
	.disable = ast27xx_wdt_disable,
};

#define AST27XX_WDT_DEFINE(inst)                                                                   \
	static const struct ast27xx_wdt_config ast27xx_wdt_config_##inst = {                       \
		.base = DT_INST_REG_ADDR(inst),                                                    \
	};                                                                                         \
	static struct ast27xx_wdt_data ast27xx_wdt_data_##inst;                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, ast27xx_wdt_init, NULL, &ast27xx_wdt_data_##inst,              \
			      &ast27xx_wdt_config_##inst, PRE_KERNEL_1,                            \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ast27xx_wdt_api);

DT_INST_FOREACH_STATUS_OKAY(AST27XX_WDT_DEFINE)
