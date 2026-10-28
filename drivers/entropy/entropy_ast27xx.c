/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_rng

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define AST27XX_RNG_CTRL       0xf0U
#define AST27XX_RNG_DATA       0xf4U
#define AST27XX_RNG_DATA_VALID BIT(31)
#define AST27XX_RNG_SOURCE     BIT(5)

#define AST27XX_RNG_TIMEOUT_US    1000U
#define AST27XX_RNG_DISCARD_WORDS 32U

struct ast27xx_rng_config {
	const struct device *syscon;
};

static int ast27xx_rng_try_read(const struct device *dev, uint32_t *word)
{
	const struct ast27xx_rng_config *config = dev->config;
	uint32_t ctrl;
	unsigned int key;
	int ret;

	/* Keep another caller from consuming data between the valid check and read. */
	key = irq_lock();
	ret = syscon_read_reg(config->syscon, AST27XX_RNG_CTRL, &ctrl);
	if (ret != 0) {
		goto out;
	}

	if ((ctrl & AST27XX_RNG_DATA_VALID) == 0U) {
		ret = -EAGAIN;
		goto out;
	}

	ret = syscon_read_reg(config->syscon, AST27XX_RNG_DATA, word);

out:
	irq_unlock(key);
	return ret;
}

static int ast27xx_rng_read_word(const struct device *dev, uint32_t *word, bool busywait)
{
	bool ready;
	int ret;

	if (!busywait) {
		return ast27xx_rng_try_read(dev, word);
	}

	ready = WAIT_FOR((ret = ast27xx_rng_try_read(dev, word)) != -EAGAIN,
			 AST27XX_RNG_TIMEOUT_US, k_busy_wait(1U));
	if (!ready) {
		return -ETIMEDOUT;
	}

	return ret;
}

static int ast27xx_rng_get_entropy_isr(const struct device *dev, uint8_t *buffer,
				     uint16_t length, uint32_t flags)
{
	bool busywait = (flags & ENTROPY_BUSYWAIT) != 0U;
	size_t count = 0U;
	size_t to_copy;
	uint32_t word;
	int ret;

	while (count < length) {
		ret = ast27xx_rng_read_word(dev, &word, busywait);
		if (ret == -EAGAIN) {
			break;
		}
		if (ret != 0) {
			return ret;
		}

		to_copy = MIN(length - count, sizeof(word));
		memcpy(buffer + count, &word, to_copy);
		count += to_copy;
	}

	return (int)count;
}

static int ast27xx_rng_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	int ret = ast27xx_rng_get_entropy_isr(dev, buffer, length, ENTROPY_BUSYWAIT);

	return ret < 0 ? ret : 0;
}

static int ast27xx_rng_init(const struct device *dev)
{
	const struct ast27xx_rng_config *config = dev->config;
	uint32_t word;
	int ret;

	if (!device_is_ready(config->syscon)) {
		return -ENODEV;
	}

	ret = syscon_write_reg(config->syscon, AST27XX_RNG_CTRL, AST27XX_RNG_SOURCE);
	if (ret != 0) {
		return ret;
	}

	/* The first word read after power-on is the same on every boot. */
	for (uint32_t i = 0U; i < AST27XX_RNG_DISCARD_WORDS; i++) {
		ret = ast27xx_rng_read_word(dev, &word, true);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static DEVICE_API(entropy, ast27xx_rng_api) = {
	.get_entropy = ast27xx_rng_get_entropy,
	.get_entropy_isr = ast27xx_rng_get_entropy_isr,
};

static const struct ast27xx_rng_config ast27xx_rng_config = {
	.syscon = DEVICE_DT_GET(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, ast27xx_rng_init, NULL, NULL, &ast27xx_rng_config, PRE_KERNEL_2,
		      CONFIG_ENTROPY_INIT_PRIORITY, &ast27xx_rng_api);
