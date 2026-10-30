/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_mailbox

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define AST27XX_MBOX_TRIGGER      0x00U
#define AST27XX_MBOX_ENABLE       0x04U
#define AST27XX_MBOX_STATUS       0x08U
#define AST27XX_MBOX_DATA         0x10U
#define AST27XX_MBOX_CHANNELS     4U
#define AST27XX_MBOX_MSG_SIZE     32U
#define AST27XX_MBOX_MSG_WORDS    8U
#define AST27XX_MBOX_CHANNEL_MASK BIT_MASK(AST27XX_MBOX_CHANNELS)

struct ast27xx_mbox_config {
	uintptr_t tx;
	uintptr_t rx;
};

struct ast27xx_mbox_data {
	struct k_spinlock lock;
	mbox_callback_t callback[AST27XX_MBOX_CHANNELS];
	void *user_data[AST27XX_MBOX_CHANNELS];
	uint32_t enabled_mask;
	uint32_t rx_buffer[AST27XX_MBOX_MSG_WORDS];
};

static K_SEM_DEFINE(ast27xx_mbox_wake, 0U, 1U);
static atomic_t ast27xx_mbox_enabled_channels;

static int ast27xx_mbox_send(const struct device *dev, mbox_channel_id_t channel,
			     const struct mbox_msg *msg)
{
	const struct ast27xx_mbox_config *config = dev->config;
	struct ast27xx_mbox_data *data = dev->data;
	uint32_t words[AST27XX_MBOX_MSG_WORDS] = {0U};
	k_spinlock_key_t key;

	if (channel >= AST27XX_MBOX_CHANNELS) {
		return -EINVAL;
	}

	if (msg != NULL) {
		if (msg->size > AST27XX_MBOX_MSG_SIZE) {
			return -EMSGSIZE;
		}
		if (msg->size > 0U) {
			if (msg->data == NULL) {
				return -EINVAL;
			}
			memcpy(words, msg->data, msg->size);
		}
	}

	key = k_spin_lock(&data->lock);
	if ((sys_read32(config->tx + AST27XX_MBOX_STATUS) & BIT(channel)) != 0U) {
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	if (msg != NULL) {
		uintptr_t addr = config->tx + AST27XX_MBOX_DATA + AST27XX_MBOX_MSG_SIZE * channel;

		for (size_t i = 0U; i < DIV_ROUND_UP(msg->size, sizeof(uint32_t)); i++) {
			sys_write32(words[i], addr + i * sizeof(uint32_t));
		}
	}

	sys_write32(BIT(channel), config->tx + AST27XX_MBOX_TRIGGER);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ast27xx_mbox_register_callback(const struct device *dev, mbox_channel_id_t channel,
					  mbox_callback_t callback, void *user_data)
{
	struct ast27xx_mbox_data *data = dev->data;
	k_spinlock_key_t key;

	if (channel >= AST27XX_MBOX_CHANNELS) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	data->callback[channel] = callback;
	data->user_data[channel] = user_data;
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ast27xx_mbox_mtu_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return AST27XX_MBOX_MSG_SIZE;
}

static uint32_t ast27xx_mbox_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return AST27XX_MBOX_CHANNELS;
}

static int ast27xx_mbox_set_enabled(const struct device *dev, mbox_channel_id_t channel,
				    bool enable)
{
	const struct ast27xx_mbox_config *config = dev->config;
	struct ast27xx_mbox_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t reg;

	if (channel >= AST27XX_MBOX_CHANNELS) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	if (((data->enabled_mask & BIT(channel)) != 0U) == enable) {
		k_spin_unlock(&data->lock, key);
		return -EALREADY;
	}

	reg = sys_read32(config->rx + AST27XX_MBOX_ENABLE);
	if (enable) {
		reg |= BIT(channel);
		data->enabled_mask |= BIT(channel);
		atomic_inc(&ast27xx_mbox_enabled_channels);
	} else {
		reg &= ~BIT(channel);
		data->enabled_mask &= ~BIT(channel);
		atomic_dec(&ast27xx_mbox_enabled_channels);
	}
	sys_write32(reg, config->rx + AST27XX_MBOX_ENABLE);
	k_spin_unlock(&data->lock, key);
	k_sem_give(&ast27xx_mbox_wake);

	return 0;
}

static void ast27xx_mbox_poll(const struct device *dev)
{
	const struct ast27xx_mbox_config *config = dev->config;
	struct ast27xx_mbox_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t status = 0U;

	if (data->enabled_mask != 0U) {
		status = sys_read32(config->rx + AST27XX_MBOX_STATUS) & data->enabled_mask;
	}
	k_spin_unlock(&data->lock, key);

	for (mbox_channel_id_t channel = 0U; channel < AST27XX_MBOX_CHANNELS; channel++) {
		struct mbox_msg msg = {
			.data = data->rx_buffer,
			.size = AST27XX_MBOX_MSG_SIZE,
		};
		mbox_callback_t callback;
		void *user_data;
		uintptr_t addr;

		if ((status & BIT(channel)) == 0U) {
			continue;
		}

		key = k_spin_lock(&data->lock);
		callback = data->callback[channel];
		user_data = data->user_data[channel];
		if ((data->enabled_mask & BIT(channel)) == 0U || callback == NULL) {
			k_spin_unlock(&data->lock, key);
			continue;
		}

		addr = config->rx + AST27XX_MBOX_DATA + AST27XX_MBOX_MSG_SIZE * channel;
		for (size_t i = 0U; i < ARRAY_SIZE(data->rx_buffer); i++) {
			data->rx_buffer[i] = sys_read32(addr + i * sizeof(uint32_t));
		}
		k_spin_unlock(&data->lock, key);

		callback(dev, channel, user_data, &msg);
		sys_write32(BIT(channel), config->rx + AST27XX_MBOX_STATUS);
	}
}

static int ast27xx_mbox_init(const struct device *dev)
{
	const struct ast27xx_mbox_config *config = dev->config;

	sys_write32(0U, config->rx + AST27XX_MBOX_ENABLE);
	sys_write32(AST27XX_MBOX_CHANNEL_MASK, config->rx + AST27XX_MBOX_STATUS);

	return 0;
}

static DEVICE_API(mbox, ast27xx_mbox_driver_api) = {
	.send = ast27xx_mbox_send,
	.register_callback = ast27xx_mbox_register_callback,
	.mtu_get = ast27xx_mbox_mtu_get,
	.max_channels_get = ast27xx_mbox_max_channels_get,
	.set_enabled = ast27xx_mbox_set_enabled,
};

#define AST27XX_MBOX_DEFINE(n)                                                                     \
	static const struct ast27xx_mbox_config ast27xx_mbox_config_##n = {                        \
		.tx = DT_INST_REG_ADDR_BY_NAME(n, tx),                                             \
		.rx = DT_INST_REG_ADDR_BY_NAME(n, rx),                                             \
	};                                                                                         \
	static struct ast27xx_mbox_data ast27xx_mbox_data_##n;                                     \
	DEVICE_DT_INST_DEFINE(n, ast27xx_mbox_init, NULL, &ast27xx_mbox_data_##n,                  \
			      &ast27xx_mbox_config_##n, POST_KERNEL, CONFIG_MBOX_INIT_PRIORITY,    \
			      &ast27xx_mbox_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AST27XX_MBOX_DEFINE)

#define AST27XX_MBOX_DEVICE(n) DEVICE_DT_INST_GET(n),

static const struct device *const ast27xx_mbox_devices[] = {
	DT_INST_FOREACH_STATUS_OKAY(AST27XX_MBOX_DEVICE)};

static void ast27xx_mbox_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_timeout_t timeout = K_FOREVER;

		if (atomic_get(&ast27xx_mbox_enabled_channels) != 0) {
			for (size_t i = 0U; i < ARRAY_SIZE(ast27xx_mbox_devices); i++) {
				ast27xx_mbox_poll(ast27xx_mbox_devices[i]);
			}
			timeout = K_MSEC(CONFIG_MBOX_AST27XX_POLL_INTERVAL_MS);
		}

		(void)k_sem_take(&ast27xx_mbox_wake, timeout);
	}
}

K_THREAD_DEFINE(ast27xx_mbox_poll_thread, CONFIG_MBOX_AST27XX_THREAD_STACK_SIZE,
		ast27xx_mbox_thread, NULL, NULL, NULL, CONFIG_MBOX_AST27XX_THREAD_PRIORITY, 0U, 0);
