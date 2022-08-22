/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#include "app_version.h"

#include <zephyr/logging/log.h>

#include "MCP356X.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

struct spi_dt_spec bus = SPI_DT_SPEC_GET(DT_NODELABEL(myadc), SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0);


static void mcp356x_data11_get(struct mcp356x_data11 * value)
{
	uint8_t tx[5];
	uint8_t rx[5];
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, MCP356X_REG_ADC_DATA, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[2] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[3] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[4] = 0; // Need to write in order to read. Exchange 0 for reading data.
	
	printk("rx: %02x %02x %02x %02x %02x\n", rx[0], rx[1], rx[2], rx[3], rx[4]);

	spi_transceive_dt(&bus, &tx_buf, &rx_buf);
	value->channel = rx[1] >> 4;
	uint8_t sign = rx[1] & 0x01;
	value->value = (rx[2] << 16) | (rx[3] << 8) | (rx[4] << 0);
	if (sign != 0)
	{
		value->value -= 16777215;
	}
}


#define VREF  2048
#define MY_GAIN MCP356X_CFG_2_GAIN_X_033

void main(void)
{
	int ret;
	const struct device *sensor;
	const struct device *myadc;

	printk("Zephyr Example Application %s\n", APP_VERSION_STR);


	sensor = DEVICE_DT_GET(DT_NODELABEL(examplesensor0));
	myadc = DEVICE_DT_GET(DT_NODELABEL(myadc));
	
	if (!spi_is_ready(&bus))
	{
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (!device_is_ready(myadc)) {
		LOG_ERR("ADC not ready");
		return;
	}

	if (!device_is_ready(sensor)) {
		LOG_ERR("examplesensor0 not ready");
		return;
	}



	while (1) {
		struct sensor_value val;

		ret = sensor_sample_fetch(sensor);
		if (ret < 0) {
			LOG_ERR("Could not fetch sample (%d)", ret);
			return;
		}

		ret = sensor_channel_get(sensor, SENSOR_CHAN_PROX, &val);
		if (ret < 0) {
			LOG_ERR("Could not get sample (%d)", ret);
			return;
		}

		printk("Sensor value: %d\n", val.val1);

		struct mcp356x_data11 data;
		mcp356x_data11_get(&data);
		printk("Voltage: %02x %08x %08i\n", data.channel, data.value, MCP356X_raw_to_mv(data.value, VREF, MY_GAIN));

		k_sleep(K_MSEC(1000));
	}
}

