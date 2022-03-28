/* main.c - Application main entry point */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <sys/__assert.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

#include <drivers/gpio.h>
#include <drivers/spi.h>

#define SENSOR_1_NAME				"Temperature Sensor 1"
#define SENSOR_2_NAME				"Temperature Sensor 2"
#define SENSOR_3_NAME				"Humidity Sensor"

/* Sensor Internal Update Interval [seconds] */
#define SENSOR_1_UPDATE_IVAL			5
#define SENSOR_2_UPDATE_IVAL			12
#define SENSOR_3_UPDATE_IVAL			60

/* ESS error definitions */
#define ESS_ERR_WRITE_REJECT			0x80
#define ESS_ERR_COND_NOT_SUPP			0x81

/* ESS Trigger Setting conditions */
#define ESS_TRIGGER_INACTIVE			0x00
#define ESS_FIXED_TIME_INTERVAL			0x01
#define ESS_NO_LESS_THAN_SPECIFIED_TIME		0x02
#define ESS_VALUE_CHANGED			0x03
#define ESS_LESS_THAN_REF_VALUE			0x04
#define ESS_LESS_OR_EQUAL_TO_REF_VALUE		0x05
#define ESS_GREATER_THAN_REF_VALUE		0x06
#define ESS_GREATER_OR_EQUAL_TO_REF_VALUE	0x07
#define ESS_EQUAL_TO_REF_VALUE			0x08
#define ESS_NOT_EQUAL_TO_REF_VALUE		0x09





// ADC pin CSK ---orange--- (PA5,D13,20,SCK)
// ADC pin SDO ---yellow--- (PA6,D12,21,MISO)
// ADC pin SDI ---green---- (PA7,D11,22,MOSI)
#define MY_DEVICE_SPI "SPI_1"

// ADC pin CS ---blue--- (PA10,D3,51)
#define MY_DEVICE_CS_PIN 10
#define MY_DEVICE_CS_PORT "GPIOA"








static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const uint16_t *u16 = attr->user_data;
	uint16_t value = sys_cpu_to_le16(*u16);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
				 sizeof(value));
}

/* Environmental Sensing Service Declaration */

struct es_measurement {
	uint16_t flags; /* Reserved for Future Use */
	uint8_t sampling_func;
	uint32_t meas_period;
	uint32_t update_interval;
	uint8_t application;
	uint8_t meas_uncertainty;
};

struct temperature_sensor {
	int16_t temp_value;

	/* Valid Range */
	int16_t lower_limit;
	int16_t upper_limit;

	/* ES trigger setting - Value Notification condition */
	uint8_t condition;
	union {
		uint32_t seconds;
		int16_t ref_val; /* Reference temperature */
	};

	struct es_measurement meas;
};

struct humidity_sensor {
	int16_t humid_value;

	struct es_measurement meas;
};

static bool simulate_temp;
static struct temperature_sensor sensor_1 = {
		.temp_value = 1200,
		.lower_limit = -10000,
		.upper_limit = 10000,
		.condition = ESS_VALUE_CHANGED,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = SENSOR_1_UPDATE_IVAL,
		.meas.application = 0x1c,
		.meas.meas_uncertainty = 0x04,
};

static struct temperature_sensor sensor_2 = {
		.temp_value = 1800,
		.lower_limit = -1000,
		.upper_limit = 5000,
		.condition = ESS_VALUE_CHANGED,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = SENSOR_2_UPDATE_IVAL,
		.meas.application = 0x1b,
		.meas.meas_uncertainty = 0x04,
};

static struct humidity_sensor sensor_3 = {
		.humid_value = 6233,
		.meas.sampling_func = 0x02,
		.meas.meas_period = 0x0e10,
		.meas.update_interval = SENSOR_3_UPDATE_IVAL,
		.meas.application = 0x1c,
		.meas.meas_uncertainty = 0x01,
};

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	simulate_temp = value == BT_GATT_CCC_NOTIFY;
}

struct read_es_measurement_rp {
	uint16_t flags; /* Reserved for Future Use */
	uint8_t sampling_function;
	uint8_t measurement_period[3];
	uint8_t update_interval[3];
	uint8_t application;
	uint8_t measurement_uncertainty;
} __packed;

static ssize_t read_es_measurement(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr, void *buf,
				   uint16_t len, uint16_t offset)
{
	const struct es_measurement *value = attr->user_data;
	struct read_es_measurement_rp rsp;

	rsp.flags = sys_cpu_to_le16(value->flags);
	rsp.sampling_function = value->sampling_func;
	sys_put_le24(value->meas_period, rsp.measurement_period);
	sys_put_le24(value->update_interval, rsp.update_interval);
	rsp.application = value->application;
	rsp.measurement_uncertainty = value->meas_uncertainty;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &rsp,
				 sizeof(rsp));
}

static ssize_t read_temp_valid_range(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr, void *buf,
				     uint16_t len, uint16_t offset)
{
	const struct temperature_sensor *sensor = attr->user_data;
	uint16_t tmp[] = {sys_cpu_to_le16(sensor->lower_limit),
			  sys_cpu_to_le16(sensor->upper_limit)};

	return bt_gatt_attr_read(conn, attr, buf, len, offset, tmp,
				 sizeof(tmp));
}

struct es_trigger_setting_seconds {
	uint8_t condition;
	uint8_t sec[3];
} __packed;

struct es_trigger_setting_reference {
	uint8_t condition;
	int16_t ref_val;
} __packed;

static ssize_t read_temp_trigger_setting(struct bt_conn *conn,
					 const struct bt_gatt_attr *attr,
					 void *buf, uint16_t len,
					 uint16_t offset)
{
	const struct temperature_sensor *sensor = attr->user_data;

	switch (sensor->condition) {
	/* Operand N/A */
	case ESS_TRIGGER_INACTIVE:
		__fallthrough;
	case ESS_VALUE_CHANGED:
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
					 &sensor->condition,
					 sizeof(sensor->condition));
	/* Seconds */
	case ESS_FIXED_TIME_INTERVAL:
		__fallthrough;
	case ESS_NO_LESS_THAN_SPECIFIED_TIME: {
			struct es_trigger_setting_seconds rp;

			rp.condition = sensor->condition;
			sys_put_le24(sensor->seconds, rp.sec);

			return bt_gatt_attr_read(conn, attr, buf, len, offset,
						 &rp, sizeof(rp));
		}
	/* Reference temperature */
	default: {
			struct es_trigger_setting_reference rp;

			rp.condition = sensor->condition;
			rp.ref_val = sys_cpu_to_le16(sensor->ref_val);

			return bt_gatt_attr_read(conn, attr, buf, len, offset,
						 &rp, sizeof(rp));
		}
	}
}

static bool check_condition(uint8_t condition, int16_t old_val, int16_t new_val,
			    int16_t ref_val)
{
	switch (condition) {
	case ESS_TRIGGER_INACTIVE:
		return false;
	case ESS_FIXED_TIME_INTERVAL:
	case ESS_NO_LESS_THAN_SPECIFIED_TIME:
		/* TODO: Check time requirements */
		return false;
	case ESS_VALUE_CHANGED:
		return new_val != old_val;
	case ESS_LESS_THAN_REF_VALUE:
		return new_val < ref_val;
	case ESS_LESS_OR_EQUAL_TO_REF_VALUE:
		return new_val <= ref_val;
	case ESS_GREATER_THAN_REF_VALUE:
		return new_val > ref_val;
	case ESS_GREATER_OR_EQUAL_TO_REF_VALUE:
		return new_val >= ref_val;
	case ESS_EQUAL_TO_REF_VALUE:
		return new_val == ref_val;
	case ESS_NOT_EQUAL_TO_REF_VALUE:
		return new_val != ref_val;
	default:
		return false;
	}
}

static void update_temperature(struct bt_conn *conn,
			       const struct bt_gatt_attr *chrc, int16_t value,
			       struct temperature_sensor *sensor)
{
	bool notify = check_condition(sensor->condition,
				      sensor->temp_value, value,
				      sensor->ref_val);

	/* Update temperature value */
	sensor->temp_value = value;

	/* Trigger notification if conditions are met */
	if (notify) {
		value = sys_cpu_to_le16(sensor->temp_value);

		bt_gatt_notify(conn, chrc, &value, sizeof(value));
	}
}

BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

	/* Temperature Sensor 1 */
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &sensor_1.temp_value),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_1.meas),
	BT_GATT_CUD(SENSOR_1_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
			   read_temp_valid_range, NULL, &sensor_1),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING,
			   BT_GATT_PERM_READ, read_temp_trigger_setting,
			   NULL, &sensor_1),
	BT_GATT_CCC(temp_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Temperature Sensor 2 */
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &sensor_2.temp_value),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_2.meas),
	BT_GATT_CUD(SENSOR_2_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
			   read_temp_valid_range, NULL, &sensor_2),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING,
			   BT_GATT_PERM_READ, read_temp_trigger_setting,
			   NULL, &sensor_2),
	BT_GATT_CCC(temp_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Humidity Sensor */
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &sensor_3.humid_value),
	BT_GATT_CUD(SENSOR_3_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_3.meas),
);

static void ess_simulate(void)
{
	static uint8_t i;
	uint16_t val;

	if (!(i % SENSOR_1_UPDATE_IVAL)) {
		val = 1200 + i;
		update_temperature(NULL, &ess_svc.attrs[2], val, &sensor_1);
	}

	if (!(i % SENSOR_2_UPDATE_IVAL)) {
		val = 1800 + i;
		update_temperature(NULL, &ess_svc.attrs[9], val, &sensor_2);
	}

	if (!(i % SENSOR_3_UPDATE_IVAL)) {
		sensor_3.humid_value = 6233 + (i % 13);
	}

	if (!(i % INT8_MAX)) {
		i = 0U;
	}

	i++;
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x00, 0x03),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_ESS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}


#define CONFIG0_CLK_SEL_POS 4
#define CONFIG0_CLK_SEL_INT 0b11 << CONFIG0_CLK_SEL_POS
#define CONFIG0_CLK_SEL_EXT 0b00 << CONFIG0_CLK_SEL_POS
#define CONFIG0_ADC_MODE_POS 0
#define CONFIG0_ADC_MODE_CONV 0b11 << CONFIG0_ADC_MODE_POS


#define MCP356X_COMMAND_BYTE(addr, cmd, type) (((addr)<<6) | ((cmd)<<2) | ((type)<<0))
#define MCP356X_DEVICE_ADDRESS 0b01
#define MCP356X_SREAD 0b01
#define MCP356X_IWRITE 0b10
#define MCP356X_IREAD 0b11
#define MCP356X_ADDR_ADCDATA 0x0
#define MCP356X_ADDR_CONFIG0 0x1
#define MCP356X_ADDR_CONFIG1 0x2
#define MCP356X_ADDR_CONFIG2 0x3
#define MCP356X_ADDR_CONFIG3 0x4
#define MCP356X_ADDR_IRQ 0x05
#define MCP356X_ADDR_MUX 0x05

#define CONFIG1_OSR_POS 2
#define CONFIG1_OSR_32 0b0000 << CONFIG1_OSR_POS
#define CONFIG1_OSR_256 0b0011 << CONFIG1_OSR_POS

#define CONFIG3_CONV_MODE_POS 6
#define CONFIG3_CONV_MODE_CONTINUOUS 0b11 << CONFIG3_CONV_MODE_POS


#define IRQ_MODE_HIGH 0b01110111

static struct spi_config spi_cfg = {0};
static struct device *dev_spi1 = NULL;
static struct device * dev_porta = NULL;


static uint8_t transfer(uint8_t data_tx)
{
	uint8_t data_rx;
	struct spi_buf buf_tx[] = {{.buf = &data_tx,.len = sizeof(data_tx)}};
	struct spi_buf buf_rx[] = {{.buf = &data_rx,.len = sizeof(data_rx)}};
	struct spi_buf_set tx = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx = {.buffers = buf_rx, .count = 1};
	spi_transceive(dev_spi1, &spi_cfg, &tx, &rx);
	return data_rx;
}



static void init_adc()
{
	printk("Init SPI\n");
	//dev_spi1 = DEVICE_DT_GET(DT_ALIAS(spi_1)); // TODO: Why does'nt this work???
    dev_spi1 = device_get_binding(MY_DEVICE_SPI);
	dev_porta = device_get_binding(MY_DEVICE_CS_PORT); //--white---(PA10,D3,51)
	__ASSERT(dev_spi1, "device_get_binding failed");
	__ASSERT(dev_porta, "device_get_binding failed");
	//spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_GET(0);
	spi_cfg.frequency = 1*1000*1000;
	
	{
		int r = 0;
		r = gpio_pin_configure(dev_porta, MY_DEVICE_CS_PIN, GPIO_OUTPUT_ACTIVE);
		__ASSERT(r == 0, "gpio_pin_configure failed (err %u)", r);
	}

	
	
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	{
		uint32_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADDRESS, MCP356X_ADDR_CONFIG0, MCP356X_IWRITE);
		transfer(w);
		transfer(CONFIG0_CLK_SEL_INT|CONFIG0_ADC_MODE_CONV); // CONFGI0: Use Internal clock
		transfer(CONFIG1_OSR_256); //CONFGI1: Use Internal clock
		
		printk("0b11<<4:   %02x\n", CONFIG0_CLK_SEL_INT|CONFIG0_ADC_MODE_CONV);
		printk("CONFIG1_OSR_256:   %02x\n", CONFIG1_OSR_256);
		
	}
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	
	
	
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	{
		uint32_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADDRESS, MCP356X_ADDR_CONFIG3, MCP356X_IWRITE);
		transfer(w);
		transfer(CONFIG3_CONV_MODE_CONTINUOUS); // Use Internal clock
	}
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	
	
	
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	{
		uint32_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADDRESS, MCP356X_ADDR_IRQ, MCP356X_IWRITE);
		transfer(w);
		transfer(IRQ_MODE_HIGH); // Use Internal clock
	}
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	
	
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	{
		uint32_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADDRESS, MCP356X_ADDR_MUX, MCP356X_IWRITE);
		transfer(w);
		transfer(0b00001100); // plus:CH0   minus:REFIN-
	}
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	
	
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	{
		uint8_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADDRESS, MCP356X_ADDR_ADCDATA, MCP356X_IREAD);
		uint8_t r[4];
		r[0] = transfer(w);
		r[1] = transfer(0);
		r[2] = transfer(0);
		r[3] = transfer(0);
		printk("adcdata: %02x %02x %02x %02x\n", r[0], r[1], r[2], r[3]);
		
		uint8_t config0 = transfer(0);
		uint8_t config1 = transfer(0);
		uint8_t config2 = transfer(0);
		uint8_t config3 = transfer(0);
		uint8_t irq = transfer(0);
		uint8_t mux = transfer(0);
		uint8_t scan[4];
		scan[0] = transfer(0);
		scan[1] = transfer(0);
		scan[2] = transfer(0);
		scan[3] = transfer(0);
		uint8_t timer[4];
		timer[0] = transfer(0);
		timer[1] = transfer(0);
		timer[2] = transfer(0);
		timer[3] = transfer(0);
		uint8_t offsetcal[4];
		offsetcal[0] = transfer(0);
		offsetcal[1] = transfer(0);
		offsetcal[2] = transfer(0);
		offsetcal[3] = transfer(0);
		uint8_t gaincal[4];
		gaincal[0] = transfer(0);
		gaincal[1] = transfer(0);
		gaincal[2] = transfer(0);
		gaincal[3] = transfer(0);
		printk("config0:   %02x\n", config0);
		printk("config1:   %02x\n", config1);
		printk("config2:   %02x\n", config2);
		printk("config3:   %02x\n", config3);
		printk("irq:       %02x\n", irq);
		printk("mux:       %02x\n", mux);
		printk("scan:      %02x %02x %02x %02x\n", scan[0], scan[1], scan[2], scan[3]);
		printk("timer:     %02x %02x %02x %02x\n", timer[0], timer[1], timer[2], timer[3]);
		printk("offsetcal: %02x %02x %02x %02x\n", offsetcal[0], offsetcal[1], offsetcal[2], offsetcal[3]);
		printk("gaincal:   %02x %02x %02x %02x\n", gaincal[0], gaincal[1], gaincal[2], gaincal[3]);
	}
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
}







void test(void)
{
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	uint8_t r[5];
	uint8_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADDRESS, MCP356X_ADDR_ADCDATA, MCP356X_SREAD);
	r[0] = transfer(w);
	r[1] = transfer(0);
	r[2] = transfer(0);
	r[3] = transfer(0);
	r[4] = transfer(0);
	
	
	uint32_t tmp_data;
    tmp_data = r[2];
    tmp_data <<= 8;
    tmp_data |= r[3];
    tmp_data <<= 8;
    tmp_data |= r[4];
	uint8_t temp = r[1];
	
	uint8_t chan = ( temp >> 4 ) & 0x0F;
    uint8_t sign =  temp & 0x01;
    if( sign != 0 )
    {
        tmp_data -= 16777215;
    }

	//printk("r:      %02x %02x %02x %02x\n", r[0], r[1], r[2], r[3]);
	//uint32_t a = (r[3] << 16) | (r[2] << 8) | (r[1] << 0);
	//gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	printk("ADC: %02x: %u\n", chan, tmp_data);
}








void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);
	
	init_adc();
	
	
	while (1)
	{
		
		k_sleep(K_MSEC(1000));
		
		/*
		k_sleep(K_SECONDS(4));
		printk("MY_DEVICE_CS_PIN 0\n");
		gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
		k_sleep(K_SECONDS(4));
		printk("MY_DEVICE_CS_PIN 1\n");
		gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
		*/
		
		test();

		/* Temperature simulation */
		if (simulate_temp) {
			ess_simulate();
		}

		/* Battery level simulation */
		bas_notify();
	}
}
