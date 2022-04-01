#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <sys/__assert.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/pinctrl.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "adc.h"
#include "bt.h"

// https://os.mbed.com/platforms/ST-Nucleo-WB55RG/
// https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/arm/nucleo_wb55rg/nucleo_wb55rg.dts
/*
&spi1 {
	pinctrl-0 = <&spi1_nss_pa4 &spi1_sck_pa5
		     &spi1_miso_pa6 &spi1_mosi_pa7>;
	pinctrl-names = "default";
	status = "okay";
};
*/
// ADC pin CSK ---orange--- (PA5,D13,20,SCK)
// ADC pin SDO ---yellow--- (PA6,D12,21,MISO)
// ADC pin SDI ---green---- (PA7,D11,22,MOSI)
// ADC pin CS  ---white---- (PA4,D10)
// https://github.com/zephyrproject-rtos/zephyr-testing/blob/0f09360456592ecf0d0413352dc1ef5d23cd0364/include/drivers/spi.h#L252
// https://github.com/zephyrproject-rtos/zephyr-testing/blob/2fc0ace3c2462cb33dc8c25e33c629cedba08ebb/boards/arm/rm1xx_dvk/rm1xx_dvk.dts#L60
// https://github.com/zephyrproject-rtos/zephyr/blob/ee91cd5665b1590c9a8a9a992bb19d81feeeb461/boards/arm/bt610/bt610.dts#L268
// struct spi_cs_control *ctrl = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(spi1), 2);
static struct spi_config spi_cfg = 
{
	.operation = SPI_WORD_SET(8) | SPI_MODE_GET(0),
	.frequency = 1*1000*1000,
	//.cs = SPI_CS_CONTROL_PTR_DT(spi1, 1)
	.cs = &(struct spi_cs_control) 
	{
		//.gpio_dev = DEVICE_DT_GET(DT_NODELABEL(spi1_nss_pa4))
		//.gpio = SPI_CS_GPIOS_DT_SPEC_GET(spi1)
		.gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
		.delay = 1,
		.gpio_pin = 4,
		.gpio_dt_flags = GPIO_ACTIVE_LOW
	}
};


static struct device const * dev_spi1 = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_3), gpios);



static void set8(uint8_t reg, uint8_t value)
{
	uint8_t tx[2];
	uint8_t rx[2];
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_WRITE);
	tx[1] = value;
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
}


static uint8_t get8(uint8_t reg)
{
	uint8_t tx[2];
	uint8_t rx[2];
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	return rx[1];
}

static uint32_t get24(uint8_t reg)
{
	uint8_t tx[4];
	uint8_t rx[4];
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[2] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[3] = 0; // Need to write in order to read. Exchange 0 for reading data.
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	uint32_t v = (rx[1] << 16) | (rx[2] << 8) |  (rx[3] << 0);
	return v;
}

static uint32_t get32(uint8_t reg)
{
	uint8_t tx[5];
	uint8_t rx[5];
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[2] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[3] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[4] = 0; // Need to write in order to read. Exchange 0 for reading data.
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	uint32_t v = (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) |  (rx[4] << 0);
	return v;
}




static void set24(uint8_t reg, uint32_t value)
{
	uint8_t tx[4];
	uint8_t rx[4];
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_WRITE);
	tx[1] = ( value >> 16 ) & 0xFF;
	tx[2] = ( value >> 8 ) & 0xFF;
	tx[3] = ( value >> 0 ) & 0xFF;
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
}



static void set8_verbose(uint8_t reg, uint8_t value)
{
	set8(reg, value);
	uint8_t v = get8(reg);
	printk("SET8: %s: %02x %02x\n", MCP356X_REG_tostring(reg), value, v);
}

static void set24_verbose(uint8_t reg, uint32_t value)
{
	set24(reg, value);
	uint32_t v = get24(reg);
	printk("SET24: %s: %08x %08x\n", MCP356X_REG_tostring(reg), value, v);
}


static void init_adc()
{
	printk("Init ADC MCP356X\n");

	set8_verbose(MCP356X_REG_CFG_0,
	MCP356X_CFG_0_VREF_SEL_0 |
	MCP356X_CFG_0_CLK_SEL_2 |
	MCP356X_CFG_0_CS_SEL_0 |
	MCP356X_CFG_0_MODE_CONV
	);
	set8_verbose(MCP356X_REG_CFG_1,
	MCP356X_CFG_1_PRE_1 |
	MCP356X_CFG_1_OSR_32 |
	MCP356X_CFG_1_DITHER_DEF
	);
	set8_verbose(MCP356X_REG_CFG_2,
	MCP356X_CFG_2_BOOST_X_1 |
	MCP356X_CFG_2_GAIN_X_1 |
	MCP356X_CFG_2_AZ_MUX_DIS |
	MCP356X_CFG_2_AZ_VREF_EN |
	MCP356X_CFG_2_AZ_FREQ_HIGH
	);
	set8_verbose(MCP356X_REG_CFG_3,
	MCP356X_CFG_3_CONV_MODE_CONT |
	MCP356X_CFG_3_DATA_FORMAT_DEF |
	MCP356X_CFG_3_CRC_FORMAT_16 |
	MCP356X_CFG_3_CRC_COM_DIS |
	MCP356X_CFG_3_CRC_OFF_CAL_EN |
	MCP356X_CFG_3_CRC_GAIN_CAL_EN
	);
	set8_verbose(MCP356X_REG_MUX,
	MCP356X_MUX_VIN_POS_CH0 | 
	MCP356X_MUX_VIN_NEG_CH1
	);
	set24_verbose(MCP356X_REG_SCAN, 0);
	set24_verbose(MCP356X_REG_OFFSET_CAL, 0);
	set24_verbose(MCP356X_REG_GAIN_CAL, 0x00800000);
	set24_verbose(MCP356X_RSV_REG_W_A, 0x00900F00);
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

	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
	/*
	gpio_pin_toggle_dt(&led1);
	gpio_pin_toggle_dt(&led2);
	gpio_pin_toggle_dt(&led3);
	*/
	
	while (1)
	{
		k_sleep(K_MSEC(1000));
		int32_t v = get32(MCP356X_REG_ADC_DATA);
		v = adc9_volt_calc(v);
		printk("Voltage: %i mV\n", v);


		if (simulate_temp)
		{
			ess_simulate();
		}
		bas_notify();
	}
}
