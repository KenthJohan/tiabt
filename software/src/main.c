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


#include "adc.h"
#include "bt.h"

// https://os.mbed.com/platforms/ST-Nucleo-WB55RG/

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
#define MY_DEVICE_SPI "SPI_1"

// ADC pin CS ---blue--- (PA10,D3,51)
#define MY_DEVICE_CS_PIN 4
#define MY_DEVICE_CS_PORT "GPIOA"





static struct spi_config spi_cfg = {0};
static struct spi_cs_control cs_ctrl;
static struct device const * dev_spi1 = NULL;
static struct device const * dev_porta = NULL;


static uint8_t transfer(uint8_t data_tx)
{
	uint8_t data_rx;
	struct spi_buf buf_tx[] = {{.buf = &data_tx,.len = sizeof(data_tx)}};
	struct spi_buf buf_rx[] = {{.buf = &data_rx,.len = sizeof(data_rx)}};
	struct spi_buf_set tx = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx = {.buffers = buf_rx, .count = 1};
	spi_transceive(dev_spi1, &spi_cfg, &tx, &rx);
	k_sleep(K_MSEC(1));
	return data_rx;
}

static uint8_t transfer1(char const * s, uint8_t tx)
{
	uint8_t rx = transfer(tx);
	printk("transfer %10s: %02x : %02x\n", s, tx, rx);
	return rx;
}




static void set8(uint8_t reg, uint8_t value)
{
	uint8_t tx[4];
	uint8_t rx[4];
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_WRITE);
	tx[1] = value;
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
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
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	return rx[1];
}

static uint32_t get24(uint8_t reg)
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
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	uint32_t v = (rx[1] << 16) | (rx[2] << 8) |  (rx[3] << 0);
	return v;
}

static int32_t get32(uint8_t reg)
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
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	int32_t v = (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) |  (rx[4] << 0);
	v = v / 256;
	printk("GET32: %s: %i\n", MCP356X_REG_tostring(reg), v);
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
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	spi_transceive(dev_spi1, &spi_cfg, &tx_buf, &rx_buf);
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
}



static void set24_debug(uint8_t reg, uint32_t value)
{
	set24(reg, value);
	uint32_t v = get24(reg);
	printk("SET24: %s: %08x %08x\n", MCP356X_REG_tostring(reg), value, v);
}

static void set8_debug(uint8_t reg, uint8_t value)
{
	set8(reg, value);
	uint8_t v = get8(reg);
	printk("SET8: %s: %08x %08x\n", MCP356X_REG_tostring(reg), value, v);
}






#define ADC9_CALC_COEF                    8388608
#define VREF  2048
float adc9_volt_calc ( int32_t adc_val)
{
	float volt;
	float gain = 1.0f;
	uint32_t coef = ADC9_CALC_COEF;
	volt = ( float )( adc_val / ( float )( coef * gain ) ) * ( float ) VREF;
	return volt;
}

#define PWM_NODE DT_NODELABEL(blue_led_1)

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_3), gpios);

static void init_adc()
{
	printk("Init SPI\n");
	dev_spi1 = DEVICE_DT_GET(DT_NODELABEL(spi1));
	dev_porta = device_get_binding(MY_DEVICE_CS_PORT); //--white---(PA10,D3,51)
	__ASSERT(dev_spi1, "device_get_binding failed");
	__ASSERT(dev_porta, "device_get_binding failed");
	//spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_GET(0);
	spi_cfg.frequency = 1*1000*1000;
	
	cs_ctrl.gpio_dev = device_get_binding(MY_DEVICE_CS_PORT);
	cs_ctrl.gpio_pin = MY_DEVICE_CS_PIN;

	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);

	/*
	gpio_pin_toggle_dt(&led1);
	gpio_pin_toggle_dt(&led2);
	gpio_pin_toggle_dt(&led3);
	*/

	{
		int r = 0;
		r = gpio_pin_configure(dev_porta, MY_DEVICE_CS_PIN, GPIO_OUTPUT_ACTIVE);
		__ASSERT(r == 0, "gpio_pin_configure failed (err %u)", r);
	}



	set8_debug(MCP356X_REG_CFG_0,
	MCP356X_CFG_0_VREF_SEL_0 |
	MCP356X_CFG_0_CLK_SEL_2 |
	MCP356X_CFG_0_CS_SEL_0 |
	MCP356X_CFG_0_MODE_CONV
	);

	set8_debug(MCP356X_REG_CFG_1,
	MCP356X_CFG_1_PRE_1 |
	MCP356X_CFG_1_OSR_32 |
	MCP356X_CFG_1_DITHER_DEF
	);

	set8_debug(MCP356X_REG_CFG_2,
	MCP356X_CFG_2_BOOST_X_1 |
	MCP356X_CFG_2_GAIN_X_1 |
	MCP356X_CFG_2_AZ_MUX_DIS |
	MCP356X_CFG_2_AZ_VREF_EN |
	MCP356X_CFG_2_AZ_FREQ_HIGH
	);

	set8_debug(MCP356X_REG_CFG_3,
	MCP356X_CFG_3_CONV_MODE_CONT |
	MCP356X_CFG_3_DATA_FORMAT_DEF |
	MCP356X_CFG_3_CRC_FORMAT_16 |
	MCP356X_CFG_3_CRC_COM_DIS |
	MCP356X_CFG_3_CRC_OFF_CAL_EN |
	MCP356X_CFG_3_CRC_GAIN_CAL_EN
	);

	set8_debug(MCP356X_REG_MUX,
	MCP356X_MUX_VIN_POS_CH0 | 
	MCP356X_MUX_VIN_NEG_CH1
	);


	set24_debug(MCP356X_REG_SCAN, 0);
	set24_debug(MCP356X_REG_OFFSET_CAL, 0);
	set24_debug(MCP356X_REG_GAIN_CAL, 0x00800000);
	set24_debug(MCP356X_RSV_REG_W_A, 0x00900F00);


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
		int32_t v = get32(MCP356X_REG_ADC_DATA);
		v = adc9_volt_calc(v);
		printk("Voltage: %i\n", v);


		if (simulate_temp)
		{
			ess_simulate();
		}
		bas_notify();
	}
}
