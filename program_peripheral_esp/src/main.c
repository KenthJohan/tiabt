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



static void set(uint8_t reg, uint8_t val)
{
	uint8_t cmd;
	uint8_t ack;
	uint8_t rx;

	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	cmd = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_WRITE);
	transfer(cmd);
	k_sleep(K_MSEC(1));
	transfer(val);
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	k_sleep(K_MSEC(1));



	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	cmd = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_READ);
	ack = transfer(cmd);
	k_sleep(K_MSEC(1));
	rx = transfer(0);
	printk("ADCSET: %s: %02x %02x ack:%02x\n", MCP356X_REG_tostring(reg), val, rx, ack);
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	k_sleep(K_MSEC(1));

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



set(MCP356X_REG_CFG_0,
MCP356X_CFG_0_VREF_SEL_0 |
MCP356X_CFG_0_CLK_SEL_2 |
MCP356X_CFG_0_CS_SEL_0 |
MCP356X_CFG_0_MODE_CONV
);

set(MCP356X_REG_CFG_1,
MCP356X_CFG_1_PRE_1 |
MCP356X_CFG_1_OSR_32 |
MCP356X_CFG_1_DITHER_DEF
);

set(MCP356X_REG_CFG_2,
MCP356X_CFG_2_BOOST_X_1 |
MCP356X_CFG_2_GAIN_X_1 |
MCP356X_CFG_2_AZ_MUX_DIS |
MCP356X_CFG_2_AZ_VREF_EN |
MCP356X_CFG_2_AZ_FREQ_HIGH
);

set(MCP356X_REG_CFG_3,
MCP356X_CFG_3_CONV_MODE_CONT |
MCP356X_CFG_3_DATA_FORMAT_DEF |
MCP356X_CFG_3_CRC_FORMAT_16 |
MCP356X_CFG_3_CRC_COM_DIS |
MCP356X_CFG_3_CRC_OFF_CAL_EN |
MCP356X_CFG_3_CRC_GAIN_CAL_EN
);

#if 0
gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
{
	uint8_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, MCP356X_REG_ADC_DATA, MCP356X_CMD_INC_READ);
	uint8_t r[5];
	r[0] = transfer(w);
	r[1] = transfer(0); // Read ADC Byte Value 0
	r[2] = transfer(0); // Read ADC Byte Value 1
	r[3] = transfer(0); // Read ADC Byte Value 2
	r[4] = transfer(0); // Read ADC Byte Value 3
	printk("adcdata: %02x %02x %02x %02x\n", r[0], r[1], r[2], r[3]);
}
gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	
gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
{
	uint32_t cmd = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, MCP356X_REG_CFG_0, MCP356X_CMD_INC_WRITE);
	transfer1("INIT CFG_0", cmd);
	transfer1
	("CFG_0",
	MCP356X_CFG_0_VREF_SEL_0 |
	MCP356X_CFG_0_CLK_SEL_2 |
	MCP356X_CFG_0_CS_SEL_0 |
	MCP356X_CFG_0_MODE_CONV
	);
	transfer1
	("CFG_1",
	MCP356X_CFG_1_PRE_1 |
	MCP356X_CFG_1_OSR_32 |
	MCP356X_CFG_1_DITHER_DEF
	);
	transfer1
	("CFG_2",
	MCP356X_CFG_2_BOOST_X_1 |
	MCP356X_CFG_2_GAIN_X_1 |
	MCP356X_CFG_2_AZ_MUX_DIS |
	MCP356X_CFG_2_AZ_VREF_EN |
	MCP356X_CFG_2_AZ_FREQ_HIGH
	);
	transfer1
	("CFG_3",
	MCP356X_CFG_3_CONV_MODE_CONT |
	MCP356X_CFG_3_DATA_FORMAT_DEF |
	MCP356X_CFG_3_CRC_FORMAT_16 |
	MCP356X_CFG_3_CRC_COM_DIS |
	MCP356X_CFG_3_CRC_OFF_CAL_EN |
	MCP356X_CFG_3_CRC_GAIN_CAL_EN
	);
}
gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
#endif

	
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	{
		/*
		uint8_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, MCP356X_REG_ADC_DATA, MCP356X_CMD_INC_READ);
		uint8_t r[5];
		r[0] = transfer(w);
		r[1] = transfer(0); // Read ADC Byte Value 0
		r[2] = transfer(0); // Read ADC Byte Value 1
		r[3] = transfer(0); // Read ADC Byte Value 2
		r[4] = transfer(0); // Read ADC Byte Value 3
		printk("adcdata: %02x %02x %02x %02x\n", r[0], r[1], r[2], r[3]);
		*/
		uint8_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, MCP356X_REG_CFG_0, MCP356X_CMD_INC_READ);
		transfer1("Read", w);
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
	uint8_t r[5];
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 0);
	{
		uint8_t w = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, MCP356X_REG_ADC_DATA, MCP356X_CMD_INC_READ);
		r[0] = transfer(w);
		r[1] = transfer(0); // Read ADC Byte Value 0
		r[2] = transfer(0); // Read ADC Byte Value 1
		r[3] = transfer(0); // Read ADC Byte Value 2
		r[4] = transfer(0); // Read ADC Byte Value 3
	}
	gpio_pin_set(dev_porta, MY_DEVICE_CS_PIN, 1);
	printk("ADC: %02x: %02x %02x %02x %02x\n", r[0], r[1], r[2], r[3], r[4]);
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
		
		//test();

		/* Temperature simulation */
		if (simulate_temp) {
			ess_simulate();
		}

		/* Battery level simulation */
		bas_notify();
	}
}
