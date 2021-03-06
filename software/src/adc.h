#pragma once
#include "MCP356X.h"



// The ADC9 uses 2000mV voltage ref chip MCP1501
#define VREF  2048


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

//struct spi_cs_control *ctrl = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(spi1), 2);
//struct spi_dt_spec spi_spec1 = SPI_DT_SPEC_GET(DT_NODELABEL(spi1), SPI_WORD_SET(8) | SPI_MODE_GET(0), 1);

struct spi_dt_spec spi_spec = 
{
	.bus = DEVICE_DT_GET(DT_NODELABEL(spi1)),
	.config = 
	{
		.operation = SPI_WORD_SET(8) | SPI_MODE_GET(0),
		.frequency = 1*1000*1000,
		.cs = &(struct spi_cs_control) 
		{
			.gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
			.delay = 1,
			.gpio_pin = 4,
			.gpio_dt_flags = GPIO_ACTIVE_LOW
		}
	}
};


//static struct device const * dev_spi1 = DEVICE_DT_GET(DT_NODELABEL(spi1));
//static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_1), gpios);
//static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios);
//static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_3), gpios);



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
	spi_transceive_dt(&spi_spec, &tx_buf, &rx_buf);
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
	spi_transceive_dt(&spi_spec, &tx_buf, &rx_buf);
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
	spi_transceive_dt(&spi_spec, &tx_buf, &rx_buf);
	uint32_t v = (rx[1] << 16) | (rx[2] << 8) | (rx[3] << 0);
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
	spi_transceive_dt(&spi_spec, &tx_buf, &rx_buf);
	uint32_t v = (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) | (rx[4] << 0);
	return v;
}

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

	spi_transceive_dt(&spi_spec, &tx_buf, &rx_buf);
	value->channel = rx[1] >> 4;
	uint8_t sign = rx[1] & 0x01;
	value->value = (rx[2] << 16) | (rx[3] << 8) | (rx[4] << 0);
	if (sign != 0)
	{
		value->value -= 16777215;
	}
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
	spi_transceive_dt(&spi_spec, &tx_buf, &rx_buf);
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

/*
5.3.1
ANALOG GAIN
The gain settings from 0.33x to 16x are done in the
analog domain. This analog gain is placed on each ADC
differential input. Each doubling of the gain improves the
thermal noise due to sampling by approximately 3 dB,
which means the lowest noise configuration is obtained
when using the highest analog gain. The SNR, however,
is degraded, since doubling the gain factor reduces the
maximum allowable input signal amplitude by
approximately 6 dB.
If the gain is set to 0.33x, the differential input range
theoretically becomes ??3 * VREF. However, the device
does not support input voltages outside of the power
supply voltage range. If large reference voltages are
used with this gain, the input voltage range will be
clipped between AGND and AVDD, and therefore, the
output code span will be limited. This gain is useful
when the reference voltage is small and when the
input signal voltage is large.
The analog gain stage can be used to amplify very low
signals, but the differential input range of the
Delta-Sigma modulator must not be exceeded.
*/
#define MY_GAIN MCP356X_CFG_2_GAIN_X_033

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
	//MCP356X_CFG_2_GAIN_X_1 |
	MY_GAIN |
	MCP356X_CFG_2_AZ_MUX_DIS |
	MCP356X_CFG_2_AZ_VREF_EN |
	MCP356X_CFG_2_AZ_FREQ_HIGH
	);
	set8_verbose(MCP356X_REG_CFG_3,
	MCP356X_CFG_3_CONV_MODE_CONT |
	//MCP356X_CFG_3_DATA_FORMAT_DEF |
	MCP356X_CFG_3_DATA_FORMAT_CH_ADC |
	MCP356X_CFG_3_CRC_FORMAT_16 |
	MCP356X_CFG_3_CRC_COM_DIS |
	MCP356X_CFG_3_CRC_OFF_CAL_EN |
	MCP356X_CFG_3_CRC_GAIN_CAL_EN
	);
	set8_verbose(MCP356X_REG_MUX,
	//MCP356X_MUX_VIN_POS_CH0 | 
	//MCP356X_MUX_VIN_NEG_CH1
	//MCP356X_MUX_VIN_POS_CH0 | 
	//MCP356X_MUX_VIN_NEG_AGND
	0
	);
	//set24_verbose(MCP356X_REG_SCAN, 0);
	//set24_verbose(MCP356X_REG_SCAN, MCP356X_SCAN_CH0|MCP356X_SCAN_CH1|MCP356X_SCAN_CH2);
	
	//set24_verbose(MCP356X_REG_IRQ, MCP356X_IRQ_MODE_LOGIC_HIGH);
	set24_verbose(MCP356X_REG_SCAN, MCP356X_SCAN_CH3);
	set24_verbose(MCP356X_REG_OFFSET_CAL, 0);
	set24_verbose(MCP356X_REG_GAIN_CAL, 0x00800000);
	//set24_verbose(MCP356X_RSV_REG_W_A, 0x00900F00);
	set24_verbose(MCP356X_RSV_REG_W_A, 0x00900000);
}





static void adc_print()
{
	/*
	printk("Channel:  ch0  ch1  ch2  ch3  ch4  ch5  ch6  ch7\n"), 
	printk("Voltage: %4i %4i %4i %4i %4i %4i %4i %4i\n", 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH0), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH1), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH2), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH3), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH4), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH5), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH6), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH7)
	);
	*/
	// printk("Voltage0: %4i\n", (int)voltage_ch(MCP356X_MUX_VIN_POS_CH0));
	// printk("Voltage7: %4i\n", (int)voltage_ch(MCP356X_MUX_VIN_POS_CH7));
	//printk("ADCDATA: %08x\n", get32(MCP356X_REG_ADC_DATA));
	struct mcp356x_data11 data;
	mcp356x_data11_get(&data);
	printk("Voltage: %02x %08x %08i\n", data.channel, data.value, MCP356X_raw_to_mv(data.value, VREF, MY_GAIN));
}




