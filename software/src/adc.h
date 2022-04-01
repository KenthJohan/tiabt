#pragma once

// https://github.com/MikroElektronika/mikrosdk_click_v2
// https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/adc9

#define MCP356X_COMMAND_BYTE(addr, cmd, type) (((addr)<<6) | ((cmd)<<2) | ((type)<<0))

#define MCP356X_DEVICE_ADR                   0x01



#define MCP356X_CMD_DONT_CARE                0x00
#define MCP356X_CMD_STAT_READ                0x01
#define MCP356X_CMD_INC_WRITE                0x02
#define MCP356X_CMD_INC_READ                 0x03


#define MCP356X_REG_ADC_DATA                 0x00
#define MCP356X_REG_CFG_0                    0x01
#define MCP356X_REG_CFG_1                    0x02
#define MCP356X_REG_CFG_2                    0x03
#define MCP356X_REG_CFG_3                    0x04
#define MCP356X_REG_IRQ                      0x05
#define MCP356X_REG_MUX                      0x06
#define MCP356X_REG_SCAN                     0x07
#define MCP356X_REG_TIMER                    0x08
#define MCP356X_REG_OFFSET_CAL               0x09
#define MCP356X_REG_GAIN_CAL                 0x0A
#define MCP356X_RSV_REG_W_A                  0x0B
#define MCP356X_REG_LOCK                     0x0D
#define MCP356X_RSV_REG                      0x0E
#define MCP356X_REG_CRC_CFG                  0x0F

static char const * MCP356X_REG_tostring(int a)
{
	switch(a)
	{
	case MCP356X_REG_ADC_DATA    : return "MCP356X_REG_ADC_DATA    ";
	case MCP356X_REG_CFG_0       : return "MCP356X_REG_CFG_0       ";
	case MCP356X_REG_CFG_1       : return "MCP356X_REG_CFG_1       ";
	case MCP356X_REG_CFG_2       : return "MCP356X_REG_CFG_2       ";
	case MCP356X_REG_CFG_3       : return "MCP356X_REG_CFG_3       ";
	case MCP356X_REG_IRQ         : return "MCP356X_REG_IRQ         ";
	case MCP356X_REG_MUX         : return "MCP356X_REG_MUX         ";
	case MCP356X_REG_SCAN        : return "MCP356X_REG_SCAN        ";
	case MCP356X_REG_TIMER       : return "MCP356X_REG_TIMER       ";
	case MCP356X_REG_OFFSET_CAL  : return "MCP356X_REG_OFFSET_CAL  ";
	case MCP356X_REG_GAIN_CAL    : return "MCP356X_REG_GAIN_CAL    ";
	case MCP356X_RSV_REG_W_A     : return "MCP356X_RSV_REG_W_A     ";
	case MCP356X_REG_LOCK        : return "MCP356X_REG_LOCK        ";
	case MCP356X_RSV_REG         : return "MCP356X_RSV_REG         ";
	case MCP356X_REG_CRC_CFG     : return "MCP356X_REG_CRC_CFG     ";
	default:return "";
	}
}


#define MCP356X_CFG_0_VREF_SEL_0             0x00
#define MCP356X_CFG_0_VREF_SEL_1             0x40
#define MCP356X_CFG_0_VREF_SEL_2             0x80
#define MCP356X_CFG_0_VREF_SEL_3             0xC0

#define MCP356X_CFG_0_CLK_SEL_0              0x00
#define MCP356X_CFG_0_CLK_SEL_1              0x10
#define MCP356X_CFG_0_CLK_SEL_2              0x20
#define MCP356X_CFG_0_CLK_SEL_3              0x30

#define MCP356X_CFG_0_CS_SEL_0               0x00
#define MCP356X_CFG_0_CS_SEL_1               0x04
#define MCP356X_CFG_0_CS_SEL_2               0x08
#define MCP356X_CFG_0_CS_SEL_3               0x0C

#define MCP356X_CFG_0_MODE_SHD_DEF           0x00
#define MCP356X_CFG_0_MODE_SHD               0x01
#define MCP356X_CFG_0_MODE_STANDBY           0x02
#define MCP356X_CFG_0_MODE_CONV              0x03

#define MCP356X_CFG_1_PRE_1                  0x00
#define MCP356X_CFG_1_PRE_2                  0x40
#define MCP356X_CFG_1_PRE_4                  0x80
#define MCP356X_CFG_1_PRE_8                  0xC0

#define MCP356X_CFG_1_OSR_98304              0x3C
#define MCP356X_CFG_1_OSR_81920              0x38
#define MCP356X_CFG_1_OSR_49152              0x34
#define MCP356X_CFG_1_OSR_40960              0x30
#define MCP356X_CFG_1_OSR_24576              0x2C
#define MCP356X_CFG_1_OSR_20480              0x28
#define MCP356X_CFG_1_OSR_16384              0x24
#define MCP356X_CFG_1_OSR_8192               0x20
#define MCP356X_CFG_1_OSR_4096               0x1C
#define MCP356X_CFG_1_OSR_2048               0x18
#define MCP356X_CFG_1_OSR_1024               0x14
#define MCP356X_CFG_1_OSR_512                0x10
#define MCP356X_CFG_1_OSR_256                0x0C
#define MCP356X_CFG_1_OSR_128                0x08
#define MCP356X_CFG_1_OSR_64                 0x04
#define MCP356X_CFG_1_OSR_32                 0x00

#define MCP356X_CFG_1_DITHER_MAX             0x03
#define MCP356X_CFG_1_DITHER_MED             0x02
#define MCP356X_CFG_1_DITHER_MIN             0x01
#define MCP356X_CFG_1_DITHER_DEF             0x00

#define MCP356X_CFG_2_BOOST_X_2              0xC0
#define MCP356X_CFG_2_BOOST_X_1              0x80
#define MCP356X_CFG_2_BOOST_X_066            0x40
#define MCP356X_CFG_2_BOOST_X_05             0x00

#define MCP356X_CFG_2_GAIN_X_64              0x38
#define MCP356X_CFG_2_GAIN_X_32              0x30
#define MCP356X_CFG_2_GAIN_X_16              0x28
#define MCP356X_CFG_2_GAIN_X_8               0x20
#define MCP356X_CFG_2_GAIN_X_4               0x18
#define MCP356X_CFG_2_GAIN_X_2               0x10
#define MCP356X_CFG_2_GAIN_X_1               0x08
#define MCP356X_CFG_2_GAIN_X_033             0x00

#define MCP356X_CFG_2_AZ_MUX_EN              0x04
#define MCP356X_CFG_2_AZ_MUX_DIS             0x00
#define MCP356X_CFG_2_AZ_VREF_EN             0x02
#define MCP356X_CFG_2_AZ_VREF_DIS            0x00

#define MCP356X_CFG_2_AZ_FREQ_HIGH           0x01
#define MCP356X_CFG_2_AZ_FREQ_LOW            0x00

#define MCP356X_CFG_3_CONV_MODE_CONT         0xC0
#define MCP356X_CFG_3_CONV_MODE_STANDBY      0x80
#define MCP356X_CFG_3_CONV_MODE_SHD          0x40
#define MCP356X_CFG_3_CONV_MODE_SHD0         0x00

#define MCP356X_CFG_3_DATA_FORMAT_CH_ADC     0x30
#define MCP356X_CFG_3_DATA_FORMAT_SIGN_ADC   0x20
#define MCP356X_CFG_3_DATA_FORMAT_LEFT_JUST  0x10
#define MCP356X_CFG_3_DATA_FORMAT_DEF        0x00

#define MCP356X_CFG_3_CRC_FORMAT_32          0x08
#define MCP356X_CFG_3_CRC_FORMAT_16          0x00

#define MCP356X_CFG_3_CRC_COM_EN             0x04
#define MCP356X_CFG_3_CRC_COM_DIS            0x00

#define MCP356X_CFG_3_CRC_OFF_CAL_EN         0x02
#define MCP356X_CFG_3_CRC_OFF_CAL_DIS        0x00
#define MCP356X_CFG_3_CRC_GAIN_CAL_EN        0x01
#define MCP356X_CFG_3_CRC_GAIN_CAL_DIS       0x00

#define MCP356X_IRQ_MODE_MDAT                0x08
#define MCP356X_IRQ_MODE_IRQ                 0x00
#define MCP356X_IRQ_MODE_LOGIC_HIGH          0x04
#define MCP356X_IRQ_MODE_HIGH_Z              0x00
#define MCP356X_IRQ_FASTCMD_EN               0x02
#define MCP356X_IRQ_FASTCMD_DIS              0x00
#define MCP356X_IRQ_STP_EN                   0x01
#define MCP356X_IRQ_STP_DIS                  0x00

#define MCP356X_MUX_VIN_POS_NO_IN            0xF0
#define MCP356X_MUX_VIN_POS_VCM              0xE0
#define MCP356X_MUX_VIN_POS_TEMP             0xD0
#define MCP356X_MUX_VIN_POS_VREF_EXT_MINUS   0xC0
#define MCP356X_MUX_VIN_POS_VREF_EXT_PLUS    0xB0
#define MCP356X_MUX_VIN_POS_VREF_INT         0xA0
#define MCP356X_MUX_VIN_POS_AVDD             0x90
#define MCP356X_MUX_VIN_POS_VSS              0x80
#define MCP356X_MUX_VIN_POS_CH7              0x70
#define MCP356X_MUX_VIN_POS_CH6              0x60
#define MCP356X_MUX_VIN_POS_CH5              0x50
#define MCP356X_MUX_VIN_POS_CH4              0x40
#define MCP356X_MUX_VIN_POS_CH3              0x30
#define MCP356X_MUX_VIN_POS_CH2              0x20
#define MCP356X_MUX_VIN_POS_CH1              0x10
#define MCP356X_MUX_VIN_POS_CH0              0x00

#define MCP356X_MUX_VIN_NEG_NO_IN            0x0F
#define MCP356X_MUX_VIN_NEG_VCM              0x0E
#define MCP356X_MUX_VIN_NEG_TEMP             0x0D
#define MCP356X_MUX_VIN_NEG_VREF_EXT_MINUS   0x0C
#define MCP356X_MUX_VIN_NEG_VREF_EXT_PLUS    0x0B
#define MCP356X_MUX_VIN_NEG_VREF_INT         0x0A
#define MCP356X_MUX_VIN_NEG_AVDD             0x09
#define MCP356X_MUX_VIN_NEG_VSS              0x08
#define MCP356X_MUX_VIN_NEG_CH7              0x07
#define MCP356X_MUX_VIN_NEG_CH6              0x06
#define MCP356X_MUX_VIN_NEG_CH5              0x05
#define MCP356X_MUX_VIN_NEG_CH4              0x04
#define MCP356X_MUX_VIN_NEG_CH3              0x03
#define MCP356X_MUX_VIN_NEG_CH2              0x02
#define MCP356X_MUX_VIN_NEG_CH1              0x01
#define MCP356X_MUX_VIN_NEG_CH0              0x00

#define MCP356X_SCAN_DLY_DM_CLK_X_512        0x00E00000
#define MCP356X_SCAN_DLY_DM_CLK_X_256        0x00C00000
#define MCP356X_SCAN_DLY_DM_CLK_X_128        0x00A00000
#define MCP356X_SCAN_DLY_DM_CLK_X_64         0x00800000
#define MCP356X_SCAN_DLY_DM_CLK_X_32         0x00600000
#define MCP356X_SCAN_DLY_DM_CLK_X_16         0x00400000
#define MCP356X_SCAN_DLY_DM_CLK_X_8          0x00200000
#define MCP356X_SCAN_DLY_NO_DELAY            0x00000000

#define MCP356X_SCAN_PSAV_VREF_OFF           0x00100000
#define MCP356X_SCAN_PSAV_VREF_ON            0x00000000

#define MCP356X_SCAN_OFFSET                  0x00008000
#define MCP356X_SCAN_VREF                    0x00004000
#define MCP356X_SCAN_AVDD                    0x00002000
#define MCP356X_SCAN_TEMP                    0x00001000
#define MCP356X_SCAN_DIFF_D                  0x00000800
#define MCP356X_SCAN_DIFF_C                  0x00000400
#define MCP356X_SCAN_DIFF_B                  0x00000200
#define MCP356X_SCAN_DIFF_A                  0x00000100
#define MCP356X_SCAN_CH7                     0x00000080
#define MCP356X_SCAN_CH6                     0x00000040
#define MCP356X_SCAN_CH5                     0x00000020
#define MCP356X_SCAN_CH4                     0x00000010
#define MCP356X_SCAN_CH3                     0x00000008
#define MCP356X_SCAN_CH2                     0x00000004
#define MCP356X_SCAN_CH1                     0x00000002
#define MCP356X_SCAN_CH0                     0x00000001

#define MCP356X_TIMER_DLY_DMCLK_X_16777215   0x00FFFFFF
#define MCP356X_TIMER_DLY_DMCLK_X_8388607    0x007FFFFF
#define MCP356X_TIMER_DLY_DMCLK_X_4194303    0x003FFFFF
#define MCP356X_TIMER_DLY_DMCLK_X_2097151    0x001FFFFF
#define MCP356X_TIMER_DLY_DMCLK_X_1048575    0x000FFFFF
#define MCP356X_TIMER_DLY_DMCLK_X_524287     0x0007FFFF
#define MCP356X_TIMER_DLY_DMCLK_X_262143     0x0003FFFF
#define MCP356X_TIMER_DLY_DMCLK_X_131071     0x0001FFFF
#define MCP356X_TIMER_DLY_DMCLK_X_65535      0x0000FFFF
#define MCP356X_TIMER_DLY_DMCLK_X_32767      0x00007FFF
#define MCP356X_TIMER_DLY_DMCLK_X_16383      0x00003FFF
#define MCP356X_TIMER_DLY_DMCLK_X_8191       0x00001FFF
#define MCP356X_TIMER_DLY_DMCLK_X_4095       0x00000FFF
#define MCP356X_TIMER_DLY_DMCLK_X_2047       0x000007FF
#define MCP356X_TIMER_DLY_DMCLK_X_1023       0x000003FF
#define MCP356X_TIMER_DLY_DMCLK_X_511        0x000001FF
#define MCP356X_TIMER_DLY_DMCLK_X_255        0x000000FF
#define MCP356X_TIMER_DLY_DMCLK_X_127        0x0000007F
#define MCP356X_TIMER_DLY_DMCLK_X_63         0x0000003F
#define MCP356X_TIMER_DLY_DMCLK_X_31         0x0000001F
#define MCP356X_TIMER_DLY_DMCLK_X_15         0x0000000F
#define MCP356X_TIMER_DLY_DMCLK_X_2          0x00000002
#define MCP356X_TIMER_DLY_DMCLK_X_1          0x00000001
#define MCP356X_TIMER_DLY_NO_DELAY           0x00000000

#define MCP356X_FAST_CMD_ADC_CONV_START      0x28
#define MCP356X_FAST_CMD_ADC_STANDBY_MODE    0x2C
#define MCP356X_FAST_CMD_ADC_SHUTDOWN_MODE   0x30
#define MCP356X_FAST_CMD_FULL_SHUTDOWN_MODE  0x34
#define MCP356X_FAST_CMD_DEV_FULL_RESET      0x38

#define MCP356X_CALC_COEF                    8388608
