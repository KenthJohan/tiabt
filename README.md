# tiabt
Low cost 8 channels transimpedance amplifier with bluetooth.


## circuit1
The `/circuit1` is low cost.
* op-amp `MCP6004` 0.5 EUR<br>

|Name                  |Category|  EUR|Footprint |   |   |   |   |   |   |
|----------------------|--------|-----|----------|---|---|---|---|---|---|
| [MCP6004][3]         | Op-Amp |0.5  |SOIC-14   |   |   |   |   |   |   |
| [MCP3564][4]         | ADC    |6    |          |   |   |   |   |   |   |
| [nRF52840 Dongle][1] | MCU    |     |          |   |   |   |   |   |   |
|                      |        |     |          |   |   |   |   |   |   |
|                      |        |     |          |   |   |   |   |   |   |
|                      |        |     |          |   |   |   |   |   |   |
|                      |        |     |          |   |   |   |   |   |   |
|                      |        |     |          |   |   |   |   |   |   |
|                      |        |     |          |   |   |   |   |   |   |
|                      |        |     |          |   |   |   |   |   |   |


## circuit2
The `/circuit2` is high cost.

|Name                  |Category|  EUR|Footprint|   |   |   |   |   |   |
|----------------------|--------|-----|---------|---|---|---|---|---|---|
| [MCP6034][9]         | Op-Amp |0.5  |SOIC-14  |   |   |   |   |   |   |
| [MCP3564][4]         | ADC    |6    |         |   |   |   |   |   |   |
| [nRF52840 Dongle][1] | MCU    |     |         |   |   |   |   |   |   |
| [MAX6070][5]         | VREF   |     |SOT-23-6 |   |   |   |   |   |   |
| [LP2985-3.6][6]      | VREG   |     |SOT-23-5 |   |   |   |   |   |   |
| [LP5907MFX-4.5][7]   | VREG   |     |SOT-23-5 |   |   |   |   |   |   |
| [LM1117-5.0][8]      | VREG   |     |SOT-223-3|   |   |   |   |   |   |
|                      |        |     |         |   |   |   |   |   |   |
|                      |        |     |         |   |   |   |   |   |   |
|                      |        |     |         |   |   |   |   |   |   |
|                      |        |     |         |   |   |   |   |   |   |



[1]:https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dongle
[2]:https://docs.zephyrproject.org/2.6.0/boards/arm/nrf52840dongle_nrf52840/doc/index.html
[3]:https://www.microchip.com/en-us/product/MCP6004
[4]:https://www.microchip.com/en-us/product/MCP3564
[5]:https://www.maximintegrated.com/en/products/analog/voltage-references/MAX6070.html
[6]:https://www.ti.com/product/LP2985
[7]:https://www.ti.com/product/LP5907
[8]:https://www.onsemi.com/products/power-management/linear-regulators-ldo/lm1117
[9]:https://www.microchip.com/en-us/product/MCP3564