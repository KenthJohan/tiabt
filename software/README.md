## Build nrf52840dongle
https://docs.zephyrproject.org/2.6.0/boards/arm/nrf52840dongle_nrf52840/doc/index.html
```bash
west build -b nrf52840dongle_nrf52840 --pristine
nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
        --application build/zephyr/zephyr.hex \
        --application-version 1 blinky.zip
nrfutil dfu usb-serial -pkg blinky.zip -p COM5
```

## Build nucleo_wb55rg
https://docs.zephyrproject.org/2.6.0/boards/arm/nucleo_wb55rg/doc/nucleo_wb55rg.html?highlight=nucleo_wb55rg
```bash
west build -b nucleo_wb55rg --pristine
west flash
```



* https://github.com/edmundsj/MCP3561DevBoard/blob/master/mcp3561_arduino/mcp3561.h
* https://github.com/MoustachedBird/ESP32_seismic_datalogger/blob/main/main/spi_mcp356x.c
* https://github.com/KenthJohan/zephyrtest/blob/master/src/tmc2130.h





## NOTES
Using the following
```
CONFIG_ASSERT=y
CONFIG_ASSERT_VERBOSE=y
```
for the board `nrf52840dongle_nrf52840` in prj.conf does not work for unknown reason.