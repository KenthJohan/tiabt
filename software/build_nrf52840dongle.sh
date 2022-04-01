#https://docs.zephyrproject.org/2.6.0/boards/arm/nrf52840dongle_nrf52840/doc/index.html
west build -b nrf52840dongle_nrf52840 --pristine
nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
        --application build/zephyr/zephyr.hex \
        --application-version 1 blinky.zip
nrfutil dfu usb-serial -pkg blinky.zip -p COM5