

# ESP32_WIFI_ETH_BRIDGE_HUE

## Description
* Uses a [WT32-ETH01 ESP32](https://github.com/egnor/wt32-eth01) to connect a Hue Bridge by bridging the ethernet and wifi.

## Issues Encountered
* If you're using ethernet and flashing to the [WT32-ETH01](https://github.com/egnor/wt32-eth01/tree/main?tab=readme-ov-file), remember to disconnect the connector from GPIO_NUM_0 when done flashing. If you don't, this causes an issue with initializing the PHY because the 50,000 MHz clock is being interfered with

## Install
* Use `git clone https://github.com/helghast098/wifi_eth_bridge_hue.git`
  
## Run
* The project is ready to run, so just use `idf.py -p [device] flash monitor` to flash the device and monitor the flashed target

## Configuration
Configuration toggles available to user:
* `WIFI_NAME`/`WIFI_PASSWORD`
  
DEFAULT: `WIFI_NAME`
* `` 

DEFAULT: `WIFI_PASSWORD`
* ``

## LICENSE
* This project is licensed under the MIT License - see the LICENSE.md file for details
