# iot_bwt

ESP32 based IoT device to monitor usage of a BWT water filter and report it to Home Assistant via MQTT with TLS and authentication.

## Components

This code is running on a [FireBeetle ESP32 v4.0](https://www.dfrobot.com/product-1590.html) and is connected to a YF-S201 flow sensor along with a no-name 1100mAh LiPo battery. Connect the pins of YF-S201 like this: red - 3.3V, black - GND, yellow - `D2`/`IO25`on the FireBeetle board and use the JST PH 2-Pin battery terminal already on the board for the battery.

On the FireBeetle to get battery monitoring on pin A0 to work R10 and R11 need to be shorted (these are 0R resistors in the schematic). Reference and schematic: https://www.dfrobot.com/forum/viewtopic.php?t=19292

Flashing the FireBeetle with Arduino IDE: set upload speed to 921600 and flash frequency to 80MHz.

## Dependencies

* [ESP32 board definitions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
* [ArduinoJson](https://arduinojson.org/)
* [PubSubClient](https://github.com/knolleary/pubsubclient/)
* [ulpcc](https://github.com/jasonful/lcc)

## Configuration

All relevant configuration values are in [config.h](config.h), fill in the necessary values to get the project working.

To be able to use secure MQTT connections the appropriate CA has to be added. The one in the code is Let's Encrypt's cross-signed intermediate CA, any pem format CA should work.
**Disclaimer:** I've only got this working got this working with directly the signing certificate, not with a complete certificate chain.

### Home Assistant

The way the code communicates with Home Assistant assumes that [MQTT Discovery](https://www.home-assistant.io/docs/mqtt/discovery/) is enabled and presents as two sensors to be used with a [Utility Meter](https://www.home-assistant.io/integrations/utility_meter/). See example of relevant configuration in [home-assistant.yaml](home-assistant.yaml)

## Acknowledgments

* Espessif's [ULP Pulse Counting Example](https://github.com/espressif/esp-idf/tree/master/examples/system/ulp)
* [C version](https://github.com/jasonful/lcc/blob/master/ulpcc/examples/pulse_cnt.c) of the above courtesy of [ulpcc](https://github.com/jasonful/lcc)
