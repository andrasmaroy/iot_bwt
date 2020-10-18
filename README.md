# iot_bwt

ESP32 based IoT device to monitor usage of a BWT filter and report it to Home Assistant via MQTT.

## Components

This code is running on a [FireBeetle ESP32 v4.0](https://www.dfrobot.com/product-1590.html) and is connected to a YF-S201 flow sensor along with a no-name 1100mAh LiPo battery.

On the FireBeetle to get battery monitoring on pin A0 to work R10 and R11 need to be shorted (these 0R resistors in the schematic). Reference and schematic: https://www.dfrobot.com/forum/viewtopic.php?t=19292

## Dependencies

* [ESP32 board definitions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
* [ArduinoJson](https://arduinojson.org/)
* [PubSubClient](https://github.com/knolleary/pubsubclient/)
* [ulpcc](https://github.com/jasonful/lcc)

## Configuration

All relevant configuration values are listed at the beginning of [iot_bwt.ino](iot_bwt.ino), fill in the necessary values to get the project working.

To be able to use secure MQTT connections the appropriate CA has to be added. The one in the code is Let's Encrypt's cross-signed intermediate CA, any pem format CA should work.

## Acknowledgments

* Espessif's [ULP Pulse Counting Example](https://github.com/espressif/esp-idf/tree/master/examples/system/ulp)
* [C version](https://github.com/jasonful/lcc/blob/master/ulpcc/examples/pulse_cnt.c) of the above courtesy of [ulpcc](https://github.com/jasonful/lcc)
