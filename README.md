# X-NUCLEO-53L1A2

Arduino library to support the X-NUCLEO-53L1A2 based on VL53L1 ranging sensor with advanced multi-zone and multi-object detection.
This sensor uses I2C to communicate. An I2C instance is required to access to the sensor.
The APIs provide simple distance measure and multi-object detection in both polling and interrupt modes.

## Examples

There are 2 examples with the X-NUCLEO-53L1A2 library.

* X_NUCLEO_53L1A2_HelloWorld: This example code is to show how to get multi-object detection and proximity
  values of the onboard VL53L1 sensor in polling mode.

* X_NUCLEO_53L1A2_HelloWorld_Interrupt: This example code is to show how to get multi-object detection and proximity
  values of the onboard VL53L1 sensor in interrupt mode.

## Dependencies

This package requires the following Arduino libraries:

* STM32duino VL53L1: https://github.com/stm32duino/VL53L1
  
## Note

The maximum detection distance is influenced by the color of the target and
the indoor or outdoor situation due to absence or presence of external
infrared. The detection range can be comprise between ~40cm and ~400cm.

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-53L1A2

The VL53L1 datasheet is available at  
https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l1.html
