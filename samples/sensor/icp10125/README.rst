.. _icp10125:

ICP10125: Barometric Pressure and Temperature Sensor
############################################

Description
***********

This sample application periodically measures the sensor
temperature and pressure, displaying the 
values on the console along with a timestamp since startup.

Wiring
*******

This sample uses an external breakout for the sensor. A devicetree
overlay must be provided to identify the SPI bus and GPIO used to
control the sensor.

Building and Running
********************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/icp10125
   :board: nrf52dk_nrf52832
   :goals: build flash

Sample Output
=============

.. code-block:: console

## Default configuration

*** Booting Zephyr OS build zephyr-v3.5.0-3187-g44282d7f4801 ***
Found device "icp10125@63", getting sensor data
[00:00:00.544,433] <inf> ICP10125_SAMPLE: temp 25.31 Cel, pressure 96.260871 kPa
[00:00:00.595,184] <inf> ICP10125_SAMPLE: temp 25.32 Cel, pressure 96.260269 kPa
[00:00:00.645,935] <inf> ICP10125_SAMPLE: temp 25.32 Cel, pressure 96.260673 kPa
[00:00:00.696,655] <inf> ICP10125_SAMPLE: temp 25.33 Cel, pressure 96.260246 kPa
[00:00:00.747,406] <inf> ICP10125_SAMPLE: temp 25.34 Cel, pressure 96.261734 kPa
[00:00:00.798,156] <inf> ICP10125_SAMPLE: temp 25.34 Cel, pressure 96.260246 kPa
[00:00:00.848,907] <inf> ICP10125_SAMPLE: temp 25.33 Cel, pressure 96.260360 kPa
[00:00:00.899,658] <inf> ICP10125_SAMPLE: temp 25.35 Cel, pressure 96.262138 kPa

<repeats endlessly>
