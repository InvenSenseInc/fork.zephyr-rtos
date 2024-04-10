.. _icp201xx:

ICP201xx: Pressure change interrupt sample
############################################

Description
***********

This sample application sets the icp201xx pressure change
interrupt to detect a pressure change between 2 samples
greater than 10 Pa. A message is printed on the UART when
the sensor detect a this pressure change.

Wiring
*******

This sample uses an external breakout for the sensor.  A devicetree
overlay must be provided to identify the I2C/SPI bus and GPIO used to
control the sensor.

Building and Running
********************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/icp201xx
   :board: nrf52dk_nrf52832
   :goals: build flash

Sample Output
=============

.. code-block:: console

## Default configuration

*** Booting Zephyr OS build zephyr-v3.5.0-3418-g42ed70b772b8 ***
Found device "icp201xx@63", getting sensor data
[00:00:01.122,528] <inf> ICP201XX_SAMPLE: Starting ICP201xx pressure change interrupt sample.
[00:00:05.899,810] <inf> ICP201XX_SAMPLE: PRESSURE CHANGE INTERRUPT
[00:00:10.014,221] <inf> ICP201XX_SAMPLE: PRESSURE CHANGE INTERRUPT

<repeats endlessly>
