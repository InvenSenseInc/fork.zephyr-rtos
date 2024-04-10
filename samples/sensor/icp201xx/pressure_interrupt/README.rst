.. _icp201xx:

ICP201xx: Pressure interrupt sample
############################################

Description
***********

This sample application measures the pressure and temperature
at reset and estimates the corresponding altitude.
It sets the pressure interrupt from ICP201XX sensor to detect
the pressure corresponding to a 50cm higher altitude.
A message is printed on the UART when the sensor is moved 50cm higher.

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
[00:00:01.124,877] <inf> ICP201XX_SAMPLE: Starting ICP201xx threshold interrupt sample.
[00:00:43.961,303] <inf> ICP201XX_SAMPLE: THRESHOLD INTERRUPT
[00:00:47.716,583] <inf> ICP201XX_SAMPLE: THRESHOLD INTERRUPT
