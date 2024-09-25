.. _ict1531x:

ICT1531x: High precision 3-axis electronic magnetometer sensor IC
####################################################

Description
***********

This sample application periodically measures the sensor
temperature and magnetoc field, displaying the
values on the console along with a timestamp since startup.

Wiring
*******

This sample uses an external breakout for the sensor.  A devicetree
overlay must be provided to identify the I2C used to control the sensor.

Building and Running
********************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/ict1531x
   :board: nrf52dk_nrf52832
   :goals: build flash

Sample Output
=============

.. code-block:: console

## Default configuration

Found device "ict1531x@1E", getting sensor data[00:00:00.266,693] <inf> ICT1531X_SAMPLE: Starting ICT1531x sample.
[00:00:00.778,472] <inf> ICT1531X_SAMPLE: temp=25.00 Cel, mag_x=0.00 Gs, mag_y=0.00 Gs, mag_z=0.00 Gs
[00:00:00.788,635] <inf> ICT1531X_SAMPLE: temp=25.40 Cel, mag_x=-0.13 Gs, mag_y=-0.04 Gs, mag_z=-0.28 Gs
[00:00:00.798,797] <inf> ICT1531X_SAMPLE: temp=25.40 Cel, mag_x=-0.13 Gs, mag_y=-0.04 Gs, mag_z=-0.28 Gs
[00:00:00.808,959] <inf> ICT1531X_SAMPLE: temp=25.40 Cel, mag_x=-0.14 Gs, mag_y=-0.04 Gs, mag_z=-0.28 Gs
[00:00:00.819,122] <inf> ICT1531X_SAMPLE: temp=25.40 Cel, mag_x=-0.14 Gs, mag_y=-0.04 Gs, mag_z=-0.28 Gs
[00:00:00.829,284] <inf> ICT1531X_SAMPLE: temp=25.40 Cel, mag_x=-0.13 Gs, mag_y=-0.04 Gs, mag_z=-0.28 Gs
[00:00:00.839,447] <inf> ICT1531X_SAMPLE: temp=25.40 Cel, mag_x=-0.13 Gs, mag_y=-0.04 Gs, mag_z=-0.28 Gs

<repeats endlessly>
