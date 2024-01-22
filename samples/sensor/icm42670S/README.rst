.. _icm42670S:

ICM42670S: Invensense Motion Tracking Device
############################################

Description
***********

This sample application periodically (100 Hz) measures the sensor
temperature, acceleration, and angular velocity, displaying the 
values on the console along with a timestamp since startup.

Through KConfig, the application starts the APEX (Advanced Pedometer 
and Event Detection) features. It consists of:
** Pedometer: Tracks step count.
** Tilt Detection: Detects the Tilt angle exceeds 35 degrees during 4s still. 
** Wake on Motion (WoM): Detects motion when accelerometer samples exceed 
a programmable threshold. This motion event can be used to enable device 
operation from sleep mode.
** Significant Motion Detector (SMD): Detects significant motion based on
accelerometer data.

Wiring
*******

This sample uses an external breakout for the sensor.  A devicetree
overlay must be provided to identify the SPI bus and GPIO used to
control the sensor.

Building and Running
********************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/icm42670S
   :board: nrf52dk_nrf52832
   :goals: build flash

Sample Output
=============

.. code-block:: console

Configured for IMU data collecting.
[0:00:27.656]: temp 24.00 Cel   accel -0.418998 -1.339596 9.949408 m/s/s   gyro  0.012250 -0.026099 -0.010652 rad/s
[0:00:27.666]: temp 24.00 Cel   accel -0.566245 -1.303682 9.847651 m/s/s   gyro  0.022371 -0.000532 -0.017577 rad/s
[0:00:27.676]: temp 24.00 Cel   accel -0.664411 -1.312062 9.854834 m/s/s   gyro  0.025034 0.013848 -0.015446 rad/s
[0:00:27.686]: temp 24.00 Cel   accel -0.630891 -1.277345 9.840469 m/s/s   gyro  0.020240 0.000000 -0.014381 rad/s
[0:00:27.696]: temp 24.00 Cel   accel -0.648848 -1.267768 9.890748 m/s/s   gyro  0.004793 -0.010120 -0.006924 rad/s
[0:00:27.706]: temp 24.00 Cel   accel -0.659622 -1.261782 9.994899 m/s/s   gyro  -0.009587 0.005859 -0.001065 rad/s
[0:00:27.716]: temp 24.00 Cel   accel -0.642862 -1.273754 9.997294 m/s/s   gyro  -0.015446 0.036220 0.013848 rad/s
[0:00:27.726]: temp 24.00 Cel   accel -0.736239 -1.309668 9.981731 m/s/s   gyro  -0.009587 0.037285 0.028230 rad/s
[0:00:27.736]: temp 24.00 Cel   accel -0.711099 -1.367130 9.955394 m/s/s   gyro  0.009587 -0.002663 0.031426 rad/s
[0:00:27.746]: temp 24.00 Cel   accel -0.633285 -1.397059 9.872791 m/s/s   gyro  0.031426 -0.020773 0.026632 rad/s
[0:00:27.756]: temp 24.00 Cel   accel -0.599765 -1.367130 9.823709 m/s/s   gyro  0.036220 -0.014914 0.015979 rad/s
[0:00:27.766]: temp 24.00 Cel   accel -0.739830 -1.345582 9.935042 m/s/s   gyro  0.027165 -0.015446 0.017577 rad/s
[0:00:27.777]: temp 24.00 Cel   accel -0.957709 -1.306076 9.978139 m/s/s   gyro  0.005859 -0.045807 0.037818 rad/s
[0:00:27.787]: temp 24.00 Cel   accel -0.792504 -1.288119 9.975745 m/s/s   gyro  -0.029295 -0.085223 0.054862 rad/s


## ICM42670S_APEX = y
## APEX_FEATURES = ICM42670S_APEX_PEDOMETER

Configured for APEX data collecting.
[0:00:07.614]: STEP_DET     count: 6 steps  cadence: 2.0 steps/s  activity: Unknown
[0:00:07.975]: STEP_DET     count: 7 steps  cadence: 2.0 steps/s  activity: Walk
[0:00:08.337]: STEP_DET     count: 8 steps  cadence: 2.1 steps/s  activity: Walk
[0:00:08.719]: STEP_DET     count: 9 steps  cadence: 2.3 steps/s  activity: Walk
[0:00:09.121]: STEP_DET     count: 10 steps  cadence: 2.2 steps/s  activity: Walk
[0:00:09.482]: STEP_DET     count: 11 steps  cadence: 2.4 steps/s  activity: Walk
[0:00:09.844]: STEP_DET     count: 12 steps  cadence: 2.4 steps/s  activity: Run
[0:00:10.226]: STEP_DET     count: 13 steps  cadence: 2.4 steps/s  activity: Run 


## ICM42670S_APEX = y
## APEX_FEATURES = ICM42670S_APEX_TILT

Configured for APEX data collecting.
[0:12:38.342]: TILT
[0:12:43.767]: TILT
[0:13:57.641]: TILT
[0:14:03.005]: TILT


<repeats endlessly>
