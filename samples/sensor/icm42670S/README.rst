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

Through KConfig also, the application runs the Air-Motion Library for managing 
mouse cursor motion from 3 axes gyroscope and 3 axes accelerometer. The library 
is intended to be used in free space pointing devices to operate in-air point 
and click navigation, just like a classic 2D mouse will do on a desk. 
Air Motion Library is capable of :
** Pointing : Converts motion sensors data into delta X and delta Y pointer movements.
** Gesture recognition: Detects 'Up', 'Down', 'Left', 'Right', 'Clockwise' and 'CounterClockwise' swipes, the device position and the device static.
** Gyroscope offsets calibration: Computes gyroscope offsets.
** Quaternion computing; Computes the orientation of the device.
For more information, see zephyr\drivers\sensor\icm42670S\invn.algo.sw-integration.aml-42670S-gcc-zephyr-1.6.0\include\invn_algo_aml.h

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

## Default configuration

Configured for IMU sampling.
[00:00:00.710,571] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[00:00:00.711,090] <dbg> : icm42670S_attr_set: Set accel full scale to: 2 G
[00:00:00.711,547] <dbg> : icm42670S_attr_set: Set gyro fullscale to: 1000 dps
[0:00:01.716]: temp 23.00 Cel   accel 0.150839 -0.140065 9.994899 m/s/s   gyro  -0.001597 0.005859 0.001597 rad/s
[0:00:01.726]: temp 23.00 Cel   accel 0.140065 -0.146050 9.988914 m/s/s   gyro  -0.002663 0.005859 0.003195 rad/s
[0:00:01.736]: temp 23.50 Cel   accel 0.146050 -0.130487 9.988914 m/s/s   gyro  -0.001597 0.006391 0.003195 rad/s
[0:00:01.746]: temp 23.00 Cel   accel 0.149642 -0.136473 9.990111 m/s/s   gyro  -0.002663 0.004261 0.002663 rad/s
[0:00:01.756]: temp 23.00 Cel   accel 0.146050 -0.136473 9.979337 m/s/s   gyro  -0.002130 0.005326 0.001597 rad/s
[0:00:01.766]: temp 23.00 Cel   accel 0.136473 -0.147247 9.986519 m/s/s   gyro  -0.001065 0.005859 0.002663 rad/s


## ICM42670S_APEX = y
## APEX_FEATURES = ICM42670S_APEX_PEDOMETER

Configured for APEX data collecting.
[00:00:00.591,796] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[0:00:09.287]: STEP_DET     count: 6 steps  cadence: 2.0 steps/s  activity: Unknown
[0:00:09.689]: STEP_DET     count: 7 steps  cadence: 2.1 steps/s  activity: Walk
[0:00:10.051]: STEP_DET     count: 8 steps  cadence: 2.2 steps/s  activity: Walk
[0:00:10.433]: STEP_DET     count: 9 steps  cadence: 2.2 steps/s  activity: Walk
[0:00:10.835]: STEP_DET     count: 10 steps  cadence: 2.3 steps/s  activity: Walk
[0:00:11.297]: STEP_DET     count: 11 steps  cadence: 2.3 steps/s  activity: Walk


## ICM42670S_APEX = y
## APEX_FEATURES = ICM42670S_APEX_TILT
Configured for APEX data collecting.
[00:00:00.706,817] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[0:00:15.249]: TILT
[0:00:21.479]: TILT
[0:00:26.765]: TILT


## ICM42670S_APEX = y
## APEX_FEATURES = ICM42670S_APEX_WOM

Configured for APEX data collecting.
[00:00:00.707,336] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[0:00:02.555]: WOM x=1 y=0 z=1
[0:00:02.636]: WOM x=0 y=0 z=1
[0:00:02.797]: WOM x=0 y=1 z=0
[0:00:02.877]: WOM x=0 y=0 z=1
[0:00:02.957]: WOM x=1 y=1 z=1


## ICM42670S_APEX = y
## APEX_FEATURES = ICM42670S_APEX_SMD

Configured for APEX data collecting.
[00:00:00.707,336] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[0:00:04.622]: SMD
[0:00:05.084]: SMD
[0:00:05.566]: SMD


## ICM42670S_AML = y
## AML_FEATURES = ICM42670S_AML_POINTING

Configured for AML data collecting.
[00:00:00.719,726] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[00:00:01.090,148] <dbg> ICM42670S_AML: icm42670S_aml_init: Algorithm init OK
[0:00:02.093]: AML Pointing Delta=[7, -2]
[0:00:02.104]: AML Pointing Delta=[7, -3]
[0:00:02.114]: AML Pointing Delta=[6, -2]
[0:00:02.124]: AML Pointing Delta=[3, -3]
[0:00:02.134]: AML Pointing Delta=[2, -3]
[0:00:02.144]: AML Pointing Delta=[1, -2]
[0:00:02.154]: AML Pointing Delta=[0, -2]


## ICM42670S_AML = y
## AML_FEATURES = ICM42670S_AML_GESTURES

Configured for AML data collecting.
[00:00:00.709,442] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[00:00:01.079,864] <dbg> ICM42670S_AML: icm42670S_aml_init: Algorithm init OK
[0:00:02.083]: AML Remote status: Static
[0:00:05.688]: AML Remote status: Non-static
[0:00:06.200]: AML Remote status: Static
[0:00:06.602]: AML Remote position: Top
[0:00:07.696]: AML Remote status: Non-static
[0:00:07.877]: AML Gestures: Swipe Clockwise
[0:00:07.947]: AML Gestures: Swipe Clockwise
[0:00:07.997]: AML Gestures: Swipe Clockwise


## ICM42670S_AML = y
## AML_FEATURES = ICM42670S_AML_GYR_OFFSET

Configured for AML data collecting.
[00:00:00.708,251] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[00:00:01.090,209] <dbg> ICM42670S_AML: icm42670S_aml_init: Algorithm init OK
[0:00:05.317]: AML Gyro biases: 0.094811 -0.033024 -0.034089 rad/s
[0:00:09.333]: AML Gyro biases: 0.085223 -0.030893 -0.033024 rad/s


## ICM42670S_AML = y
## AML_FEATURES = ICM42670S_AML_QUATERNION

Configured for AML data collecting.
[00:00:00.591,735] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[00:00:00.962,249] <dbg> ICM42670S_AML: icm42670S_aml_init: Algorithm init OK
[0:00:01.966]:AML Quaternion=[0.992187 -0.119140 -0.007812 -0.003906]
[0:00:01.976]:AML Quaternion=[0.992187 -0.122070 -0.007812 -0.003906]
[0:00:01.986]:AML Quaternion=[0.992187 -0.123046 -0.007812 -0.003906]
[0:00:01.996]:AML Quaternion=[0.992187 -0.125000 -0.007812 -0.003906]
[0:00:02.006]:AML Quaternion=[0.991210 -0.125976 -0.007812 -0.003906]
[0:00:02.016]:AML Quaternion=[0.991210 -0.126953 -0.007812 -0.003906]


<repeats endlessly>
