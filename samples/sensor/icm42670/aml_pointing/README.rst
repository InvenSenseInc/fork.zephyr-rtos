.. _icm42670:

ICM42670-S: Invensense Motion Tracking Device
#############################################

Overview
********

This sample application runs the Air-Motion Library for managing 
mouse cursor motion from 3 axes gyroscope and 3 axes accelerometer. 
The library is intended to be used in free space pointing devices 
to operate in-air point and click navigation, just like a classic 
2D mouse will do on a desk. 
Air Motion Library is capable of :
** Pointing : Converts motion sensors data into delta X and delta Y 
pointer movements.
** Gesture recognition: Detects 'Up', 'Down', 'Left', 'Right', 
'Clockwise' and 'CounterClockwise' swipes, the device position and 
the device static.
** Gyroscope offsets calibration: Computes gyroscope offsets.
** Quaternion computing; Computes the orientation of the device.
For more information, see documentation:
- zephyr\
	drivers\
	  sensor\
		icm42670\invn.algo.sw-integration.aml-42670S-gcc-zephyr-1.6.0\
		  InvnAlgoAML.pdf

Driver configuration
********************

The Air Motion Library is bonded to the ICM42670-S device. At initialization 
stage, the driver configures the highest FSR for accelerometer and gyroscope
and the algorithm operating frequency to 100Hz. 

Wiring
*******

This sample uses an external breakout for the sensor. A devicetree
overlay must be provided to identify the SPI or I2C bus and the interrupt 
sensor GPIO.

Building and Running instructions
*********************************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

.. zephyr-app-commands:
   :zephyr-app: samples/sensor/icm42670/aml_pointing
   :board: nrf52dk/nrf52832
   :goals: build flash

Sample Output
=============

.. code-block:: console

## Default configuration
## ICM42670S_AML_POINTING=y

*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670@68", getting sensor data
[0:00:02.093]: AML Pointing Delta=[7, -2]
[0:00:02.104]: AML Pointing Delta=[7, -3]
[0:00:02.114]: AML Pointing Delta=[6, -2]
[0:00:02.124]: AML Pointing Delta=[3, -3]
[0:00:02.134]: AML Pointing Delta=[2, -3]
[0:00:02.144]: AML Pointing Delta=[1, -2]
[0:00:02.154]: AML Pointing Delta=[0, -2]

<repeats endlessly>

## ICM42670S_AML_GESTURES=y

*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670@68", getting sensor data
[0:00:02.083]: AML Remote status: Static
[0:00:05.688]: AML Remote status: Non-static
[0:00:06.200]: AML Remote status: Static
[0:00:06.602]: AML Remote position: Top
[0:00:07.696]: AML Remote status: Non-static
[0:00:07.877]: AML Gestures: Swipe Clockwise
[0:00:07.947]: AML Gestures: Swipe Clockwise
[0:00:07.997]: AML Gestures: Swipe Clockwise

<repeats endlessly>

## ICM42670S_AML_GYR_OFFSET=y

*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670@68", getting sensor data
[0:00:05.317]: AML Gyro biases: 0.094811 -0.033024 -0.034089 rad/s
[0:00:09.333]: AML Gyro biases: 0.085223 -0.030893 -0.033024 rad/s

<repeats endlessly>

## ICM42670S_AML_QUATERNION=y

*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670@68", getting sensor data
[0:00:01.966]:AML Quaternion=[0.992187 -0.119140 -0.007812 -0.003906]
[0:00:01.976]:AML Quaternion=[0.992187 -0.122070 -0.007812 -0.003906]
[0:00:01.986]:AML Quaternion=[0.992187 -0.123046 -0.007812 -0.003906]
[0:00:01.996]:AML Quaternion=[0.992187 -0.125000 -0.007812 -0.003906]
[0:00:02.006]:AML Quaternion=[0.991210 -0.125976 -0.007812 -0.003906]
[0:00:02.016]:AML Quaternion=[0.991210 -0.126953 -0.007812 -0.003906]

<repeats endlessly>
