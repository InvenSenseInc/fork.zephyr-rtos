.. _icm42670S:

ICM42670S: Invensense Motion Tracking Device
############################################

Ovreview
********

This sample application starts the APEX (Advanced Pedometer 
and Event Detection) features. It consists of:
** Pedometer: Tracks step count.
** Tilt Detection: Detects the Tilt angle exceeds 35 degrees during 4s still. 
** Wake on Motion (WoM): Detects motion when accelerometer samples exceed 
a programmable threshold. This motion event can be used to enable device 
operation from sleep mode.
** Significant Motion Detector (SMD): Detects significant motion based on
accelerometer data.
Each feature is enabled through KConfig choice.

Wiring
*******

This sample uses an external breakout for the sensor.  A devicetree
overlay must be provided to identify the SPI or I2C bus and the interrupt 
sensor GPIO.

Building and Running instructions
*********************************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

.. zephyr-app-commands:
   :zephyr-app: samples/sensor/icm42670S/apex
   :board: nrf52dk_nrf52832
   :goals: build flash

Sample Output
=============

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


## APEX_FEATURES = ICM42670S_APEX_TILT

Configured for APEX data collecting.
[00:00:00.706,817] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[0:00:15.249]: TILT
[0:00:21.479]: TILT
[0:00:26.765]: TILT


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


## APEX_FEATURES = ICM42670S_APEX_SMD

Configured for APEX data collecting.
[00:00:00.707,336] <dbg> : icm42670S_chip_init: "icm42670S@68" OK
*** Booting Zephyr OS build zephyr-v3.5.0-3192-g528359f60dd9 ***
Found device "icm42670S@68", getting sensor data
[0:00:04.622]: SMD
[0:00:05.084]: SMD
[0:00:05.566]: SMD


