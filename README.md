# OtterPilot v3.7.5 Hardware Test

Production Hardware Testing for OtterPilot v3.7.5.

CrayUncleBurton.com OtterWorks
OtterPilot v3.7.5
Bryan A. "CrazyUncleBurton" Thompson
Last Updated 1/23/2025

Github Repository:  OtterPilot-V3.7.5-Hardware-Test
<https://github.com/CrazyUncleBurton/OtterPilot-V3.7.5-Hardware-Test>

## Warning

This is code that the hardware manufacturer uses to test the hardware.  

This is NOT firmware for the OtterPilot v3.7.5!
  
The production firmware is located Here:

Github Repository:  OtterPilot-v3.6.0
<https://github.com/CrazyUncleBurton/OtterPilot-v3.6.0>

## Test Setup

1) Connect RX1 receiver input to the SERVO1 Steering servo output.
2) Connect RX2 receiver input to the SERVO2 Throttle servo output.
3) Connect RX3 receiver input to the SERVO3 Channel 3 servo output.
4) Connect the OtterPilot to anything that can supply 5V to the USB-C port.  Note:  Powering the device with USB will result in a low voltage reading because of the backfeed diode.  If it measures 4.3+V, it's probably fine.  Power with the BEC from the receiver to be sure.
5) Connect the GPS to the external I2C port (the white connector on the black microcontroller board).
6) If used, connect the SparkFun OpenSD MicroSD card to the external GPS open port (the white connector on the black microcontroller board).

Note:  The tests are automated.  If the test passes, the text is green.  If the test detects a fixable issue, the text is yellow.  If the text fails, the text is red.  Watch the test a few times to be sure those things are working before proceeding.
8) Watch the PCA9685 line - it should be green.
9) Watch the LSM6DSOX line - it should be green.
10) Watch the GPS line - it should be green.
11) Watch the OpenLog line - it should be green if the OpenLog is connected.
12) Watch the Steering line - it should be green.
13) Watch the Throttle line - it should be green.
14) Watch the Channel 3 line - it should be green.
15) Disconnect the RX1 and SERVO1 connectors.  
16) Connect the RX1 receiver input to the SERVO4 Steering servo output.
17) Watch the Steering/Aux test and make sure it still succeeds.  We know RX1 is good, so we are testing SERVO4 output here.

## I2C Info

### External I2C Bus Connections

External Bus:  SCA = G44, SCL = G43, speed = 750kHz for GPS

### External I2C QUIIC Connector on LilyGo board

1. +3V
2. GND
3. External SDA Pin G43
4. External SCL Pin G44

The external I2C Bus Speed 750kHz for GPS and OpenLog MicroSD devices.  Connect both the QWIIC connector on the LilyGo CPU Board, along with anything which is connected to the external QWIIC Connector.

### Internal I2C Bus Connections

Internal Bus:  SDA = G18, SCL = G17, speed = 1MHz for everything else

#### Internal I2C QWIIC Connector on OtterPilot Board

1. +3V3_SENSOR
2. GND
3. Internal SDA Pin G18
4. Internal SCL Pin G17

The internal I2C Bus Speed is 1MHz.  Everything on the OtterPilot board is connected to the INTERNAL I2C bus (including the accelerometer, the magnetometer, and anything connected to the QWIIC connector on the OtterPilot board).  

### System I2C Map

* 0x10 - PA1010D GPS - Internal or External I2C bus
* 0x15 - CST816S Touch Controller - permanently connected to the Internal I2C bus
* 0x2A - Atmega 328P Used by OpenLogger MicroSD Card - Internal or External I2C bus
* 0x40 - PCA9685 PWM Servo Controller - permanently connected to the Internal I2C bus
* 0x6A - LSM6DSOX - 6DOF IMU - permanently connected to the Internal I2C bus
* 0x70 - Something to do with the PCA9685 - permanently connected to the Internal I2C bus - might be a ghost?
* 0x7E - Something to do with the Adafruit IMU board - permanently connected to the Internal I2C bus - might be a ghost?

### OtterPilot Connector Inputs and Outputs Working

#### System GPIO Connections

* G1 - G1_3V3 - Unused
* G2 - CAL_INPUT_G2_3V3 - Connected to PWM2 Output on PCA9685
* G3 - TRAXXAS_LIGHTING_G3_3V3 - connected to Traxxas lighting connector marked L
* G10 - VIN_MEAS_G10_3V3 - Connected to VIN measurement divider
* G11 - RX_CH3_G11_3V3 - Aux Reciever Input - Connected to Receiver Input marked RA
* G12 - RX_CH2_G12_3V3 - Throttle Receiver Input - Connected to Receiver Input marked RT
* G13 - RX_CH1_G13_3V3 - Steering Receiver Input - Connected to Receiver Input marked RS
* G16 - G16_3V3 - ESP TP INT - Touch Controller Interrupt
* G17 - INT_SCL_G17_3V3 - Internal SCL Connector mounted to OtterPilot Board
* G18 - INT_SDA_G18_3V3 - Internal SDA Connector mounted to OtterPilot Board
* G21 - G21_3V3 - ESP TP RESET - Touch Controller Reset
* G43 - EXT_SDA_G43_3V3 - External I2C SDA Connector Mounted to LilyGo CPU Board
* G44 - EXT_SDA_G44_3V3 - External I2C SCL Connector Mounted to LilyGo CPU Board

### VIN Voltage Test

* The connections for this are internal on the OtterPilot v3.5.0
* G10_VIN_MEAS is the GPIO which measures this voltage.
* This is an analog voltage measurement, which is not the ESP32's best feature.  It will be close-ish.
* When the OtterPilot is powered by the USB port (i.e. during programming), VIN will be dropped by 0.4-ish volts due to the Schottly diode D2 which prevents backfeeding excess voltage into the USB port of the computer.  The VIN_MEAS test will rightly fail during this test.

### Steering Servo Output Test

* PWM Output 0 on PCA9685 - Steering Servo Output - Connected to Output Connector Marked SERVO1

### Throttle ESC/Servo Output Test

* PWM Output 1 on PCA9685 - Throttle / ESC Output - Connected to Output Connector Marked SERVO2

### Cal Output Test

* PWM Output 4 on PCA9685 - Calibration Output - Connected to G2 Input on microcontroller
