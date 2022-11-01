# CAN Bus

## Overview
The CAN network provides communication between the various ECUs in the system. The code available is based on a set of pre-defined messages with their own identifier and payload.


The CAN bus is configured with a **bitrate of 400 Kbit/s**.

## CAN Configuration for STM32

**Clock Tree:** APB1 frequency = 24 MHz

**CAN configuration with CubeMX:**

* Prescaler = 6 (for 4MhZ)
* Time quantum (tq): tq = 1 / (f_APB1/prescaler) =  250 ns
* Bit Segment 1 (BS1) = 7 tq
* Bit Segment 2 (BS2) = 2 tq
* Synchronization Jump Width (SJW) = 1 tq

baudrate = 1 / [tq * (SJW + BS1 + BS2) ]  = 400 kBits/s

## How to Use the PICAN 2
On the Raspberry Pi the CAN uses the shield **PICAN 2**. This shield is designed by **CopperhillTech**. The procedure to configure the PICAN 2 is explained on the website of [CopperhillTech](https://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/). Samples of C and python code are available on the web site as well as a set of programs to test the configuration. For example, the traffic on `can0` can be monitored with:

~~~~
./candump can0
~~~~

Main steps to config the Pican are :

* Edit the file `/boot/firmware/config.txt` by :

~~~~
sudo nano /boot/firmware/config.txt
~~~~

* Add these 3 lines to the end of file:

~~~~
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=spi-bcm2835-overlay
~~~~

* Reboot the Raspberry Pi:

~~~~
sudo reboot
~~~~

* Initialize the CAN interface by entering:

~~~~
sudo /sbin/ip link set can0 up type can bitrate 400000
~~~~



## Description of the CAN Messages

### MOTORS_CMD

* **From:** Raspberry
* **To:** NucleoF103
* **Lenght (Bytes):** 3
* **ID:** 0x100
* **Data field:**

|Byte 0 |Byte 1 | Byte 2|
|:------|:------|:------|
|LeftRear | RightRear | Steering |

Values between 0% and 100%.

* **LeftRear: Left rear motor command**
		This byte is used to control the speed of the left rear motor. The value 0 is the maximum speed backwards. The value 50 stops the motor. The value 100 is the maximum value forward.
		

* **RightRear: Right rear motor command**
		This byte is used to control the speed of the right rear motor. The value 0 is the maximum speed backwards. The value 50 stops the motor. The value 100 is the maximum value forward.
		

* **Steering: Steering motor command**
		This byte is used to control the speed of the steering motor. The value 0 is the maximum speed right. The value 50 stops the motor. The value 100 is the maximum speed left.
		



### MOTORS_DATA (Odometry, Speed, Steering angle)

* **From:** NucleoF103
* **To:** Raspberry
* **Lenght (Bytes):** 7
* **ID:** 0x200
* **Data field:**

|Byte 0 |Byte 1 |Bytes 2-3| Bytes 4-5| Byte 6 |
|:------|:------|:------|:------|:------|
|LeftRearOdometry | RightRearOdometry | LeftRearSpeed | RightRearSpeed | SteeringAngle |

* **LeftRearOdometry: Number of magnetic sensor pulses since the last frame (left rear motor)**
* **RightRearOdometry: Number of magnetic sensor pulses since the last frame (right rear motor)**
* **LeftRearSpeed: Left Rear Motor Speed**	
	* The speed of the left rear motor in *0.01 rpm. The direction of rotation of the motor must be deduced from the command.
* **RightRearSpeed: Right Rear Motor Speed**	
	* The speed of the left rear motor in *0.01 rpm. The direction of rotation of the motor must be deduced from the command.
* **SteeringAngle: Steering Wheel Angle**	
	* Steering wheel angle between 0 (full left) and 200 (full right)

### Ultrasonic Sensors 1 (US1)

* **From:** NucleoF103
* **To:** Raspberry
* **Lenght (Bytes):** 6
* **ID:** 0x211
* **Data field:**

|Bytes 0-1 |Bytes 2-3| Bytes 4-5|
|:------|:------|:------|
|FrontLeft | FrontCenter | FrontRight|

* **FrontLeft: Front Left Ultrasonic**
	* distance in cm measured by the front left ultrasonic sensor.
* **FrontCenter: Front Center Ultrasonic**
	* distance in cm measured by the front center ultrasonic sensor.
* **FrontRight: Front Right Ultrasonic**
	* distance in cm measured by the front right ultrasonic sensor.

Ultrasonic distances are between 0cm and 536cm. **A value of 536 indicates a distance greater than or equal to 536cm**

### Ultrasonic Sensors 2 (US2)

* **From:** NucleoF103
* **To:** Raspberry
* **Lenght (Bytes):** 6
* **ID:** 0x221
* **Data field:**

|Bytes 0-1 |Bytes 2-3| Bytes 4-5|
|:------|:------|:------|
|RearLeft | RearCenter | RearRight|

* **RearLeft: Rear Left Ultrasonic**
	* distance in cm measured by the rear left ultrasonic sensor.
* **RearCenter: Rear Center Ultrasonic**
	* distance in cm measured by the rear center ultrasonic sensor.
* **RearRight: Rear Right Ultrasonic**
	* distance in cm measured by the rear right ultrasonic sensor.

Ultrasonic distances are between 0cm and 536cm. **A value of 536 indicates a distance greater than or equal to 536cm**


### Magnetic field (IMU1)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 7
* **ID:** 0x232
* **Data field:**

|Bytes 0-1 |Bytes 2-3| Bytes 4-5| Byte 6|
|:------|:------|:------|:------|
|x axis | Y axis | Z axis| Sign|

* **X axis: Magnetic field measured on the x axis in milli Gauss (mG)**
* **Y axis: Magnetic field measured on the y axis in milli Gauss (mG)**
* **Z axis: Magnetic field measured on the z axis in milli Gauss (mG)**
* **Sign :** 
	* bit 0: sign of the z axis value (0 : positive ; 1 : negative)
	* bit 1 : sign of the y axis value (0 : positive ; 1 : negative)
	* bit 2 : sign of the x axis value (0 : positive ; 1 : negative)

### Angular velocity (IMU2)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 7
* **ID:** 0x242
* **Data field:**

|Bytes 0-1 |Bytes 2-3| Bytes 4-5| Byte 6|
|:------|:------|:------|:------|
|x axis | Y axis | Z axis| Sign|

* **X axis: Angular velocity measured on the x axis in milli degree per seconde (mdps)**
* **Y axis: Angular velocity measured on the y axis in milli degree per seconde (mdps)**
* **Z axis: Angular velocity measured on the z axis in milli degree per seconde (mdps)**
* **Sign :** 
	* bit 0 : sign of the z axis value (0 : positive ; 1 : negative)
	* bit 1 : sign of the y axis value (0 : positive ; 1 : negative)
	* bit 2 : sign of the x axis value (0 : positive ; 1 : negative)

### Linear acceleration (IMU3)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 7
* **ID:** 0x252
* **Data field:**

|Bytes 0-1 |Bytes 2-3| Bytes 4-5| Byte 6|
|:------|:------|:------|:------|
|x axis | Y axis | Z axis| Sign|

* **X axis: Linear acceleration measured on the x axis in milli g (mg)**
* **Y axis: Linear acceleration measured on the y axis in milli g (mg)**
* **Z axis: Linear acceleration measured on the z axis in milli g (mg)**
* **Sign :** 
	* bit 0 : sign of the z axis value (0 : positive ; 1 : negative)
	* bit 1 : sign of the y axis value (0 : positive ; 1 : negative)
	* bit 2 : sign of the x axis value (0 : positive ; 1 : negative)


### General data (IMU4)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 5
* **ID:** 0x263
* **Data field:**

|Bytes 0-1 |Bytes 2-3| Byte 4|
|:------|:------|:------|
|Temperature | Pressure | Humidity|

* **Temperature: Temperature measured in \*10 °C**
* **Pressure: Pressure measured in hPa**
* **Humidity: Humidity measured %**

### GPS latitude (GPS1)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 8
* **ID:** 0x272
* **Data field:**

|Byte 0-7|
|:------|
|Latitude |

* **Latitude : Raw data from the GPS [decimal degree]**

### GPS longitude (GPS2)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 8
* **ID:** 0x282
* **Data field:**

|Byte 0-7|
|:------|
|Longitude |

* **Longitude : Raw data from the GPS [decimal degree]**

### GPS altitude (GPS3)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 8
* **ID:** 0x292
* **Data field:**

|Byte 0-7|
|:------|
|Altitude |

* **Altitude : Raw data from the GPS [meters]**

### GPS Status (GPS4)

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 7
* **ID:** 0x302
* **Data field:**

|Byte 0|Byte 1-3|Byte 4-6|
|:------|:------|:------|
|Quality | hAcc | vAcc|

* **Quality : GPS fix status**
  - 0 = No fix
  - 1 = Autonomous GNSS fix
  - 2 = Differential GNSS fix
  - 4 = RTK Fixed
  - 5 = RTK Float
  - 6 = Estimated/dead reckoning fix

* **hAcc : Estimated horizontal accuracy [mm]**
* **vAcc : Estimated vertical accuracy [mm]**


### Battery Level

* **From:** NucleoL476
* **To:** Raspberry
* **Lenght (Bytes):** 1
* **ID:** 0x273
* **Data field:**

|Byte 0|
|:------|
|Battery Level |

* **Battery level : Raw data from the battery sensor**\
The value is between 0 and 0xFFF. The battery level U(V) can be computed by U = batMes * (11.65/2794.0). The nominal operation of the battery has to be between 11 and 14 V.


### Calibration Mode

* **From:** NucleoF103 / Raspberry
* **To:** Raspberry / NucleoF103
* **Lenght (Bytes):** 2
* **ID:** 0x400
* **Data field:**

|Byte 0| Byte 1|
|:------|:------|
|CalibrationStatus| UserNeed|

* **CalibrationStatus : Status of the calibration**
	* 0x1 -> Calibration request
	* 0x2 -> Calibration in progress
	* 0x3 -> Calibration success
	* 0x4 -> Calibration failure
	
* **UserNeed : Indicates if user intervention is required**
	* 0x0 -> No user intervention required
	* 0x1 -> User intervention required


### Communication check

* **From:** NucleoF103 / NucleoL476 / Raspberry
* **To:** Raspberry / NucleoF103 / NucleoL476
* **Lenght (Bytes):** 2
* **ID:** 0x410
* **Data field:**

|Byte 0| Byte 1|
|:------|:------|
|Request| Response|

* **Request :**
	* 0x1 -> Communication request : from Raspberry
	
* **Response :**
	* 0x1 -> NucleoF103 ACK
	* 0x2 -> NucleoL476 ACK


## IDs of the CAN Messages

|Name                        |Class ID |SubClass ID|Priority |ID    |
|----------------------------|:-------:|:---------:|:-------:|:----:|
|                            |3bits    |4bits      |4bits    |11bits|
|MOTORS_CMD		     |0x1      |0x0        |0x0      |0x100 |
|MOTORS_DATAS	             |0x2      |0x0        |0x0      |0x200 |
|Ultrasonic Sensors 1 (US1)  |0x2      |0x1        |0x1      |0x211 |
|Ultrasonic Sensors 2 (US2)  |0x2      |0x2        |0x1      |0x221 |
|IMU1 : Magnetic field	     |0x2      |0x3        |0x2      |0x232 |
|IMU2 : Angular velocity     |0x2      |0x4        |0x2      |0x242 |
|IMU3 : Linear acceleration  |0x2      |0x5        |0x2      |0x252 |
|IMU4 : General		     |0x2      |0x6        |0x3      |0x263 |
|Battery Level		     |0x2      |0x7        |0x3      |0x273 |
|GPS1 : Latitude	     |0x2      |0x7        |0x2      |0x272 |
|GPS2 : Longitude	     |0x2      |0x8        |0x2      |0x282 |
|GPS3 : Altitude	     |0x2      |0x9        |0x2      |0x292 |
|GPS4 : Status		     |0x3      |0x0        |0x2      |0x302 |
|Calibration Mode	     |0x4      |0x0        |0x0      |0x400 |
|Communication check	     |0x4      |0x1        |0x0      |0x410 |
