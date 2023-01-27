# Overview of the NucleoF103 software architecture

## Contents
1. Pin configuration
2. How to manage sensors and actuators
3. How to use the CAN bus
4. Scheduling
5. Steering calibration

## 1. Pin configuration

The Nucleo board controls the vehicle's engines and steering, the ultrasonic sensors and the battery level. The full description of the port configuration is provided in the file [nucleoF103_hardware_interface](../hardware/nucleoF103_hardware_interface.pdf)


### Power switch
| Port | Configuration | Remap | Description                     |
|------|---------------|-------|---------------------------------|
| PA6  | Output PP     | -     |Control the power                |


### Sensors
| Port | Configuration | Remap | Description                     |
|------|---------------|-------|---------------------------------|
| PA0  | Analog Input  | ADC1 IN0 |Battery Level             |
| PA1  | Analog Input  | ADC1 IN1 | Steering wheel angle     |
| PB10 | AF Output PP  | TIM2 CH3 | Position left motor |
| PB8 | AF Output PP  | TIM4 CH3 | Position right motor   |

### Left wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC10 | Output PP 10MHz| -        | Enable left motor         |
| PA8  | AF Output PP  | TIM1 CH1 |  Control left motor (IN1) |
| PA7  | AF Output PP  | TIM1 CH1N|  Control left motor (IN2) |
| PA4  | Analog Input  | ADC1 IN4 | Current left motor        |

### Right wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC11 | Output PP 10MHz| -        | Enable right motor         |
| PA9  | AF Output PP  | TIM1 CH2 |  Control right motor (IN1) |
| PB0  | AF Output PP  | TIM1 CH2N|  Control right motor (IN2) |
| PA5  | Analog Input  | ADC1 IN5 | Current right motor        |

### Steering wheel manual buttons
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PB14 | Input Pullup  | —     | Status of the right button | 
| PB15 | Input Pullup  | —     | Status of the left button | 

### Steering wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC12 | Output PP 10MHz| -       | Enable steering motor         |
| PA10 | AF Output PP  | TIM1 CH3 |  Control steering motor (IN1) |
| PB1  | AF Output PP  | TIM1 CH3N|  Control steering motor (IN2) |
| PC0  | Analog Input  | ADC1 IN10 | Current steering motor      |

### Ultrasonic sensors
#### Trigger pins
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC1  | Output PP  | -        | Front left us trigger        |
| PC2  | Output PP  | -        | Front center us trigger      |
| PC3  | Output PP  | -        | Front right us trigger       |
| PC4  | Output PP  | -        | Rear left us trigger         |
| PC5  | Output PP  | -        | Rear center us trigger       |
| PC7  | Output PP  | -        | Rear right us trigger        |

#### Echo pins
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC6  |IT Rising Falling| -   | Right us echo                |
| PC8  |IT Rising Falling| -   | Left us echo                 |
| PC9  |IT Rising Falling| -   | Center us echo               |

### CAN bus
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PA11 | AF Output PP  | CAN RX | CAN Rx bus                  |
| PA12 | AF Output PP  | CAN TX | CN TX bus                   |


## 2. How to manage sensors and actuators

### Bootstrap power

To maintain the power it is necessary to set PA6. If the pin is reset the power of the car is off.

Functions `power_bootstrap` and  `power_shutdown`can be used to control the state of the PA6.

> [!IMPORTANT]
> When launching the DEBUG or LOAD of the STM32 card, it is necessary to manually press and hold the START button (ST-LINK imposes a reset that cuts off the latch circuit).

### Voltage measurements (battery, steering angle, left rear motor, right rear motor and steering motor)

All voltage measurements (battery, steering angle, current left rear motor, current right rear motor and current front motor)  are made in continuous mode by the ADC. The measurements are stored in a circular buffer in the DMA.

The position angle of the steering can be obtained by calling the function `int get_steering_angle(void)`. The returned steering angle is between 0 (full left) and 200 (full right).

The value for the battery is between 0 and 0xFFF. The value is between 0 and 0xFFF. The battery level U(V) can be computed by U = batMes * (11.65/2794.0). The nominal operation of the battery has to be between 11 and 14 V.

The steering angle has to be calibrate.

### Wheel motor speeds (left rear, right rear)

The `HAL_TIM_IC_CaptureCallback` function in file `main.c` calculates and updates the motor speed data (VMG_mes, VMD_mes). The value is given in *0.01 rpm.


### Motor Commands

All engines are speed controlled, even the steering wheel engine. 

The command of a motor is encoded by 8-bit data. The bit 7 is used to enable or disable the motor and bits 6-0 are used to code the value of the command. The values are between 0 and 100 with 50 to stop the motor. To avoid problems some software thresholds were implemented (25-75 for the wheels and 40-60 for the steering).

**Warning:** All motors operate in open loop. Thus, the steering  is not controled in angle.

The motor control of the wheels is done by calling the function `wheels_set_speed`. The motor control of the steering is done by calling `steering_set_speed`. These functions are periodically called in the main file.

### Ultrasonic measurements

The car is equipped with 6 ultrasonic sensors. To measure a distance, we send a 10us signal on the Trigger pin of the sensor. In return, the function `HAL_GPIO_EXTI_Callback` (file `main.c`) measures the duration of the signal on the Echo pin. This duration is directly proportional to the measured distance: **distance\[cm] = echoDuration\[us] / 58**. 

The measurements are made one by one, each measurement lasting 40ms. The measurements are updated with a period of 50ms. It can be changed by changing the value of the `PERIOD_UPDATE_US` constant in the `Inc/main.h` file. The distances of the 6 sensors are stored in the variable usDistance before being sent to the Raspberry by the CAN bus. 

* Distance range : \[4cm - 535cm]
* Maximum echo duration: 31ms


## 3. How to use the CAN bus

Sending a message on the CAN bus is done by calling the function `void CAN_Send (unint8_t* data, unint32_t id)`. 

The reception of a message is done in the callback function of the CAN interruption `HAL_CAN_RxCpltCallback` in file `can.c`. Motor controls (wheel and steering) are shared via global variables (leftRearSpeed, rightRearSpeed, steeringSpeed). These variables are updated on receipt of the CAN message. The actuators are updated periodically in the main file.

## 4. Scheduling

### Motor speed
The speed is calculated under interruption at each edge of the coder of each wheel. The clock used to calculate the speed is the timer TIM2 for the left wheel and TIM4 for the right wheel. This timer is reset at each interruption and give directly the speed. if the car does not move then the speed is set to zero after 100ms.

### Voltage measurements
Voltage measurements are continuously updated by the ADC, so there is no scheduling for these values. They can be read asynchronously. The freshness of the data is less than 1ms.

### Motor Commands
The motor consigns and the actuators are updated each time a CAN message (corresponding to the CAN_ID_MOTORS_CMD) is received. The maximum response time to react to a command is approximatively egals to the time transmission of the CAN MCM message, plus the period of the update loop.

### Sending data sensors
* Motors data (left rear and right rear odometry, left rear and right rear speeds, steering angle) are periodically send with a period egals to 100ms. It can be changed by changing the value of the `PERIOD_SEND_MOTORS` constant in the `Inc/main.h` file.
* The data from the ultrasonic sensors are sent each time the 6 sensors have completed their measurement. 
* The battery level is periodically send with a period egals to 30s. It can be changed by changing the value of the `PERIOD_SEND_BATT` constant in the `Inc/main.h` file.

## 5. Steering calibration
The steering limit switches are not symmetrical to the center of the steering. Therefore, the steering is calibrated to stop before the farthest switch from the center, in order to have a symmetrical steering on the left and on the right.\
The value of the angle (between 0 and 200) is given by : `angle = a * steering_sensor + b`\
The coefficients a and b are calculated by the `calibrate` function, available in the file 'calibrate.c'. These coefficients are stored in flash memory.\
The calibration procedure communicates with the Raspberry through the CAN bus.

**Two possibilities to start the calibration procedure:** 
After starting the ROS nodes on the Raspberry (see [how_to_use.md](../../../general/how_to_use.md), step n°3), it is possible to launch the calibration:
* from the Xbox controller ("DPAD bottom" + "Start")
* from a terminal on the Raspberry, by running:
```sh
ros2 service call /steering_calibration std_srvs/srv/Empty 
```

**Detail of the procedure :**
1. Full left steering
2. Full right steering
3. **User intervention:** manually center the steering wheels (using the L/R buttons). When the steering is centered, press the blue button on the NucleoF103 board (this is indicated in the ROS launch terminal on the Raspberry)
4. Compute steering coefficients (a and b) and write to flash memory

### 6. Communication checking
When a communication check request frame is received (from CAN bus), a response (ACK) is sent to the Raspberry indicating that the F103 board and the communication are working
