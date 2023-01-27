# GeiCar Project

The GeiCar project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous car in order to carry out different missions. Several projects are exposed on the [official website] (https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiCar. The present code as well as the documentation is the result of internship carried out by [Alexis24](https://github.com/Alexix24) (Alexis Pierre Dit Lambert)

The platform is developped and maintained by :

* DI MERCURIO Sébastien
* LOMBARD Emmanuel
* MARTIN José

The projects are (or were) surpervised by:

* CHANTHERY Elodie
* DELAUTIER Sébastien
* MONTEIL Thierry
* LE BOTLAN Didier
* AURIOL Guillaume
* DI MERCURIO Sébastien

## Quick User Guide
###Turn the car on and off
* To turn on the car:
  * Toggle the red button to bring the power.
  * Press the START push button (hold it down for a short while).
  * Wait until Raspberry boot up and connect to it using its IP address (written on the board): `ssh pi@10.105.1.xx`
  * When connected start ROS subsystem using :`ros2 launch geicar_start geicar.launch.py`
  * Then, you will get a report of subsystems and be able to control the car using XBOX controler 

* To turn off the car:
	* Use the red button as a switch to turn off the power.

###Use of this repository
* First of all, [fork this repository](https://docs.github.com/en/get-started/quickstart/fork-a-repo) to get yours.
* Then, clone your fresh and new repository on your computer: `git clone https://github.com/<your id>/<your wonderful repo>`
* Have a look in "general" directory for how to connect and work with your car
* For more 'in depth' documentation on a particular subsystem, have a look in following directories:
    * raspberryPI3: Everything about raspberry setup
    * nucleoL476: Stuff about GPS-RTK and IMU
    * nucleoF103: informations on motors control, main power and ultrasonics sensors
    * jetsonNano: Directory containing info on IA, Camera and Lidar
    * simulation: if you want to setup a carla simulation environment

__warning__
You normally do not need to change firmware running in F103 and F476 boards. You main work is on the raspberry and jetson side.

# Tow-Mater instructions

[Website](https://sites.google.com/view/insa-5siec/projets-2022-2023/project-benny)

The main branch corresponds to the code of the main vehicle (the towing vehicle). The integration-dc branch corresponds to the towed vehicle. It is not necessary to compile/execute the code on this car.

After compiling the whole project, you can launch the ros nodes on the jetson and on the raspberry:
```sh
ros2 launch geicar_start_jetson geicar.jetson.launch.py
```
```sh
ros2 launch geicar_start geicar.launch.py
```

When the system is started, you can use the joystick :
* Press "A" to switch to autonomous mode
* Press "Y" to switch to manual mode
* Press "Start" to start
* Press "B" to stop

In autonomous mode, you can choose a particular maneuver (No U-turn, u-turn, reverse ...). To do this, you must hold down the LB button and select the maneuver with DPAD-RIGHT. Release LB to confirm the choice.

You must place the two vehicles parallel to each other, with a space of 50 cm between them. The towing vehicle must be on the left side of the towed vehicle (see [Video](https://youtu.be/MB7T-dSHWLg)). 
