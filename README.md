CRAZYFLIE ROS DRIVER WITH GUI
============================

# Overview
This package / document was created to help some friends get working with the crazyflie and ROS. Still very much a work in progress


# Crazyflie Concepts

### Hardware
  * ~20g quadrotor
  * ~5g payload
  * 9 cm diagonal
  * 170 mah battery
    * 7 min flight
    * 3 with camera
    * degrades pretty quickly (100 cycles?)
  * 4 layer PCB design
  * Plastic motor mounts
    * The old ones are crap and break, be careful with the motor wires wearing through
  * 72 MHZ 32 bit MCU, 128kb flash, 20kb ram
  * 10 pin expansion header (I2C, UART, SPI/ADC, power, etc)
  

### Sensors
  * 3 axis gyro
    * drift around Z axis
    * 500hz on board polling
    * 100hz remote logging
  * 3 axis accelerometer
    * Quite noisy
    * 500hz on board polling
    * 100hz remote logging
  * 3 axis magnetometer
    * Need to calibrate it A LOT
    * No real work has been done here, Mike had some good results
    * same 500hz / 100 hz onboard/offboard logging
  * Barometer
    * Interweaved temp/pressure polling at combined 100 hz
    * With some hacks, 90hz pressure and 10hz temp onboard and offboard logging
    * Very noisy, but sensitive enough to notice +- 10ish cm differences
    * careful with air conditioning, opening windows/doors, storms
    * 

### Communication
  * 2.4GHZ via custom USB dongle
    * 127 channels = max 127 flies communicating at once
    * can be set at different kb/s (250Kb/s, 1Mbit/s, 2Mbit/s)
      * lower = more range but more collisions with wifi networks
      * I default to 2Mbit
  * Flies are addressed as URIs
    * For now its one flie per computer
      * limitation of the library, not the hardware
      * should be extendable without too much effort
    * eg ```radio://0/1/2M``` means Flie 0, on channel 1, at 2Mbit/s
  * randomly disconnects sometimes  






### Important Links


A quick list of useful links
* [Bitcraze Crazyflie Blog](http://www.bitcraze.se/) They just got GPS working....
* [Crazyflie WIKI](http://wiki.bitcraze.se/projects:crazyflie:index) Good index
* [Client Python API](http://wiki.bitcraze.se/projects:crazyflie:pc_utils:pylib) Most important resource (aside from reading code) if developing client side code
* [Firmware Stuff](http://wiki.bitcraze.se/projects:crazyflie:devenv:index) Important resource for compiling firmware
* [Intro](http://wiki.bitcraze.se/projects:crazyflie:userguide:index) Basic info, which side is the front, what leds mean
* [Assembly](http://wiki.bitcraze.se/projects:crazyflie:mechanics:assembly) Lots of pictures
* [Installing](http://wiki.bitcraze.se/projects:crazyflie:pc_utils:install)
* [Balancing, tipc, etc](http://wiki.bitcraze.se/projects:crazyflie:userguide:tips_and_tricks)
* [Troubleshooting](http://wiki.bitcraze.se/projects:crazyflie:userguide:troubleshooting)
* [Logging/param tutorial video](http://www.youtube.com/watch?v=chWrNh73YBw) Intro  video on the logging/param frame work. Not Seen this yet..
* [Crazyflie copter repo](https://bitbucket.org/bitcraze/crazyflie-pc-client)
* [Crazyflie firmware repo](https://bitbucket.org/bitcraze/crazyflie-firmware)
* [CrazyRADIO repo](https://bitbucket.org/bitcraze/crazyradio-firmware) not so important, can update the FW once in a while
* [Forum](http://forum.bitcraze.se/) Especially Crazyflie Developer Discussions section is nice :) 
* [Some pics of mine](https://picasaweb.google.com/106913106617973208533/Crazyflie?authuser=0&authkey=Gv1sRgCLuekNLn_siXzQE&feat=directlink)
* 




### Communication Protocol
The flie has its own custom communication protocol called the Crazy RealTime Protocol (CRTP). 
Has the notion of packets, ports, etc. Luckily a higher level python library implemented, so we
dont need to deal with the details


##### Parameters
Use this when you wish to set/get a variable on the crazyflie with low frequency. Eg turning an
led on/off, setting PID values, etc. Getting and setting is initiated from the computer.

TODO
  * Firmware code
  * Python code
  * ToC
  * GUI Application 


##### Logging
High throughput reading of variables on the crazyflie. 
One requests them, and the crazyflie sends them at a given frequency.
Useful for continuously reading sensor data
Maxes out at 100hz

TODO
  * Firmware code
  * Python code
  * ToC
  * GUI Application


##### Commander
Primitive but fast way to throw values at the crazyflie. Only used for sending it command data, eg roll, pitch yaw
Recommened to use at 100hz, more probably possible

TODO
  * Firmware code
  * Python code
  * GUI Application


# Installing on your sysm
### Get the code
### Arm toolkit
### Permissions

# Firmware
## Flashing, coding
## Some concepts

# Adding a camera
Examples
*  [First person video](http://youtu.be/AWSUMGJKt0U)
*  [Forum discussion](http://forum.bitcraze.se/viewtopic.php?f=6&t=491)
![Camera](https://lh6.googleusercontent.com/-hBCBGpAHvCE/UiWysHlkgzI/AAAAAAAAclU/cc-3S7ftYmI/s640/20130903_112305.jpg)

## Noise
![Before](https://lh3.googleusercontent.com/-_w6rI843KNI/UimIDFfRKjI/AAAAAAAAcEU/5aJqA4JwdHU/s640/frame0014.jpg)
![After](https://lh4.googleusercontent.com/-3_cujmAp8gE/UgNdGynVd-I/AAAAAAAAbcM/kmm1VCktbI8/s640/hud.png)
![Camera]()
## Hardware
## Viewing in VLC/ROS



# Client side GUI
## Default One
## Mine
## Using ROS
#### Setting up bluetooth PS3 Controller
Follow instructions here to install all the requirements: <http://wiki.ros.org/ps3joy>
For reference:

Connect joy stick with USB, then
```
sudo bash 
rosrun ps3joy sixpair
rosrun ps3joy ps3joy.py
```
disconnect your joystick and press the connect button. This exposes your joystick under ```/dev/input/js*```.
Next you will need to start a ros node to read the joystick data and send it over the ros network. Here Ive made a launch file that launches 3 nodes: one that exposes the controller to ROS; one that reads it and exposes crazyflie controls, and a gui to set some settings. Launch with

```
ls /dev/input/js*
roslaunch crazyflieROS joy.launch js:=0 #
```
where you might need to replace 0 with your joystick nr.


# Motion Capture System
[Video example](http://youtu.be/WPi__Q6CNdQ?t=2m4s)
## Setting it up
#### Defining ridig bodies
#### Required TF transforms
#### ROS node

---------------------------------


# Changing Code

Remember

* If you change the GUI, you need to recompile it. See ```crazyflieROS/src/crazyflieROS/ui/compileGUI.sh``` 
* Firmware changes need compiling and flashing
* If you change the TOC, you might need to regenerate the ros messages. See the Config tab


# SETUP

Sounds: beep

  * ```sudo apt-get install beep```
  * ```sudo modprobe pcspkr```






# Screenshots

![GUI Logging Tab](https://lh5.googleusercontent.com/-e9lVVdnRzhw/UwP5R2rK7TI/AAAAAAAAe58/eiTL13BXM9s/s800/gui.png)





