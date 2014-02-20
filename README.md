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
Run ```sudo setup.sh```

# Firmware
### Download firmware
hg clone https://bitbucket.org/bitcraze/crazyflie-firmware


### install dev environment
See [Firmware Stuff](http://wiki.bitcraze.se/projects:crazyflie:devenv:index) Important resource for compiling firmware for details. In short, we will use [GNU Tools for ARM Embedded Processors](https://launchpad.net/gcc-arm-embedded)
  * ```sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded```
  * ```sudo apt-get update```
  * ```sudo apt-get install gcc-arm-none-eabi```
  * 
Then we can [Download Eclipse for C++](https://www.eclipse.org/downloads/packages/eclipse-ide-cc-developers/keplersr1)
Untar it, and run eclipse/eclipse
File | import | existing code as makefile project | next
Give it a name, chose the crazyflie-firmware directory
Unclick c++
Select GNU Autotools Toolchain

See ()[] for details

## Flashing, coding
## Some concepts

# Adding a camera
  * [First person video](http://youtu.be/AWSUMGJKt0U)
  * [Forum discussion](http://forum.bitcraze.se/viewtopic.php?f=6&t=491)

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
Qualisys
 * Log on, using the PW I gave you
 * Turn on the capture system (white power socket, so the red light is on)
 * Wait for them to boot up (observe IDs being displayed on the cameras)
 * Connect your laptop to the local network cable, possibly dissable WLAN
 * Start Qualisys Track Manager (icon on the desktop)
 * Open profile / create new project using "QualisysFlie" project template
 * Define bodies you wish to track
   * Record data for a few seconds
     * Reduce realtime freq = 100
     * stop on button only = on
     * save captured and proc measurement auto = off
     * -> Start
     * -> Stop a few seconds later
   * Click 3d, ctrl+click desired markers, right click | define ridgid body | give it a name
   * Define local coord system of new tracked object
     * Tools | Project Options | 6 DOF Tracking
     * Translate to point/mean, etc etc
     * Make sure the Euler Angle sub menu is set to: GLOBAL rotation, x,y,z = roll, pitch, yaw
   * New measurement: observe in 3d, repeat for other objects
 * Read Realtime data with ROS
   * ```rosrun Qualisys2Ros Qualisys2Ros```
   * For each n of N ridgid bodies defined, this spams out ros tf /Q0, ..., /QN
   * The order corresponds to the order that the ridig bodies were defined as (see Tools | Project Options | 6 DOF Tracking)
   * Now we neet to link the /Qn transforms to ones used by the PID controler
     * The flie tf is called /cf_gt, the wand is called /wand
     * run the following in three terminals, making sure to adjust Q0, Q1 to match wand/flie ridig bodies
       * ```rosrun tf static_transform_publisher 0 0 0 0 0 0 1 "/world" "/Qualisys" 10```
       * ```rosrun tf static_transform_publisher 0 0 0 0 0 0 1 "/Q0" "/cf_gt" 10```
       * ```rosrun tf static_transform_publisher 0 0 0 0 0 0 1 "/Q1" "/wand" 10```
     * Use RVIZ to visualise tfs: ```rosrun rviz rviz```
   * Start the PID controller
     * ```roslaunch crazyflieROS pid.launch js:=X``` Where x is the joystick you need. 
       * Use ```ls /dev/input/js*``` and ```jstest /dev/input/x/``` to determine which joystick you need
     * This launched 3 nodes, and configures them to respawn if closed. Also sets deadzone/caolesce intervals, etc
       * joy_node, which reads the filesystem for joystick input and outputs a joy msg
       * joy_pid_controller, read tfs and joy msg to either remote contorl flie or pid control flie
       * dynamic_reconfigure, a gui to change the parameters of the joy node
         * 
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





# Todo

Items
 * add udev script, then ```sudo python setup.py install```
 * file permissions (cfg, etc) - not working not thanks to NTFS
 * talk about ubuntu versions, tested on 12.04
 * add to ros path
 * add python package deps
   * ```sudo apt-get install python2.7 python-usb python-pygame python-qt4 python-guiqwt```
 * common tools
   * pycharm
   * eclipse
   * terminator
 * add rviz configuration with grids and TF

 
