CRAZYFLIE ROS DRIVER WITH GUI
============================

# Overview
This package / document was created to help some friends get working with the crazyflie, ROS and the mocap system here at TUM. Still very much a work in progress.
The goal is to obtain a general overview of the flie, get a development environment set up for changing the firmware and implementing client side code.

I assume you are familiar with ROS and are running a recent version of Ubuntu. This was tested on 12.04 and ROS Fuerte to Hydro. You will need a joystick to fly the flie with - the code in this package was written to fly the flie with PS3 Sixaxis controller.




# Crazyflie Concepts

One has to distinguish between:
* Firmware
  - c code that runs on the flie
  - compiled on the computer and then flashed using the boot loader
* Client side code
  - code that runs on the computer and communicates with the flie
  - python is officially used, other wrapped exist



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
 


# First steps
Before we start to develop for the flie, it makes to test it with the officially supported stock code. First we download all the requirements, then test the flie with stock code, then run our custom ROS stuff.

### Requirements and source code
##### Required packages
You will need the following, os run:
```sudo apt-get install python2.7 python-usb python-pygame python-qt4 beep terminator ipython ros-$ROS_DISTRO-joystick-drivers```

To compile the firmware you will need [GNU Tools for ARM Embedded Processors](https://launchpad.net/gcc-arm-embedded)
```
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi
```

##### Source Code
I recommend you use git and mercurial to obtain the latest source code:
* Git: ```sudo apt-get install git-core```
* Mercurial (hg): ```sudo apt-get install mercurial meld```

You will need 3 sets of source code
* This code: ```git clone https://github.com/omwdunkley/crazyflieROS.git```
* Official client code: ```hg clone https://bitbucket.org/bitcraze/crazyflie-pc-client```
* Official firmware: ```hg clone https://bitbucket.org/bitcraze/crazyflie-firmware```


##### Permissions
To use the crazyradio you will need to set some udev rules. For conveniance, just run ```sudo sh udev.sh``` from the crazyflieROS directory.

Some of the files in this repo should be executable, but are not (thanks to my crappy NTFS permission). Especially some of the the files in crazyflieROS/cfg, crazyflieROS/bin and crazyflieROS/src/crazyflieROS/driver.py will need ```chmod +x path/file```


##### Optional
To use the optional beeping functionality of my crazyflie driver you will need to enable the terminal bell: ```sudo modprobe pcspkr```

I guess you can use any IDE, but I will use the following in this guide:
 * _python_ for client side code: [PyCharm](http://www.jetbrains.com/pycharm/download/download_thanks.jsp)
 * _c_ for the firmware [Eclipse for C++](https://www.eclipse.org/downloads/packages/eclipse-ide-cc-developers/keplersr1)


### Running stock code Crazyflie client
The crazyflie client is a GUI fully in python that exposes all aspects of the flie. You can use it to flash the flie with new firmware, remote control it, observe sensor data, set parameters. 

##### Flashing the flie with (potentially custom) firmware
First we will use it to flash the flie with latest stock firmware. For now, instead of compiling it, we will download the bin directly
```wget https://bitbucket.org/bitcraze/crazyflie-firmware/downloads/Crazyflie_2014.01.0.bin```

Now plug in the radio dongle, unplug the flie (at this point it doesnt need a battery or motors, etc), and run
```python crazyflie-client/bin/cfclient```
Then follow the Bootloader [instructions](http://wiki.bitcraze.se/projects:crazyflie:pc_utils:qt_ui) and use the bin file you just downloaded.
Using the instructions and links on that page you should now be able to connect and fly the flie.



# Developing for the flie

### Compile / modify firmware
Start up eclipse and
* _File | import | existing code as makefile project | next_. 
* Give it a name
* chose the crazyflie-firmware directory
* Unclick c++
* Select GNU Autotools Toolchain
* Click okay

This should open a new project with the crazyflie firmware. Press _ctrl+b_ to build it

### Client side code
Start up pycharm, _File | Open_, chose the crazyflie-client directory. Repeat for the crazyflieROS directory (and open in new window)

_TODO_
* _set up run configuration in pycharm_
* _experiment around with ipython_



## Communication Protocol
Some important concepts based on how the communication with the flie works are important befor you can start anything. 
The flie has its own custom communication protocol called the Crazy RealTime Protocol (CRTP). 
Has the notion of packets, ports, etc. Luckily a higher level python library implemented, so we
dont need to deal with the details
Details can be found [Here](http://wiki.bitcraze.se/projects:crazyflie:pc_utils:pylib)



##### Parameters
Use this when you wish to set/get a variable on the crazyflie with low frequency. Eg turning an
led on/off, setting PID values, etc. Getting and setting is initiated from the computer.

_TODO: Examples of how to add logging to the firmware and read it from the client side code_

##### Logging
High throughput reading of variables on the crazyflie. 
One requests them, and the crazyflie sends them at a given frequency.
Useful for continuously reading sensor data
Maxes out at 100hz
What can be logged is specified in the firmware. Logs are then requested from the client and the flie starts pushing the data back 

_TODO: Examples of how to add parameters to the firmware and set/read them from the client side code_

##### Commander
Primitive but fast way to throw values at the crazyflie. Only used for sending it command data, eg roll, pitch yaw
Recommened to use at 100hz, more

_TODO: Show the function that reads the variables in the firmware and the client side code that sends them_



## Using ROS
### Setting up bluetooth PS3 Controller using [PS3 Joy](http://wiki.ros.org/ps3joy)
To connect the PS3 controller with bluetooth, first plug it in with USB, then
```
sudo bash 
rosrun ps3joy sixpair
rosrun ps3joy ps3joy.py
```
disconnect your joystick and press the connect button. This exposes your joystick under ```/dev/input/js*```.
To test it, you can use jstest-gtk: ```jstest-gtk```

Next you will need to start a ros node to read the joystick data and send it over the ros network. Here Ive made a launch file that launches 3 nodes: one that exposes the controller to ROS; one that reads it and exposes crazyflie controls, and a gui to set some settings. Launch with
```
roslaunch crazyflieROS joy.launch js:=0
```
where you will need to replace X with your joystick nr. Now you can visualise the outputs of the nodes:
* Test the joy_node: ```rostopic echo /joy```
* Test the crazyflie joy node: ```rostopic echo /cfjoy```

### CrazyflieROS GUI Application
#### Running
```rosrun crazyflieROS driver.py```
#### Connecting
#### Checking inputs
#### Settings
#### Log
#### Param


# Motion Capture System
[Video example](http://youtu.be/WPi__Q6CNdQ?t=2m4s)

_TODO: Overview, way poin control, wand control, pid.launch_
### Setting it up Qualisys
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
     * X points forwards, Y left, Z up
     * Tools | Project Options | 6 DOF Tracking
     * Translate to point/mean, etc etc
     * Make sure the Euler Angle sub menu is set to: GLOBAL rotation, x,y,z = roll, pitch, yaw
   * New measurement: observe in 3d, repeat for wand
   
   
### Read Realtime data with ROS
##### Qualisys ROS Node
   * ```rosrun Qualisys2Ros Qualisys2Ros```
   * For each n of N ridgid bodies defined, this spams out ros tf /Q0, ..., /QN
   * Use RVIZ to visualise tfs: ```rosrun rviz rviz```
     * _TODO: default configuration file_
#### Defining ridig bodies   
   * The order corresponds to the order that the ridig bodies were defined as (see Tools | Project Options | 6 DOF Tracking)
   * Now we neet to link the /Qn transforms to ones used by the PID controler
     * The flie tf is called /cf_gt, the wand is called /wand
     * The /wand transform defines the goal position of the flie. Obviously we do not want this to be in side the defined rigid body, so we move it forward in x direction. Notice the 0.7 in the static transfrom publisher below
     * run the following in three terminals, making sure to adjust Q0, Q1 to match wand/flie ridig bodies
       * ```rosrun tf static_transform_publisher 0 0 0 0 0 0 1 "/world" "/Qualisys" 10```
       * ```rosrun tf static_transform_publisher 0 0 0 0 0 0 1 "/Q0" "/cf_gt" 10```
       * ```rosrun tf static_transform_publisher 0.7 0 0 0 0 0 1 "/Q1" "/wand" 10```


        

### PID Node
 * Start the PID controller
   * ```roslaunch crazyflieROS pid.launch js:=X``` Where x is the joystick you need. 
     * Use ```ls /dev/input/js*``` and ```jstest /dev/input/x/``` to determine which joystick you need
   * This launched 3 nodes, and configures them to respawn if closed. Also sets deadzone/caolesce intervals, etc
     * joy_node, which reads the filesystem for joystick input and outputs a joy msg
     * joy_pid_controller, read tfs and joy msg to either remote contorl flie or pid control flie
     * dynamic_reconfigure, a gui to change the parameters of the joy node

##### Controls
##### Options


#### Required TF transforms


# Kinect Tracking and Control
One can also use the kinect to track the 3d position of the flie. However, one must use the onboard attitude to estiamte the roll/pitch/yaw. As yaw drifts and is not defined, one must manually align it to the camera optical axis. The gui has an option to "set north" in the current direction the flie is facing.

```rosrun tf static_transform_publisher -1.5 0 1 0 0 0 /world /camera_link 10```
```rosrun tf static_transform_publisher 0 0 0 0 0 0 /cf /cf_gt 10```
```roslaunch freenect_launch freenect.launch```

Todo:
 * Flie Yaw Offset




# Adding a camera
  * [First person video](http://youtu.be/AWSUMGJKt0U)
  * [Forum discussion](http://forum.bitcraze.se/viewtopic.php?f=6&t=491)

![Camera](https://lh6.googleusercontent.com/-hBCBGpAHvCE/UiWysHlkgzI/AAAAAAAAclU/cc-3S7ftYmI/s640/20130903_112305.jpg)

### Hardware
##### Cam directly
###### VLC/ROS viewing
##### Cam with tx/rx
##### Cam with tx/rx powered from the flie


#### Noise
![Before](https://lh3.googleusercontent.com/-_w6rI843KNI/UimIDFfRKjI/AAAAAAAAcEU/5aJqA4JwdHU/s640/frame0014.jpg)
![After](https://lh4.googleusercontent.com/-3_cujmAp8gE/UgNdGynVd-I/AAAAAAAAbcM/kmm1VCktbI8/s640/hud.png)
![Camera]()








# Screenshots

![GUI Logging Tab](https://lh5.googleusercontent.com/-e9lVVdnRzhw/UwP5R2rK7TI/AAAAAAAAe58/eiTL13BXM9s/s800/gui.png)



---------------------------------


# Changing Code

Remember

* If you change the GUI, you need to recompile it. See ```crazyflieROS/src/crazyflieROS/ui/compileGUI.sh``` 
* Firmware changes need compiling and flashing
* If you change the TOC, you might need to regenerate the ros messages. See the Config tab




# Todo

TODO
 * JoyStick needs porting
   * Class
   * UI
   * Trim
   * Visualisation
   * Configuration options (slew, response curves, etc)
   * Add reading joy from PyGame
 * Tracker needs porting
   * Qualisys TF stuff
   * Kinect
   * Show depth image with overlay
 * PID needs porting
   * subclass QTableWidget
   * save/load configs
   * generate response curves
   * realtime graphing
   * error column in table - add colour info
   * checkboxes in table, 
   * read/write between sessions, etc
 * General Stuff
   * image topic viewer (for cam, for kinect)
   * label, spinbox, horizontal viewer combo
   * finish all the signals from logManager for the rest of the ui
   * add the offset of the gyro yaw ASAP, ie right at the source where the logs comes in
     * ROS needs it
     * The UI needs it
   * Deal with yaw offset in Settings tab
   * Deal with reading/writing the Settings tab values
   * Add ROS startup check, ie a dialog which says "WAiting ... Cancel"
   * Add Icon, change window name
   * Some things should be undockable!
   * ctrl+c from the terminal should kill it
   * parse command line arguments
     * argument to reset the default QSettings
   * If multiple drones discovered, dont autoconnect the first time
 * AI
   * needs baro, accel, motors, crash, etc connected to it
   * Add a slider and spin box in the config to set the update hz and turn it on/off
   * Show overlay from ROS images
   * right click functionality
   * Inspiration from c++ code
   * double click full screen
   
   
FIXME

```
AttributeError: 'FlieControl' object has no attribute 'param'
Traceback (most recent call last):
  File "/home/ollie/Dropbox/Code/ROS/crazyflieROS/src/crazyflieROS/FlieManager.py", line 179, in sendCmd
    self.requestHover(hover)
  File "/home/ollie/Dropbox/Code/ROS/crazyflieROS/src/crazyflieROS/FlieManager.py", line 190, in requestHover
  ```


 
