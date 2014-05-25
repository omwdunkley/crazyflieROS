CRAZYFLIE ROS DRIVER WITH GUI
============================
**CrazyflieROS Node Details [below](#details).**

__Please note this document might be outdated. For discussions and questions on using the kinect part, please ask [here](http://forum.bitcraze.se/viewtopic.php?f=6&t=800).__

__Update:__ Branch joyManager was merged into master, no more mix-ups :)

__Update:__ Sixaxis/sixpair instructions updated to (workaround for a bug introduced by the later kernels)

# Introduction
This package / document was created to help some friends get working with the crazyflie, ROS and the mocap system here at TUM. Still very much a work in progress. 
The goal is to obtain a general overview of the flie, get a development environment set up for changing the firmware and implementing client side code.

I assume you are familiar with ROS and are running a recent version of Ubuntu. This was tested on 12.04 and ROS Fuerte to Hydro. You will need a joystick to fly the flie with - the code in this package was written to fly the flie with PS3 Sixaxis controller.

Also, some permissions might be messed up (thanks Mr NTFS partition..) so you might need to chmod +x a few files (the *.cfg files probably).




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
* [Crazyflie pc client repo](https://github.com/bitcraze/crazyflie-clients-python) *(Update to github)*
* [Crazyflie firmware repo](https://github.com/bitcraze/crazyflie-firmware) *(Update to github)*
* [CrazyRADIO repo](https://github.com/bitcraze/crazyradio-firmware) not so important, can update the FW once in a while *(Update to github)*
* [Forum](http://forum.bitcraze.se/) Especially Crazyflie Developer Discussions section is nice :) 
* [Some pics of mine](https://picasaweb.google.com/106913106617973208533/Crazyflie?authuser=0&authkey=Gv1sRgCLuekNLn_siXzQE&feat=directlink)
 

# Crazyflie Concepts

One has to distinguish between:
* Firmware
  - c code that runs on the flie
  - compiled on the computer and then flashed using the boot loader
* Client side code
  - code that runs on the computer and communicates with the flie
  - python is officially used, other wrappers exist



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




# First steps
Lets download all the code and prereqs we need. I probably missed a couple, so please let me know (either via githib or [the crazyflie forums](http://forum.bitcraze.se/viewtopic.php?f=6&t=800))

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
I recommend you use git to obtain the latest source code:
* Git: ```sudo apt-get install git-core```

You will need 3 sets of source code. Put them in your working ros directory.
* This code: ```git clone https://github.com/omwdunkley/crazyflieROS.git```
* Official client code: ```git clone https://github.com/bitcraze/crazyflie-clients-python```
* Custom firmware: ```git clone https://github.com/omwdunkley/crazyflie-firmware```


##### Permissions
To use the crazyradio you will need to set some udev rules. For conveniance, just run ```sudo sh udev.sh``` from the crazyflieROS directory.

Some of the files in this repo should be executable, but are not (thanks to my crappy NTFS permission). Especially some of the the files in crazyflieROS/cfg, crazyflieROS/bin and crazyflieROS/src/crazyflieROS/driver.py will need ```chmod +x path/file``` *(Update: most of this should be fixed now, let me know if it is not)*


##### Optional
To use the optional beeping functionality of my crazyflie driver you will need to enable the terminal bell: 
```sudo modprobe pcspkr```

I guess you can use any IDE, but I will use the following in this guide:
 * _python_ for client side code: [PyCharm](http://www.jetbrains.com/pycharm/download/download_thanks.jsp)
 * _c_ for the firmware [Eclipse for C++](https://www.eclipse.org/downloads/packages/eclipse-ide-cc-developers/keplersr1)


### Running stock Crazyflie client
The crazyflie client is a GUI fully in python that exposes all aspects of the flie. You can use it to flash the flie with new firmware, remote control it, observe sensor data, set parameters. Run ```bin/cfclient``` from the crazyflie-clients-python directory.

##### Flashing the flie with (potentially custom) firmware using the stock client.
Lets flash the flie with latest stock firmware. For now, instead of compiling it, we will download the latest bin directly from [here](https://github.com/bitcraze/crazyflie-firmware/releases). Extract it, we need the .bin file.

Unplug the flie if it is plugged in. Insert the dongle into a usb2 port (usb3 might work with the most recent radio firmware). Note at this point the flie does not need a battery or motors, just the PCB will suffice.

Run ```bin/cfclient``` from the crazyflie-clients-python directory.
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

This should open a new project with the crazyflie firmware. Press _ctrl+b_ to build it. This should result in a short summary of ram/rom usage and if all goes well produce a cflie.bin file you can flash the flie with using the procedure above.

### Client side code
Start up pycharm, _File | Open_, chose the crazyflieROS directory.

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
**Update:** [here](https://github.com/bitcraze/crazyflie-clients-python/blob/master/examples/basicparam.py) is an example.

_TODO: Examples of how to add logging to the firmware and read it from the client side code_

##### Logging
High throughput reading of variables on the crazyflie. 
One requests them, and the crazyflie sends them at a given frequency.
Useful for continuously reading sensor data
Maxes out at 100hz
What can be logged is specified in the firmware. Logs are then requested from the client and the flie starts pushing the data back 
**Update:** [here](https://github.com/bitcraze/crazyflie-clients-python/blob/master/examples/basiclog.py) is an example.

_TODO: Examples of how to add parameters to the firmware and set/read them from the client side code_

##### Commander
Primitive but fast way to throw values at the crazyflie. Only used for sending it command data, eg roll, pitch yaw
Recommened to use at 100hz. Send a command at least once every 2 seconds to keep the flie alive.

_TODO: Show the function that reads the variables in the firmware and the client side code that sends them_



## Using ROS
### Setting up bluetooth PS3 Controller using [PS3 Joy](http://wiki.ros.org/ps3joy)
__Update: One the kernel updates broke the default PS3 driver (sixpair). Please see [these instruction](https://help.ubuntu.com/community/Sixaxis#Quick_Setup_Guide_for_12.10) to fix it (tested with 12.04).

Once one has done the above, run
```
sixad --start 
```
This exposes your joystick under ```ls /dev/input/js*```.
To test it, you can use jstest-gtk: ```jstest-gtk``` or ```jstest /dev/input/js*```

Next you will need to start a ros node to read the joystick data and send it over the ros network. Here Ive made a launch file that launches 3 nodes: one that exposes the controller to ROS; one that reads it and exposes crazyflie controls, and a gui to set some settings. Launch with
```
roslaunch crazyflieROS joy.launch js:=0
```
where you will need to replace X with your joystick nr. Now you can visualise the outputs of the nodes:
* Test the joy_node: ```rostopic echo /joy```
* Test the crazyflie joy node: ```rostopic echo /cfjoy```




# <a name="details"></a> CrazyflieROS GUI Application
This is a ros node wrapped in QT gui. For now some of the functionality is still in separate python nodes (especially joy_manager_pid.py), but all should eventually fuse into this one.
### Feautres
* Full blown GUI
    * settings (table column positions, dimensions, settings, etc) are saved between sessions per radio (eg if you use multiple radios, each will have its own settings saved)
* Scans for all flies, connects to a specific one
    * Options:
        * Connect on startup
        * Auto-Reconnect on disconnect
* Nodes can run in parallel, each with their own radio. This allows them to share info (eg relative barometric differences)
* Parameters
    * Can be viewed in realtime
    * Can be changed in realtime. The GUI is very particular about only showing confirmed values from the flie
    * Grouped by read/read-write
* Logging
    * Shows which values are being logged in a collapsible tree structure 
    * Shows requested logging frequency for each group
    * shows actual logging freq for each group
    * can dynamically select groups/group subsets for logging
    * can dynamically set the requested logging rate
    * shows the logging type
* ROS
    * Ros support for fuerte-hydro
    * Automatically generates .msg files, independent of the firmware used. 
        * It inspects which log variables are available and generates the required messages. 
        * This way, if you add a log group in the firmware, you can generate the needed message files with a click of a button, allowing to stream all incoming log data over the ROS network. 
        * For example, a logging configuration called "accel" with a float 'accX' and 'accY' with compile into a ros message 'accel.msg' with accX and accY defined as floats.)
    * Stream all logging data over the ros network using the generated messages.
        * Logging data will be streamed on a topic with the format ```/cf<radio nr>/<log group name>```
    * TF transforms of the flies estimated attitude are sent out if the roll, pitch and yaw logging data is available
    * crazyflie debug output is sent to the ros loggers
* Sound feedback 
    * Optionally, the GUI makes distinct sounds for connecting, and disconnecting
* Info
    * Another tab shows firmware info, crazyradio info, which sensors were detected and the self test status.
* Realtime monitoring
    * Incoming Bytes per second
    * Out going Bytes per second 
    * CPU usage
    * Link Quality (slightly more pessimistic than the original client)
    * Battery (if being logged)
    * A tab shows the crazyflies debug prints 
* Attitude Indicator
    * Realtime drawing of the flies roll, pitch, yaw, barometer, accelerometer _(fuse in more stuff, as displayed in my old c++ hud)_]
    * Batches all drawing together at a user specified frequency (eg 30hz). This should avoid some crazy CPU usage people have seen in the original hud if displayed at more than 30hz.
    * Displays if thrust failsafe is on
    * Shows yaw a compass heading
* Manual yaw offset
    * Can "zero" the yaw (set current yaw to north)
    * Or specify an offset to north
* Many Settings
    * chose how fast the GUI updates different components
        * the Attitude indicator
        * logging frequency estimtaion
        * in/out throughput
* Kinect Tracking and 3 dof pose estimation
    * Track the flie in 3d. Together with the flies attitude estimate, full PID position & yaw control possible
    * Segments the flie from depth images
    * All options exposed to the GUI
    * With rviz, one can overlay the depth/rgb images with the flie pose estimation
* PID Controller _(todo: merge in from joy_driver_pid.py)_
* Monitoring Joystick input _(todo: merge in from joy_driver_pid.py)_



### Screenshots
Here are some screenshots. Note some are outdated as they do not show CPU usage.


Logging
![Logging](https://lh6.googleusercontent.com/-KlnZGNogx0w/U36TymWN0VI/AAAAAAAAgUg/8B_ZdjNL-mU/s800/log.png)

Params
![Params](https://lh6.googleusercontent.com/-cDCNwtCNDx8/U36TzDlD9SI/AAAAAAAAgUc/byiM5ZusS_0/s800/param.png)

Attitude Indicator
![Attitude Indicator](https://lh5.googleusercontent.com/-SbHDfzH23rI/U36Txl24kpI/AAAAAAAAgUA/gEk1Bd8qcd0/s800/ai.png)

Kinect
![Kinect](https://lh5.googleusercontent.com/-oZG887fCQDc/U36TxyGb-sI/AAAAAAAAgUI/o7sAa-mQvDI/s800/kinect.png)

Input
![input](https://lh5.googleusercontent.com/-qUxco-yN9JM/U36Tx5Fbl6I/AAAAAAAAgUE/5nCNKMOMiWY/s800/input.png)

Settings
![Settings](https://lh4.googleusercontent.com/-WPE0KY-Nde4/U36Tze_Y16I/AAAAAAAAgUU/TA_vhAyUCC4/s800/settings.png)






### Starting the node
Run is just like any ros node: ```rosrun crazyflieROS driver.py```

Show optional input arguments: ```rosrun crazyflieROS driver.py --help```




# Motion Capture System
##[Video example](http://youtu.be/WPi__Q6CNdQ?t=2m4s)

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
One can also use the kinect to track the 3d position of the flie. However, one must use the onboard attitude to estiamte the yaw. As yaw drifts and is not defined, one must manually align it to the camera optical axis. The gui has an option to "set north" in the current direction the flie is facing.

Install the freenect ROS stack using the instructions from [here](http://wiki.ros.org/freenect_stack) - this will install all the drivers, etc you need to use the kinect.

You will also need the following transformations
 * ```rosrun tf static_transform_publisher -1.5 0 1 0 0 0 /world /camera_link 10```
 * ```rosrun tf static_transform_publisher 0 0 0 0 0 0 /cf0 /cf_gt 10```


You will need to start the kinect driver
 * ```roslaunch freenect_launch freenect.launch```


You will also need to launch the PID controller (also handles joystick inputs)
 * ```roslaunch crazyflieROS pid.launch js:=0```

You can use RVIZ to verify everything:
 * ```rosrun rviz rviz -d kinect.rviz```
   * Sometimes rviz doesnt display the load modules properly. Just uncheck and recheck the box to spawn the view. 
   * Fixed frame should be "world".

If you are using the kinect colour camera, you might want to align it with the depth images:
 * run rosrun crazyflieROS reconfigure_gui
 * Select /camera/driver from the drop-down menu. Enable the depth_registration checkbox
 * Note the topic changes from ```/camera/depth/points``` to ```/camera/depth_registered/points```
 

Joy Settings:
 * control - on
 * set goal, then
  * live update - off
 * xy,yaw, thrust checked
 * pid passive
 * max thrust 100
  
GUI Settings
 * log | pm | at minimum roll, pitch and yaw checked. 
 * log | pm checked
 * log | pm | set hz to 100
 * input | disable thrust unchecked
 * input | disable hover mode checked
 * input | x-mode unchecked
 * settings | yaw offset checked
 * settings | yaw offset | align the crazyflie with the kinect (either crazyflie x (front) aligned with kinect z (optical axis), or crazyflie -y (right) aligned with kinect z. The press "set north". This is needed as the kienct cannot determine the yaw of the crazyflie, so we use the onboard attitude estimation for this. But it first needs to be aligned with the other frames.
 * 

##### Add some paper to the flie to make it have a larger cross section and improve tracking (highly recommended):
![paper](https://lh6.googleusercontent.com/-cJFQnaV9QZg/U3t2aEFmKFI/AAAAAAAAgTE/X-9PeT4RFkA/s800/IMG_20140520_173419.jpg)


# Adding a camera
##[First person video](http://youtu.be/AWSUMGJKt0U)
#####[Forum discussion](http://forum.bitcraze.se/viewtopic.php?f=6&t=491)

![Camera](https://lh6.googleusercontent.com/-hBCBGpAHvCE/UiWysHlkgzI/AAAAAAAAclU/cc-3S7ftYmI/s640/20130903_112305.jpg)

![vo](https://lh5.googleusercontent.com/-cSChyQK0pXY/U3oYp7Cz4KI/AAAAAAAAgKY/pGthwsnQ48Q/s800/depth_screenshot_28.03.2014.png)

### Hardware
##### Cam directly
###### VLC/ROS viewing
##### Cam with tx/rx
##### Cam with tx/rx powered from the flie


#### Noise
![Before](https://lh3.googleusercontent.com/-_w6rI843KNI/UimIDFfRKjI/AAAAAAAAcEU/5aJqA4JwdHU/s640/frame0014.jpg)

![After](https://lh4.googleusercontent.com/-3_cujmAp8gE/UgNdGynVd-I/AAAAAAAAbcM/kmm1VCktbI8/s640/hud.png)



---------------------------------


# Changing Code

Remember

* If you change the GUI, you need to recompile it. See ```crazyflieROS/src/crazyflieROS/ui/compileGUI.sh``` 
* Firmware changes need compiling and flashing
* If you change the TOC, you might need to regenerate and recompile the ros messages. See the Config tab




# Todo

 * JoyStick needs porting
   * Class
   * UI
   * Trim
   * Visualisation
   * Configuration options (slew, response curves, etc)
   * Add reading joy from PyGame
 * Tracker needs porting
   * Qualisys TF stuff
   * ~~Kinect~~
   * ~~Show depth image with overlay~~ (now on a ros topic)
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
   * ~~finish all the signals from logManager for the rest of the ui~~
   * add the offset of the gyro yaw ASAP, ie right at the source where the logs comes in
     * ROS needs it
     * The UI needs it
   * ~~Deal with yaw offset in Settings tab~~
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
   
   


 
