CRAZYFLIE ROS DRIVER WITH GUI
============================

# Overview



-----------------------------
# Nodes


-----------------------------
# Features


Logging

* Edit
* HZ


Parameters

* Update
* Sync


Screenshots
![GUI Logging Tab](https://lh5.googleusercontent.com/-e9lVVdnRzhw/UwP5R2rK7TI/AAAAAAAAe58/eiTL13BXM9s/s800/gui.png)

# Connect joystick
To connect the ps3 joystick via bluetooth you will need to connect it via usb, then
```
sudo bash 
rosrun ps3joy sixpair
rosrun ps3joy ps3joy.py
```
disconnect your joystick and press the connect button. For more details see <http://wiki.ros.org/ps3joy>

This exposes your joystick under ```/dev/input/js*```.
Next you will need to start a ros node to read the joystick data and send it over the ros network:

```
ls /dev/input/js*
roslaunch crazyflieROS joy.launch js:=0 #
```
where you might need to replace 0 with your joystick nr.


```
rosrun dynamic_reconfigure reconfigure_gui
```
or
```
rosrun rqt_reconfigure rqt_reconfigure
```

-----------------------------
# Changing Code

Remember

* If you change the GUI, you need to recompile it. See ´´´crazyflieROS/src/crazyflieROS/ui/compileGUI.sh´´´ 
* Firmware changes need compiling and flashing
* If you change the TOC, you might need to regenerate the ros messages. See the Config tab


-----------------------------
# SETUP

Sounds: beep

* sudo apt-get install beep
* todo modprob

Firmware

* todo arm toolchain
* eclipse setup
* compiling
* flashing




