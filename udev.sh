#!/bin/bash
# Sets up the UDEV rules for using the crazyflie dongle without being root

# Run this before plugging in the dongle!!!

groupadd plugdev
usermod -a -G plugdev $USER
echo SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", \
MODE=\"0664\", GROUP=\"plugdev\" > /etc/udev/rules.d/99-crazyradio.rules

