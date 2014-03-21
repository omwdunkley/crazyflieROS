#!/usr/bin/env python
#import roslib,
#import logging
#roslib.load_manifest("crazyflieROS")

#logger = logging.getLogger(__name__)
#logging.basicConfig(level=logging.INFO)
import argparse

import sys
from driverWindow import DriverWindow
from PyQt4 import QtGui


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)

    parser = argparse.ArgumentParser(prog="CrazyflieDriver")
    parser.add_argument("-r", "--radio",
                        action="store",
                        dest="radio",
                        type=int,
                        default=0,
                        help="Radio to use if multiple dongles are plugged into the computer. -1 = only allow one radio (named ROS node)")

    parser.add_argument("-d", "--debug",
                        action="store_true",
                        dest="debug",
                        help="Enable debug output")
    parser.add_argument("-R", "--reset",
                        action="store_true",
                        dest="reset",
                        help="Reset saved session settings")

    (options, unused) = parser.parse_known_args()



    window = DriverWindow(options)
    sys.exit(app.exec_())
