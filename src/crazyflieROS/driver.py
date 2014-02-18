#!/usr/bin/env python
#import roslib,
import logging
#roslib.load_manifest("crazyflieROS")

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


import sys
from driverWindow import DriverWindow
from PyQt4 import QtGui


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = DriverWindow()
    sys.exit(app.exec_())
