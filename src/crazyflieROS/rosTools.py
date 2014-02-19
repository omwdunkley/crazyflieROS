__author__ = 'ollie'
__all__= ['generateRosMessages','ROSNode']
import logging

from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot

import rospy
import roslib
from roslib.packages import get_pkg_dir
roslib.load_manifest('crazyflieROS')

from crazyflieROS import msg
from time import time


logger = logging.getLogger(__name__)


class ROSNode(QObject):
    """ Important for two things:
        Spamming ROS with crazyflie logs,
        Receiving joystick data
    """
    def __init__(self):
        super(ROSNode, self).__init__()
        rospy.init_node('CrazyflieDriver')


    @pyqtSlot(object, int, object)
    def receiveCrazyflieLog(self, log, tsCF, tsROS):
        """ Handle sending messages to ROS """

        # GENERATED MESSAGES
        print log.keys()[0]




        # SPECIAL CASES


    def receiveJoystick(self, joy):
        print joy








def generateRosMessages(toc):
    """ Generates the *.msg files for ROS from the TOC
    """
    if not toc:
        logger.warn("No TOC available to generate ROS messages from")
        return

    path = get_pkg_dir('crazyflieROS')+"/msg/"
    for g in toc.keys():
        makeMsg(g, path, toc[g] )


def makeMsg(name, path, members):
    file = open(path+name+".msg", 'w')
    file.write("Header header\n")
    for m in sorted(members.keys()):
        # type, remove the _t or append 32
        t = members[m].ctype[:-2] if members[m].ctype!="float" else members[m].ctype+"32"
        file.write("%s %s\n" % (t, members[m].name))
    file.close()