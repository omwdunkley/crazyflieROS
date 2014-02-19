__author__ = 'ollie'
__all__= ['generateRosMessages','ROSNode']
import logging

from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot

import rospy
import roslib
from roslib.packages import get_pkg_dir
roslib.load_manifest('crazyflieROS')

from crazyflieROS import msg as msgCF
from time import time

from commonTools import hasAllKeys, hasGroup, getGroup, getNames


logger = logging.getLogger(__name__)





class ROSNode(QObject):
    """ Important for two things:
        Spamming ROS with crazyflie logs,
        Receiving joystick data
    """
    def __init__(self):
        super(ROSNode, self).__init__()
        rospy.init_node('CrazyflieDriver')
        self.publishers = {}
        self.compiledMsgs = [m for m in dir(msgCF) if m[0]!="_"]


    def pub(self, group, msg):
        """ Generates publishers if needed """
        if group in self.compiledMsgs:
            if group in self.publishers:
                self.publishers[group].publish(msg)
            else:
                self.publishers[group] = rospy.Publisher("/cf/"+group, eval("msgCF."+group))
                self.publishers[group].publish(msg)
        else:
            rospy.logerr("Please generate messages: %s.msg does not exist", group)


    def genMsg(self, data, ts):
        """ generates a message using sexy python magic"""
        group = getGroup(data)
        msgt = eval("msgCF."+group)
        msg = msgt()
        msg.header.stamp = ts
        for name in getNames(data):
            setattr(msg, name, data[group+"."+name]) # sexy python magic
        return msg



    @pyqtSlot(object, int, object)
    def receiveCrazyflieLog(self, log, tsCF, tsROS):
        """ Handle sending messages to ROS """

        # SPECIALLY HANDLED MESSAGES
        if hasGroup(log, "baro"):
            # BAROMETER - send what we have
            if hasAllKeys(log, [], "baro"):
                m = msgCF.baro()
                m.header.stamp = tsROS
                m.temp = 555
            self.pub(getGroup(log), m)

        # DEFAULTS
        else:
            self.pub(getGroup(log), self.genMsg(log, tsROS))


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