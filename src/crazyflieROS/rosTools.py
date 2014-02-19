__author__ = 'ollie'
__all__= ['generateRosMessages','ROSNode']




from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot

import rospy
import roslib
from roslib.packages import get_pkg_dir
roslib.load_manifest('crazyflieROS')
from crazyflieROS import msg as msgCF
from crazyflieROS.msg import cmd as cmdMSG

import tf


from math import radians
import logging

from commonTools import hasAllKeys, isGroup, getGroup, getNames


logger = logging.getLogger(__name__)



MIN_THRUST = 10000
MAX_THRUST_CMD = 60000.0 # as command to the flie
MAX_THRUST_FLIE  = 65535.0 # as value from the flie
def thrustToPercentage(thrust,flie=True):
    return (float(thrust-MIN_THRUST)/((MAX_THRUST_FLIE if flie else MAX_THRUST_CMD)-MIN_THRUST))*100.0


class ROSNode(QObject):
    """ Important for two things:
        Spamming ROS with crazyflie logs,
        Receiving joystick data
    """
    def __init__(self):
        super(ROSNode, self).__init__()
        rospy.init_node('CrazyflieDriver')
        self.compiledMsgs = [m for m in dir(msgCF) if m[0]!="_"] # Mmessages that are previously auto-compiled and we can send


        # Publishers
        self.publishers   = {} #Generated publishers will go here
        self.pub_tf       = tf.TransformBroadcaster()

        # Subscribers
        self.sub_tf    = tf.TransformListener()
        self.sub_joy   = rospy.Subscriber("/cfjoy", cmdMSG, self.receiveJoystick)


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


    def genMsg(self, data, ts,f=lambda x: x):
        """ generates a message using sexy python magic. Supply an optional function to apply to each member"""
        group = getGroup(data)
        msgt = eval("msgCF."+group)
        msg = msgt()
        msg.header.stamp = ts
        for name in getNames(data):
            setattr(msg, name, f(data[group+"."+name])) # sexy python magic
        return msg



    @pyqtSlot(object, int, object)
    def receiveCrazyflieLog(self, log, tsCF, tsROS):
        """ Handle sending messages to ROS """

        ## SPECIALLY HANDLED MESSAGES
        if isGroup(log, "gyro"):
            self.pub(getGroup(log), self.genMsg(log, tsROS, f=radians))
        elif isGroup(log, "motor", ):
            self.pub(getGroup(log), self.genMsg(log, tsROS), f=thrustToPercentage)
        ## AUTOMATICALLY GENERATED HERE
        else:
            self.pub(getGroup(log), self.genMsg(log, tsROS))

        ##ADITIONALLY HANDLED
        if isGroup(log, "stabilizer"):
            if hasAllKeys(log,["roll","pitch","yaw"]):
                self.pub_tf.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(
                    radians(log["stabilizer.roll"]),
                    -radians(log["stabilizer.pitch"]),
                    -radians(log["stabilizer.yaw"]),'sxyz'), tsROS, "/cf", "/world")



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