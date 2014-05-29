__author__ = 'ollie'
__all__= ['generateRosMessages','ROSNode']




from PyQt4.QtCore import Qt, QObject, pyqtSignal, pyqtSlot

import rospy
import roslib
import rosgraph

from roslib.packages import get_pkg_dir
roslib.load_manifest('crazyflieROS')
from crazyflieROS import msg as msgCF
from crazyflieROS.msg import cmd as cmdMSG

import tf


from math import radians
#import logging

from commonTools import hasAllKeys, isGroup, getGroup, getNames, thrustToPercentage





class ROSNode(QObject):
    """ Important for two things:
        Spamming ROS with crazyflie logs,
        Receiving joystick data
    """

    sig_joydata = pyqtSignal(float, float, float, float, bool) #RPYThrustHover
    sig_joydataRaw = pyqtSignal(float, float, float, int, bool) #RPYThrustHover
    sig_baro = pyqtSignal(float) # asl reading of another flie

    def __init__(self, options, parent=None):
        super(ROSNode, self).__init__(parent)

        self.compiledMsgs = [m for m in dir(msgCF) if m[0]!="_" and m[-2:]=="CF"] # Messages that are previously auto-compiled and we can send
        if len(self.compiledMsgs)==0:
            rospy.logerror('Not Messages could be loaded. Please connect to the flie and press Compile Messages, then run rosmake')
        self.options = options
        # Publishers
        self.publishers   = {} #Generated publishers will go here
        self.pub_tf       = tf.TransformBroadcaster()

        # Subscribers
        self.sub_tf    = tf.TransformListener()
        self.sub_joy   = rospy.Subscriber("/cfjoy", cmdMSG, self.receiveJoystick)
        self.sub_baro = None # Defined later

        self.master = rosgraph.Master('/rostopic')


    @pyqtSlot(str)
    def subBaro(self, topic):
        if topic == '' and self.sub_baro is not None:
            rospy.loginfo('Unsubscribing from %s')
            self.sub_baro.unregister()
            self.sub_baro = None
            self.sig_baro.emit(0)
        else:
            try:
                from crazyflieROS.msg import baroCF as baroMSG
            except ImportError as exc:
                rospy.logerr('Cound not import baroCF. Generate messages (settings tab), then rosmake. Details: '+"{}".format(exc))
                return
            rospy.loginfo('Subscribing to %s for relative barometer measurements...', topic)
            self.sub_baro = rospy.Subscriber(str(topic), baroMSG, self.newBarodata)
            rospy.loginfo('...Subscribied')


    def newBarodata(self, b):
        self.sig_baro.emit(b.asl)


    def getTopics(self):
        """ Return a list of published topics """
        #try:
        state = self.master.getSystemState()
        pubs, subs, _ = state
        topics = [t for t, _ in pubs]
        #except socket.error:
        #    raise ROSTopicIOException("Unable to communicate with master!")
        return topics

    def pub(self, group, msg):
        """ Generates publishers if needed """
        if group+'CF' in self.compiledMsgs:
            if group in self.publishers:
                self.publishers[group].publish(msg)
            else:
                rospy.loginfo('Generating Publisher for topic </cf%d/%s>',self.options.radio, group)
                self.publishers[group] = rospy.Publisher("/cf%d/%s" % (self.options.radio, group), eval('msgCF.'+group+'CF'), queue_size=25)
                self.publishers[group].publish(msg)
        else:
            rospy.logerr("Please generate messages: %s.msg does not exist", group)


    def genMsg(self, data, ts,f=lambda x: x):
        """ generates a message using sexy python magic. Supply an optional function to apply to each member"""
        group = getGroup(data)
        msgt = eval("msgCF."+group+"CF")
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
            self.pub(getGroup(log), self.genMsg(log, tsROS, f=thrustToPercentage))
        ## AUTOMATICALLY GENERATED HERE
        else:
            self.pub(getGroup(log), self.genMsg(log, tsROS))

        ##ADITIONALLY HANDLED ROS STUFF
        # spam TF #TODO test this!!
        if isGroup(log, "stabilizer"):
            if hasAllKeys(log,["roll","pitch","yaw"], "stabilizer"):
                self.pub_tf.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(
                    radians(log["stabilizer.roll"]),
                    -radians(log["stabilizer.pitch"]),
                    -radians(log["stabilizer.yaw"]),'sxyz'), tsROS, "/cf%d" % self.options.radio, "/cf_xyz")



    def receiveJoystick(self, cmd):
        """ Incoming Joystick Message """
        # Emits callback; probably connected to flie and GUI
        # TODO: have the joydriver.py class integrated in here
        # TODO: send param set request for hover/on off
        self.sig_joydata.emit(cmd.roll, cmd.pitch, cmd.yaw, thrustToPercentage(cmd.thrust,flie=False), cmd.hover)
        self.sig_joydataRaw.emit(cmd.roll, cmd.pitch, cmd.yaw, cmd.thrust, cmd.hover)


def generateRosMessages(toc):
    """ Generates the *.msg files for ROS from the TOC
    """
    rospy.loginfo('Generating Messages')
    if not toc:
        rospy.logwarn("No TOC available to generate ROS messages from")
        return

    rospy.loginfo('Generating Messages...')
    path = get_pkg_dir('crazyflieROS')+"/msg/"
    for g in toc.keys():
        makeMsg(g, path, toc[g] )
    rospy.loginfo('Done')


def makeMsg(name, path, members):
    f = open(path+name+"CF.msg", 'w')
    rospy.logino(' -> '+path+name+"CF.msg")
    f.write("#DO NOT EDIT THIS FILE - IT IS AUTO GENERATED. ALL CONTENTS WILL BE REPLACED\n")
    f.write("Header header\n")
    for m in sorted(members.keys()):
        # type, remove the _t or append 32
        t = members[m].ctype[:-2] if members[m].ctype!="float" else members[m].ctype+"32"
        file.write("%s %s\n" % (t, members[m].name))
    f.close()