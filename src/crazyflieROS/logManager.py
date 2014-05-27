__author__ = 'OMWDunkley'
__all__ = ['LogManager']


import bisect

from collections import OrderedDict
from math import ceil, floor
#import logging
#logger = logging.getLogger(__name__)

import rospy

from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot,  QVariant, QTimer, QSettings, QObject
from PyQt4.QtGui import  QTreeWidget, QTreeWidgetItem, QAbstractItemView,QHeaderView,QCheckBox
from cflib.crazyflie.log import Log, LogConfig, LogVariable

from commonTools import FreqMonitor, hasAllKeys, isGroup, thrustToPercentage
import math



class LogGroup(QTreeWidgetItem):
    """ Group
        If turned on and no children are active, activate all children
        if hz changed, create new log (and start if on)


        Everytime the children change, we stop, delete the old log and create a new one, optionally starting it
        Everytime the HZ changes, we stop, delete the old log and create a new one, optionally starting it

        Everytime we change, we start/stop the log
            We request a start, use partially checked
            Once started, use totally checked


    """

    def __init__(self, parent, name, children, hz=50):
        super(LogGroup, self).__init__(parent)
        self.name = name
        self.setFlags(self.flags() | Qt.ItemIsEditable | Qt.ItemIsUserCheckable)

        # Keep track of if all/none of the children are active
        self.cAll = True
        self.cNone = True
        self.validHZ = list(OrderedDict.fromkeys([round(100/floor(100/x),2) for x in range(1,100)]))

        # Monitor the incoming frequency
        self.fm = None
        self.fmOn = True

        # log config
        self.lg = None

        # Show text
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.DisplayRole, QVariant(name))
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Unchecked)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Off")
        QtGui.QTreeWidgetItem.setData(self, 3, Qt.DisplayRole, QVariant(0))
        #QtGui.QTreeWidgetItem.setData(self, 4, Qt.CheckStateRole, Qt.Unchecked)
        self.readSettings()

        # Initialise Children
        for c in sorted(children.keys()):
            self.addChild(LogItem(self, children[c]))

        self.c = 0


        # Now everything is initialised except the logging
        # Start logging if active
        if self.checkState(0) == Qt.Checked:
            self.requestLog()





    def setEstimateHzOn(self, on):
        """ If on then we esstiamte the HZ """
        self.fmOn = on


    def getValidHZ(self, hz):
        # zip(list(OrderedDict.fromkeys([round(100/floor(100/x),2) for x in range(1,100)])),list(OrderedDict.fromkeys([1000/round(100/floor(100/x),2) for x in range(1,100)])))

        i = bisect.bisect_left(self.validHZ, hz)
        vHZ =  min(self.validHZ[max(0, i-1): i+2], key=lambda t: abs(hz - t))
        if hz!=vHZ:
            rospy.loginfo("Could not set HZ to specific value [%d], rounded to valid HZ [%d]", hz, vHZ)
        return vHZ


    def readSettings(self):
        """ Read the HZ and selected things to log from previous session """
        settings = QSettings("omwdunkley", "flieROS")

        # Read target HZ
        hzQv = settings.value("log_"+self.name+"_hz",QVariant(20))
        QtGui.QTreeWidgetItem.setData(self, 2, Qt.DisplayRole, hzQv)

        # Read if checked. If partially checked, uncheck
        onQv = settings.value("log_"+self.name+"_on", QVariant(Qt.Unchecked))
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, onQv if onQv.toInt()[0]!=Qt.PartiallyChecked else Qt.Unchecked)

        #onRos = settings.value("ros_"+self.name+"_on", QVariant(Qt.Unchecked))
        #QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, onRos)


    def writeSettings(self):
        """ Save the HZ and selected things to log between sessions"""
        settings = QSettings("omwdunkley", "flieROS")

        # Save target HZ
        settings.setValue("log_"+self.name+"_hz", self.data(2, Qt.DisplayRole))

        # Save checked state
        settings.setValue("log_"+self.name+"_on", self.data(0, Qt.CheckStateRole))
        #settings.setValue("ros_"+self.name+"_on", self.data(4, Qt.CheckStateRole))

        # Save children state too

        for x in range(self.childCount()):
            self.child(x).writeSettings()


    def updateFM(self):
        # if not self.isActive()
        if self.fm:
            QtGui.QTreeWidgetItem.setData(self, 3, Qt.DisplayRole, round(self.fm.get_hz(),2))


    def logDataCB(self, ts, data, lg):
        """ Our data from the log """
        #print "LogCB data:", data.keys()
        # Update out HZ monitor
        if self.fmOn:
            self.fm.count()

        #TODO: add a checkbox in the row if we wanan send or not
        self.treeWidget().incomingData(data, ts)


    #TODO: there is bug here where the callbacks are called twice!! Holds for this + deleted CB
    def logStartedCB(self, on):
        """ Called when we actually start logging """
        self.c+=1
        #print "counter[%s]"%self.name, self.c
        #if self.c % 2 == 1:
         #   return


        if on:
            rospy.loginfo( "[%d: %s] LogCB STARTED" % (self.lg.id, self.name))
            QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Checked)
            QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "On")
            self.setAllState(Qt.Checked, activeOnly=True)
        else:
            rospy.loginfo( "[%d] LogCB STOPPED" % (self.name))
            pass
            #print "[%s]LogCB ENDED(%d) " % (self.name, self.c)
            #if self.allSelected():
            #    self.setAllState(Qt.Unchecked)
            ##else:
            ##    self.setAllState(Qt.Checked, activeOnly=True)
            #QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Unchecked)
            #QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Off")

    # def logAddedCB(self, l):
    #     """ Called when log added """
    #     print "LogCB added"
    #     QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Checked)
    #     QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Added")


    def requestLog(self):
        """ user requested start """
        #print "Requested log start"

        #print "None selected:",self.noneSelected()
        self.setAllState(Qt.PartiallyChecked, activeOnly=not self.noneSelected())
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.PartiallyChecked)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Requested")

        # Make and start log config
        self.makeLog()


    def stopLog(self):
        """ User request halt """
        #print "Requested log stop"
        if self.lg:
            self.lg.delete() #Invokes logStarted(False)
            self.lg = None
        if self.allSelected():
            self.setAllState(Qt.Unchecked)
        #else:
        #    self.setAllState(Qt.Checked, activeOnly=True)
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Unchecked)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Off")


    def errorLog(self, block, msg):
        """ When a log error occurs """
        #print "Log error: %s:", msg
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Err: %s"%msg)
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Unchecked)
        if self.allSelected():
            self.setAllState(Qt.Unchecked)
        else:
            self.setAllState(Qt.Checked, activeOnly=True)


    def setData(self, column, role, value):
        """ Only allow changing of the check state and the hz """

        # GROUP ON/OFF
        preValue = self.checkState(0)
        if role == Qt.CheckStateRole and column == 0 and preValue != value:
            # Check box changed, allow and trigger

            #print "\n\nCONFIG CHANGED\n"
            if preValue==Qt.PartiallyChecked:
                #print "User clicked: Waiting for log callback, cannot change config now"
                pass

            elif value==Qt.Checked:
                #print "User clicked: request logging"
                self.requestLog()

            elif value==Qt.Unchecked:
                #print "User clicked: request delete logging"
                self.stopLog()


        # HZ CHANGED
        if column == 2:
            #hz = min(max(value.toInt()[0],1),100)
            hz = self.getValidHZ(value.toFloat()[0])

            preHz = self.data(column, Qt.DisplayRole).toFloat()[0]
            #print "\n\nHZ CHANGED\n"
            if hz!=preHz:
                if preValue==Qt.PartiallyChecked:
                    pass
                    #print "Waiting for log callback, cannot change hz now"
                elif preValue==Qt.Checked:
                    #print "hz changed from %.2f to %.2f while logging, replace log" % (preHz, hz)
                    QtGui.QTreeWidgetItem.setData(self, column, role, hz)
                    self.requestLog()
                elif preValue==Qt.Unchecked:
                    #print "hz changed from %.2f to %.2f, not logging yet" % (preHz, hz)
                    QtGui.QTreeWidgetItem.setData(self, column, role, hz)

    def childStateChanged(self, child, newState):
        if self.checkState(0) == Qt.Unchecked:
            #print "Child Changed while not logging"
            QtGui.QTreeWidgetItem.setData(child, 0, Qt.CheckStateRole, newState)
        elif self.checkState(0) == Qt.PartiallyChecked:
            pass
            #print "Waiting for log callback"
        elif self.checkState(0) == Qt.Checked:
            QtGui.QTreeWidgetItem.setData(child, 0, Qt.CheckStateRole, newState)
            if self.noneSelected():
                #print "Last child unselected, remove logger"
                self.stopLog()
            elif self.allSelected():
                #print "All children selected while logging, update logger"
                self.requestLog()
            else:
                #print "Child Changed while logging, update logger"
                self.requestLog()


    def makeLog(self):
        """ Makes a log. All ative children are added """

        #print " -> Making new Log with %d ms interval"
        if self.lg:
            #print " ---> Previous Log detected, removing first"
            self.lg.delete()


        self.fm = FreqMonitor(window=max(10, int(1.2*float(self.text(2)))))
        self.lg = LogConfig(self.name, 1000/int(float(self.text(2))))
        #print " ---> Adding children to new log"
        for x in range(self.childCount()):
            c = self.child(x)
            if c.isChecked():
                name = c.log.group+"."+c.log.name
                self.lg.add_variable(name, c.log.ctype)
                #print " --- --> Adding child[%d]: [%s] to new log"%(x, name)

        #print " ---> Checking log with TOC"
        self.treeWidget().cf.log.add_config(self.lg)
        if self.lg.valid:
            #print " --- --> PASS"
            self.lg.data_received_cb.add_callback(self.logDataCB)
            self.lg.started_cb.add_callback(self.logStartedCB)
            self.lg.error_cb.add_callback(self.treeWidget().sig_logError.emit)
            self.lg.error_cb.add_callback(self.errorLog)
            #print " --- --> callbacks added, starting new log NOW"
            self.lg.start()
        else:
            #print " --- --> FAIL"
            self.errorLog(None, "Invalid Config")
            self.lg = None


    def setAllState(self, chkState, activeOnly=False):
        """ Set all children on or off without their setData function. If activeOnly only set ones who are not off """
        if activeOnly:
            #print " -> Setting all ACTIVE children to checkstate [%d]", chkState
            for x in range(self.childCount()):
                if self.child(x).isChecked():
                    self.child(x).setState(chkState)
                    #print " --->", self.child(x).log.name
        else:
            #print " -> Setting all children to checkstate [%d]", chkState
            for x in range(self.childCount()):
                self.child(x).setState(chkState)
                #print " --->", self.child(x).log.name


    def allSelected(self):
        """ Returns true if all children are selected """
        for x in range(self.childCount()):
            if not self.child(x).isChecked():
                return False
        return True


    def noneSelected(self):
        """ Returns true if no children are selected """
        for x in range(self.childCount()):
            if self.child(x).isChecked():
                return False
        return True




class LogItem(QTreeWidgetItem):
    """ Subname
        Can be turned on/off
    """

    def __init__(self, parent, log):
        super(LogItem, self).__init__(parent)
        self.log = log
        self.setFlags(self.flags() | Qt.ItemIsUserCheckable)
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.DisplayRole, log.name)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, log.ctype)
        self.readSettings()


    def setData(self, column, role, value):
        """ Detect changes in the check box by the user. Report to parent"""
        if role == Qt.CheckStateRole and column == 0 and self.getState() != value:
            self.parent().childStateChanged(self, value)


    def isChecked(self):
        """ returns true if the item is checked or partially checked """
        return self.checkState(0) != Qt.Unchecked

    def setState(self, chkState=Qt.Checked):
        """ Sets the checkbox to on/off without calling the setData CB """
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, chkState)

    def getState(self):
        """ Sets the checkbox to on/off without calling the setData CB """
        return self.checkState(0)

    def readSettings(self):
        """ Read the HZ and selected things to log from previous session """
        settings = QSettings("omwdunkley", "flieROS")
        # Read if checked
        onQv = settings.value("log_"+self.log.group+"."+self.log.name+"_on", QVariant(Qt.Unchecked))
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, onQv if onQv.toInt()[0]!=Qt.PartiallyChecked else Qt.Checked) #Default to checked


    def writeSettings(self):
        """ Save the HZ and selected group.names to log between sessions"""
        settings = QSettings("omwdunkley", "flieROS")
        # Save checked state
        settings.setValue("log_"+self.log.group+"."+self.log.name+"_on", self.data(0, Qt.CheckStateRole))



class LogManager(QTreeWidget):
    """ Class to manage all things logging. One can turn groups/vars on and off,
        save configs between sessions, set and monitor the update frequencies
    """
    sig_batteryUpdated = pyqtSignal(int)
    sig_batteryState = pyqtSignal(int)
    sig_cpuUpdated = pyqtSignal(float) #cpu usage in percent 0-100
    sig_logError = pyqtSignal(object, str) # block, msg
    sig_rosData = pyqtSignal(object, int, object) # data, time, rostime
    sig_rpy = pyqtSignal(float, float, float)
    sig_hoverTarget = pyqtSignal(float) #0 if not hovering, else ASL target
    sig_baroASL = pyqtSignal(float)
    sig_temp = pyqtSignal(float)
    sig_pressure = pyqtSignal(float)
    sig_accZ = pyqtSignal(float) #z acceleration
    sig_g = pyqtSignal(float) #g force
    sig_acc = pyqtSignal(float,float,float)
    sig_thrust = pyqtSignal(int)
    sig_motors = pyqtSignal(int,int,int,int) # M1 through M4
    #sig_sysCanFly = pyqtSignal(bool) #value never changes, fix in firmware


    def __init__(self, cf, parent=None):
        super(LogManager, self).__init__(parent)
        self.cf = cf

        self.toc = None
        self.timerHZUpdate = QTimer()
        self.timerHZUpdate.timeout.connect(self.updateFreqMonitor)
        self.setFreqMonitorFreq(hz=2)
        self.estimateHzOn = True

        self.headers = ['Name','State', 'HZ Desired', 'HZ Actual']#, 'ROS']
        self.setHeaderLabels(self.headers)
        self.header().setStretchLastSection(True)
        self.header().setResizeMode(0, QHeaderView.Stretch)
        self.header().setResizeMode(1, QHeaderView.ResizeToContents)
        self.header().setResizeMode(2, QHeaderView.ResizeToContents)
        self.header().setResizeMode(3, QHeaderView.ResizeToContents)
        #self.header().setResizeMode(4, QHeaderView.ResizeToContents)
        self.setAlternatingRowColors(True)

        # So we can select which cols we can change
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.itemDoubleClicked.connect(self.userStartEdit)

        # All log groups send errors to this signal
        self.sig_logError.connect(self.logError)

        self.cf.connected.add_callback(self.newToc)
        self.cf.disconnected.add_callback(self.purgeToc)

        # Yaw offset to compensate for gyro drift
        self.yawOffset = 0.0
        self.preYaw = 0.0

        # ASL offset
        self.groundLevel = 0.0 # ground level (offset from gui)
        self.preAsl = 0.0      # previous asl value
        self.aslOffset = 0.0   # offset from other barometer

        # If we should spam ROS messages or not
        self.pubRos = True


    @pyqtSlot(float)
    def setYawOffset(self, yaw):
        """ Due to gyro drift, we can manually add an offset to any yaw log """
        self.yawOffset = -yaw

    def getYaw(self):
        return self.preYaw


    @pyqtSlot(float)
    def setAslOffset(self, asl):
        """ asl offset from anohter flie"""
        self.aslOffset = asl

    @pyqtSlot(float)
    def setGroundLevel(self, gl):
        """ ground level offset from GUI"""
        self.groundLevel = gl

    def getASL(self):
        return self.preAsl



    @pyqtSlot(bool)
    def setPubToRos(self, on=True):
        self.pubRos = on
        rospy.loginfo("ROS Messages turned [%s]", "ON" if on else "OFF")



    def userStartEdit(self, item, col):
        """ Make sure only the target HZ can be changed """
        if col == 2:
            self.editItem(item, col)

    def setFreqMonitorFreq(self, hz=2):
        """ set how fast we with to estimate the log frequency """
        rospy.logdebug("Log rate estimation window set to %.2fms", 1000./hz)
        self.timerHZUpdate.setInterval(1000./hz)

    def updateFreqMonitor(self):
        """ Called periodically by the timerHZ timer. Makes each active log group update their log data frequency """
        for i in range(self.topLevelItemCount()):
            self.topLevelItem(i).updateFM()

    def logError(self, block, msg):
        """ All logging configurations report errors to this function """
        rospy.logerr("Logging error with %s: %s", block.name, msg )

    def newToc(self, uri):
        """ Called when a new TOC has been downloaded. Populate the table, create and start log configs"""

        # Sometimes its called twice, eg after a fast/weird reconnect
        if self.toc:
            self.purgeToc(uri)

        # Populate Table recursively
        self.toc = self.cf.log._toc.toc
        for g in sorted(self.toc.keys()):
            self.addTopLevelItem(LogGroup(self, g, self.toc[g]))

        # Start monitoring log throughput
        if self.estimateHzOn:
            self.timerHZUpdate.start()

    def saveSettings(self):
        """ This function is called when the flie disconnects. It saves the state of each log configuration (on/off, subset of params, hz) """
        for i in range(self.topLevelItemCount()):
            self.topLevelItem(i).writeSettings()

    def purgeToc(self, uri):
        """ Called when the flie disconnects. Stop monitoring HZ, saves the state clears the treeview """
        self.timerHZUpdate.stop()
        self.saveSettings()
        self.clear()
        self.toc = None


    def setEstimateHzOn(self, on):
        """ Turn on HZ estimation """
        if on != self.estimateHzOn:
            self.estimateHzOn = on
            rospy.loginfo("Log HZ Estimation %s", "ON" if on else "OFF")
            for i in range(self.topLevelItemCount()):
                self.topLevelItem(i).setEstimateHzOn(on)
            if on:
                # Show cols
                self.header().showSection(3)
                if self.toc:
                    self.timerHZUpdate.start()
            else:
                # Hide cols
                self.header().hideSection(3)
                if self.toc:
                    self.timerHZUpdate.stop()


    def incomingData(self, data, ts):
        """ All incoming data is routed to this function. Ros data is spammed from the tree groups """
        rostime = rospy.Time.now()



        # Battery
        if isGroup(data, "pm"):
            if hasAllKeys(data, ["vbat"], "pm"):
                self.sig_batteryUpdated.emit(int(1000*data["pm.vbat"]))
            if hasAllKeys(data, ["state"], "pm"):
                self.sig_batteryState.emit(data["pm.state"])

        # CPU Usage
        elif isGroup(data, "utilization"):
            if hasAllKeys(data, ["cpuUsage"], "utilization"):
                self.sig_cpuUpdated.emit(data["utilization.cpuUsage"])

        # RPY
        elif isGroup(data, "stabilizer"):
            if data.has_key("stabilizer.yaw"):
                # Yaw offset, also negate so its CW #TODO: gyro velocities negated???
                self.preYaw = -data["stabilizer.yaw"]
                data["stabilizer.yaw"] = normaliseAngle(self.yawOffset + self.preYaw)
            if hasAllKeys(data, ["roll","pitch","yaw"], "stabilizer"):
                self.sig_rpy.emit(data["stabilizer.roll"],data["stabilizer.pitch"],data["stabilizer.yaw"])
            if hasAllKeys(data, ["thrust"], "stabilizer"):
                self.sig_thrust.emit(data["stabilizer.thrust"])

        # Accelerometer
        elif isGroup(data, "acc"):
            if hasAllKeys(data, ["x","y","z"], "acc"):
                self.sig_acc.emit(data["acc.x"],data["acc.y"],data["acc.z"])
                self.sig_g.emit(math.sqrt(data["acc.x"]*data["acc.x"]+data["acc.y"]*data["acc.y"]+data["acc.z"]*data["acc.z"]))
            elif hasAllKeys(data, ["mag2"], "acc"):
                self.sig_g.emit(math.sqrt(data["mag2.x"]))
            if hasAllKeys(data, ["zw"], "acc"):
                self.sig_accZ.emit(data["acc.zw"])
                        # Hover Target

        # Motor
        elif isGroup(data, "motor"):
            if hasAllKeys(data, ["m1","m2","m3","m4"], "motor"):
                self.sig_motors.emit(thrustToPercentage(data["motor.m1"]), thrustToPercentage(data["motor.m2"]), thrustToPercentage(data["motor.m3"]), thrustToPercentage(data["motor.m4"]))

        # Barometer
        elif isGroup(data, "baro"):
            if 'baro.asl' in data:
                self.preAsl = self.preAsl*0.9 + data['baro.asl']*0.1 #little bit of smoothing
            for k in data.keys():
                if k.startswith('baro.asl'):
                    data[k] =  data[k] - self.groundLevel - self.aslOffset
            if hasAllKeys(data, ["asl"], "baro"):
                self.sig_baroASL.emit(data["baro.asl"])
            if hasAllKeys(data, ["temp"], "baro"):
                self.sig_temp.emit(data["baro.temp"])
            if hasAllKeys(data, ["pressure"], "baro"):
                self.sig_pressure.emit(data["baro.pressure"])

        # Hover Target
        elif isGroup(data, "altHold"):
            if hasAllKeys(data, ["target"], "altHold"):
                # baro offset
                data["altHold.target"] =  data["altHold.target"] - self.groundLevel - self.aslOffset
                self.sig_hoverTarget.emit(data["altHold.target"])



        ## Sys can fly TODO: doesnt seem to work
        #elif isGroup(data, "sys"):
        #    if hasAllKeys(data, ["canfly"], "sys"):
        #        self.sig_sysCanFly.emit(data["sys.canfly"]>0)


        # ROS
        if self.pubRos:
            self.sig_rosData.emit(data, ts, rostime) # Wall time ??


def normaliseAngle( angle):
    """ Normalises an angle to +-180 """
    if angle > 180.0 or angle == -180.0:
        angle = -180.0 + abs(angle + 180.0) % (abs(-180.0) + abs(180.0))
    elif angle < -180.0 or angle == 180.0:
        angle = 180.0 - abs(angle - -180.0) % (abs(-180.0) + abs(180.0))
    return angle