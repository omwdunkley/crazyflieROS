from gi.overrides import keysyms
from guiqwt.io import _add_all_supported_files
from cflib.crazyflie import Crazyflie

import rospy
import roslib
from time import time
roslib.load_manifest('crazyflieROS')
from crazyflieROS import msg

__author__ = 'OMWDunkley'
__all__ = ['LogManager']


from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot,  QVariant, QTimer, QSettings
from PyQt4.QtGui import  QTreeWidget, QTreeWidgetItem, QAbstractItemView,QMessageBox,QHeaderView
from cflib.crazyflie.log import Log, LogConfig, LogVariable

import logging
logger = logging.getLogger(__name__)



# class DefaultType:
#     def __init__(self, parent, name, hz):
#         self.parent = parent
#         self.name = name
#         self.type = eval("msg."+name)
#         self.func = eval("parent."+"name+CB")
#         self.hz = 50




class FreqMonitor():
    """
    Modified from ros_comm / tools / rostopic / src / rostopic / __init__.py
    ROSTopicHz receives messages for a topic and computes frequency stats
    """
    def __init__(self,window=200):
        self.last_printed_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.times =[]
        self.window_size = window
        self.last_rate = 0


    def count(self):
        # curr_rostime = rospy.get_rostime()
        # # time reset
        # if curr_rostime.is_zero():
        #     if len(self.times) > 0:
        #         # print("time has reset, resetting counters")
        #         self.times = []
        #     return
        curr = time()
        if self.msg_t0 < 0 or self.msg_t0 > curr:
            self.msg_t0 = curr
            self.msg_tn = curr
            self.times = []
        else:
            self.times.append(curr - self.msg_tn)
            self.msg_tn = curr

        #only keep statistics for the last X messages so as not to run out of memory
        if len(self.times) > self.window_size - 1:
            self.times.pop(0)

    def get_hz(self,use_cached = False):
        if use_cached:
            return self.last_rate
        if not self.times:
            rate = 0
        elif self.msg_tn == self.last_printed_tn:
            rate = 0
        else:
            n = len(self.times)
            #rate = (n - 1) / (rospy.get_time() - self.msg_t0)
            mean = sum(self.times) / n
            rate = 1./mean if mean > 0. else 0
            self.last_printed_tn = self.msg_tn
        self.last_rate = rate
        return rate










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
        #self.setCheckState(0, Qt.Unchecked)

        # Keep track of if all/none of the children are active
        self.cAll = True
        self.cNone = True

        # Monitor the incoming frequency
        self.fm = None
        self.fmOn = True

        # log config
        self.lg = None

        # Show text
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.DisplayRole, QVariant(name))
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Off")
        QtGui.QTreeWidgetItem.setData(self, 3, Qt.DisplayRole, QVariant(0))
        self.readSettings()

        # Initialise Children
        for c in sorted(children.keys()):
            self.addChild(LogItem(self, children[c]))

        # Now everything is initialised except the logging
        # Start logging if active
        if self.checkState(0) == Qt.Checked:
            self.requestLog()



    def setEstimateHzOn(self, on):
        """ If on then we esstiamte the HZ """
        self.fmOn = on




    def readSettings(self):
        """ Read the HZ and selected things to log from previous session """
        settings = QSettings("omwdunkley", "flieROS")

        # Read target HZ
        hzQv = settings.value("log_"+self.name+"_hz",QVariant(20))
        self.fm = FreqMonitor(window=hzQv.toInt()[0])
        QtGui.QTreeWidgetItem.setData(self, 2, Qt.DisplayRole, hzQv)

        # Read if checked. If partially checked, uncheck
        onQv = settings.value("log_"+self.name+"_on", QVariant(Qt.Unchecked))
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, onQv if onQv.toInt()[0]!=Qt.PartiallyChecked else Qt.Unchecked)


    def writeSettings(self):
        """ Save the HZ and selected things to log between sessions"""
        settings = QSettings("omwdunkley", "flieROS")

        # Save target HZ
        settings.setValue("log_"+self.name+"_hz", self.data(2, Qt.DisplayRole))

        # Save checked state
        settings.setValue("log_"+self.name+"_on", self.data(0, Qt.CheckStateRole))

        # Save children state too

        for x in range(self.childCount()):
            self.child(x).writeSettings()


    def updateFM(self):
        # if not self.isActive()
        QtGui.QTreeWidgetItem.setData(self, 3, Qt.DisplayRole, self.fm.get_hz())


    def logDataCB(self, ts, data, lg):
        """ Our data from the log """
        print "LogCB data:", data.keys()

        # Update out HZ monitor
        if self.fmOn:
            self.fm.count()


    def logStartedCB(self, smth):
        print "LogCB Started"
        """ Called when we actually start logging """
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Checked)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "On")

    def logAddedCB(self, l):
        """ Called when log added """
        print "LogCB added"
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Checked)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Added")


    def requestLog(self):
        """ user requested start """
        print "Requested log start"
        if self.noneSelected():
            self.setAllState(Qt.PartiallyChecked)
        else:
            self.setAllState(Qt.PartiallyChecked, activeOnly=True)
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.PartiallyChecked)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Requested")
        #TODO



    def stopLog(self):
        """ User request halt """
        print "Requested log stop"
        if self.allSelected():
            self.setAllState(Qt.Unchecked)
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Unchecked)
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Off")


    def errorLog(self, block, msg):
        """ When a log error occurs """
        print "Log error: %s:", msg
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, "Err: %s"%msg)
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Unchecked)
        if self.allSelected():
            self.setAllState(Qt.Unchecked)


    def setData(self, column, role, value):
        """ Only allow changing of the check state and the hz """

        # GROUP ON/OFF
        preValue = self.data(0, Qt.CheckStateRole)
        if role == Qt.CheckStateRole and column == 0 and preValue != value:
            # Check box changed, allow and trigger

            if preValue==Qt.PartiallyChecked:
                print "Waiting for log callback, cannot change config now"

            elif value==Qt.Checked:
                self.requestLog()

            elif value==Qt.Unchecked:
                self.stopLog()


        # HZ CHANGED
        if column == 2 and self.data(column, Qt.DisplayRole) != value:
            if preValue==Qt.PartiallyChecked:
                print "Waiting for log callback, cannot change hz now"
            else:
                print "hz changed from %d to %d", self.data(column, Qt.DisplayRole).toInt(), value.toInt()
                QtGui.QTreeWidgetItem.setData(self, column, role, value)
                self.requestLog()

    def childStateChanged(self, child, newState):
        if self.checkState(0) == Qt.Unchecked:
            print "Child Changed while not logging"
            QtGui.QTreeWidgetItem.setData(child, 0, Qt.CheckStateRole, newState)
        elif self.checkState(0) == Qt.PartiallyChecked:
            print "Waiting for log callback"
        elif self.checkState(0) == Qt.Checked:
            QtGui.QTreeWidgetItem.setData(child, 0, Qt.CheckStateRole, newState)
            if self.noneSelected():
                print "Last child unselected, remove logger"
                self.stopLog()
            else:
                print "Child Changed while logging, update logger"
                self.requestLog()



    def setupLog(self,hz):
        """ Update the log by deleteing any previous log and making a new one """
        # Delete the current log
        if self.lg:
            self.lg.delete()
        if self.isActive():
            self.lg = LogConfig(self.name, 1000/hz)
            for x in range(self.childCount()):
                c = self.child(x)
                if c.isActive():
                    name = c.log.group+".l"+c.log.name
            self.lg.add_variable(name, c.log.ctype)
            self.treeWidget().cf.log.add_config(self.lg)
            if self.lg.valid:
                self.lg.data_received_cb.add_callback(self.logDataCB)
                self.lg.error_cb.add_callback(self.treeWidget().sig_logError.emit)
                #if self.isActive():
                self.lg.start()
            else:
                logger.warning("logging block [%s] not valid!", self.lg.name)
                QtGui.QTreeWidgetItem.setData(self, 0, Qt.CheckStateRole, Qt.Unchecked)


    def makeLog(self):
        self.lg = LogConfig(self.name, 1000/int(self.text(2)))
        for x in range(self.childCount()):
            c = self.child(x)
            if c.isActive():
                name = c.log.group+"."+c.log.name
        self.lg.add_variable(name, c.log.ctype)
        self.treeWidget().cf.log.add_config(self.lg)
        if self.lg.valid:
            self.lg.data_received_cb.add_callback(self.logDataCB)
            self.lg.started_cb.add_callback(self.logStartedCB)
            self.lg.error_cb.add_callback(self.treeWidget().sig_logError.emit)
            return True
        else:
            logger.warning("logging block [%s] not valid!", self.lg.name)
            return False


    def setAllState(self, chkState, activeOnly=False):
        """ Set all children on or off without their setData function. If activeOnly only set ones who are not off """
        if activeOnly:
            for x in range(self.childCount()):
                if self.child(x).isChecked():
                    self.child(x).setState(chkState)
        else:
            for x in range(self.childCount()):
                self.child(x).setState(chkState)


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
    """ Class to manage all things logging.
        One can turn on/off log group/names,
        monitor their frequency,
        change their frequency
    """
    sig_batteryUpdated = pyqtSignal(int)
    sig_logError = pyqtSignal(object, str) # block, msg

    def __init__(self, cf, parent=None):
        super(LogManager, self).__init__(parent)
        self.cf = cf
        self.toc = None
        self.timerHZUpdate = QTimer()
        self.timerHZUpdate.timeout.connect(self.updateFreqMonitor)
        self.setFreqMonitorFreq(hz=2)
        self.estimateHzOn = True

        self.headers = ['Name','State', 'HZ Desired', 'HZ Actual']
        self.setHeaderLabels(self.headers)
        self.header().setStretchLastSection(True)
        self.header().setResizeMode(0, QHeaderView.Stretch)
        self.header().setResizeMode(1, QHeaderView.ResizeToContents)
        self.header().setResizeMode(2, QHeaderView.ResizeToContents)
        self.header().setResizeMode(3, QHeaderView.ResizeToContents)
        self.setAlternatingRowColors(True)

        # So we can select which cols we can change
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.itemDoubleClicked.connect(self.userStartEdit)

        # All log groups send errors to this signal
        self.sig_logError.connect(self.logError)

        self.cf.connected.add_callback(self.newToc)
        self.cf.disconnected.add_callback(self.purgeToc)

    def userStartEdit(self, item, col):
        """ Make sure only the target HZ can be changed """
        if col == 2:
            self.editItem(item, col)

    def setFreqMonitorFreq(self, hz=2):
        """ set how fast we with to estimate the log frequency """
        logger.info("Log rate estimation window set to %.2fms", 1000./hz)
        self.timerHZUpdate.setInterval(1000./hz)

    def updateFreqMonitor(self):
        """ Called periodically by the timerHZ timer. Makes each active log group update their log data frequency """
        for i in range(self.topLevelItemCount()):
            self.topLevelItem(i).updateFM()

    def logError(self, block, msg):
        """ All logging configurations report errors to this function """
        logger.error("Logging error with %s: %s", block.name, msg )


    def newToc(self, uri):
        """ Called when a new TOC has been downloaded. Populate the table, create and start log configs"""

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
            logger.info("Log HZ Estimation %s", "ON" if on else "OFF")
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


# SIMPLE VIEWER


# class LogItem(QTreeWidgetItem):
#     """ Subname
#         Can be turned on/off
#     """
#
#     def __init__(self, parent, log):
#         super(LogItem, self).__init__(parent)
#         self.log = log
#         self.setText(0, log.name)
#         self.setText(1, str(log.ident))
#         self.setText(2, log.ctype)
#         self.setText(3, "RO" if log.access else "RW")
#
#
# class LogGroup(QTreeWidgetItem):
#     """ Group
#         Can be turned on/off
#     """
#
#     def __init__(self, parent, name, children):
#         super(LogGroup, self).__init__(parent)
#         self.setText(0, name)
#         # add checkbox
#         # add hz
#         # add hz
#         for c in children.keys():
#             self.addChild(LogItem(self, children[c]))
#
# class LogManager(QTreeWidget):
#     """ Class to manage all things logging.
#         One can turn on/off log group/names,
#         monitor their frequency,
#         change their frequency
#     """
#     sig_batteryUpdated = pyqtSignal(int)
#
#     def __init__(self, cf, parent=None):
#         super(LogManager, self).__init__(parent)
#         self.cf = cf
#         self.toc = None
#
#         self.headers = ['ID','Name', 'On', 'HZ Desired', 'HZ Actual']
#         self.setColumnCount(len(self.headers))
#         self.setHeaderLabels(self.headers)
#         self.setAlternatingRowColors(True)
#
#         self.cf.connected.add_callback(self.newToc)
#         self.cf.disconnected.add_callback(self.purgeToc)
#
#     def newToc(self, uri):
#         self.toc = self.cf.log._toc.toc
#
#         for g in self.toc.keys():
#             self.addTopLevelItem(LogGroup(self, g, self.toc[g]))
#
#
#     def purgeToc(self, uri):
#         self.clear()