from gi.overrides import keysyms
from guiqwt.io import _add_all_supported_files
from cflib.crazyflie import Crazyflie



__author__ = 'OMWDunkley'
__all__ = ['LogManager']


from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot,  QVariant
from PyQt4.QtGui import  QTreeWidget, QTreeWidgetItem, QAbstractItemView,QMessageBox
from cflib.crazyflie.log import Log, LogConfig, LogVariable

import logging
logger = logging.getLogger(__name__)







default = {

}












class LogItem(QTreeWidgetItem):
    """ Subname
        Can be turned on/off
    """

    def __init__(self, parent, log):
        super(LogItem, self).__init__(parent)
        self.log = log
        self.setText(0, log.name)


class LogGroup(QTreeWidgetItem):
    """ Group
        Can be turned on/off
    """

    def __init__(self, parent, name, children):
        super(LogGroup, self).__init__(parent)
        self.setText(0, name)
        # add checkbox
        # add hz
        # add hz
        for c in children.keys():
            self.addChild(LogItem(self, children[c]))







class LogManager(QTreeWidget):
    """ Class to manage all things logging.
        One can turn on/off log group/names,
        monitor their frequency,
        change their frequency
    """

    sig_blockAdded  = pyqtSignal(object)
    sig_disconnected   = pyqtSignal(str)
    sig_batteryUpdated = pyqtSignal(int)
    sig_logError       = pyqtSignal(object, str)

    def __init__(self, cf, parent=None):
        super(LogManager, self).__init__(parent)
        self.cf = cf
        self.toc = None

        self.headers = ['ID','Name', 'On', 'HZ Desired', 'HZ Actual']
        self.setColumnCount(len(self.headers))
        self.setHeaderLabels(self.headers)
        self.setAlternatingRowColors(True)



        self.cf.connected.add_callback(self.newTOC)

    def newTOC(self, uri):
        self.toc = self.cf.log._toc.toc

        for g in self.toc.keys():
            self.addTopLevelItem(LogGroup(self, g, self.toc[g]))












#
# class LogManager(QTreeWidget):
#     """ Class that sends/receives parameters """
#     sig_blockAdded  = pyqtSignal(object)
#     sig_disconnected   = pyqtSignal(str)
#     sig_batteryUpdated = pyqtSignal(int)
#     sig_logError       = pyqtSignal(object, str)
#
#     def __init__(self, cf, parent=None):
#         super(LogManager, self).__init__(parent)
#         self.cf = cf
#
#         self.setColumnCount(3)
#         self.setHeaderLabels(['Name', 'Type', 'Value'])
#         self.setAlternatingRowColors(True)
#
#         self.sig_blockAdded.connect(self.logBlockAdded)
#         self.sig_logError.connect(self.loggingErrorMsg)
#
#         self.cf.log.block_added_cb.add_callback(self.sig_blockAdded.emit)
#
#         self.cf.connected.add_callback(self.populate)
#         self.cf.disconnected.add_callback(self.uppopulate)
#
#     @pyqtSlot(object)
#     def logBlockAdded(self, block):
#         print "Log Block added", block
#
#
#     def populate(self, uri):
#         """Populate the model with data from the param TOC"""
#         toc = self.cf.param.toc.toc
#
#         self.addLogBlock( "Battery", [("pm.vbat", "float")], 100, lambda t,d,l: self.sig_batteryUpdated.emit(int(d["pm.vbat"] * 1000)) )
#
#
#
#     def logAccCB(self, timestamp, data, logConf):
#         pass
#     def logStabilizerCB(self, timestamp, data, logConf):
#         pass
#     def logMotorCB(self, timestamp, data, logConf):
#         pass
#     def logGyroCB(self, timestamp, data, logConf):
#         pass
#     def logMagCB(self, timestamp, data, logConf):
#         pass
#     def logSysCB(self, timestamp, data, logConf):
#         pass
#     def logBaroCB(self, timestamp, data, logConf):
#         pass
#     def logAltHoldCB(self, timestamp, data, logConf):
#         pass
#     def logVPidCB(self, timestamp, data, logConf):
#         pass
#     def logGyroCB(self, timestamp, data, logConf):
#         pass
#     def logPMCB(self, timestamp, data, logConf):
#         pass
#
#
#
#
#     def loggingErrorMsg(self, logConf, msg):
#         logger.error("LOG Error when starting log config [%s]: %s" % (logConf.name, msg))
#
#     def uppopulate(self, uri):
#         self.cf.param.remove_update_callbacks()
#         self.clear()
#
#     def newLogData(self, timestamp, data, logConf):
#         print data
#         #self.sig_batteryUpdated.emit(int(data["pm.vbat"] * 1000))
#
#     def addLogBlock(self, name, nts, hz, f):
#         lg = LogConfig(name, 1000/hz)
#         for n,t in nts:
#             lg.add_variable(n, t)
#         self.cf.log.add_config(lg)
#         if lg.valid:
#             lg.data_received_cb.add_callback(f)
#             lg.error_cb.add_callback(self.sig_logError.emit)
#             lg.start()
#         else:
#             logger.warning("logging block [%s] not valid!", name)
#
#
