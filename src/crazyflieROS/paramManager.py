__author__ = 'OMWDunkley'
__all__ = ['ParamManager']


from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot,  QVariant
from PyQt4.QtGui import  QTreeWidget, QTreeWidgetItem, QAbstractItemView
import rospy

#import logging
#logger = logging.getLogger(__name__)



class ParamAccessItem(QTreeWidgetItem):
    """ Represents a row in the tree that is either RO or RW and contains child groups """
    def __init__(self, access, groups, parent):
        super(ParamAccessItem, self).__init__(parent, type=1001)
        self.setText(0, access)
        self.setExpanded(True)
        for key in groups.keys():
            self.addChild(ParamGroupItem(key, groups[key], self, access=="RW"))

    def getChildren(self):
        """ Gets all children - used to iterate through all items in the tree """
        return [self.child(x) for x in range(self.childCount())]


class ParamGroupItem(QTreeWidgetItem):
    """ Row in the tree that is a group and contains params as children that can be edited if editable = true """
    def __init__(self, group, params, parent, editable):
        super(ParamGroupItem, self).__init__(parent, type=1002)
        self.setText(0, group)
        self.setExpanded(True)
        for param in params:
            self.addChild(ParamItem(param, self, editable))

    def getChildren(self):
        """ Gets all children - used to iterate through all items in the tree """
        return [self.child(x) for x in range(self.childCount())]



class ParamItem(QTreeWidgetItem):
    def __init__(self, param, parent, editable):
        """ A parameter. If editable, the user can edit it. This does not change the value, but
            sends a request to the flie to update the value. The flie returns the new value which
            is then updated in this row
        """
        super(ParamItem, self).__init__(parent, type=1003)

        # Our param data
        self.param = param
        if editable:
            self.setFlags(self.flags() | Qt.ItemIsEditable)

        # Initial Populate
        QtGui.QTreeWidgetItem.setData(self, 0, Qt.DisplayRole, QVariant(param.name))
        QtGui.QTreeWidgetItem.setData(self, 1, Qt.DisplayRole, QVariant(param.ctype))
        QtGui.QTreeWidgetItem.setData(self, 2, Qt.DisplayRole, "Updating...")

        # Flie Updates
        self.cf = self.treeWidget().cf
        self.cf.param.add_update_callback(group=param.group, name=param.name, cb=self.updateValueCB)

    def updateValueCB(self, name, value):
        """ Set the value from the flie callback """
        QtGui.QTreeWidgetItem.setData(self, 2, Qt.DisplayRole, QVariant(value))
        #self.setData(2, Qt.DisplayRole, QVariant(value))

    def requestUpdate(self):
        """ Updates the specific value from the flie """
        self.cf.param.request_param_update("%s.%s" % (self.param.group, self.param.name))

    def requestValueChange(self, value):
        """ Send new value to flie. Flie callback updates GUI """
        try:
            self.cf.param.set_value("%s.%s" % (self.param.group, self.param.name), value)
        except NameError, err:
            QtGui.QTreeWidgetItem.setData(self, 2, Qt.DisplayRole, QVariant("Invalid..."))
            rospy.logwarn("Parameter [%s.%s] could not be updated from [%s] to [%s]: %s ", self.param.group, self.param.name, self.text(2), value, err)
            self.requestUpdate()

    def setData(self, column, role, value):
        """ Reimplement the setData role to stop the user changing paramters. The Users changes are sent
            to the flie and the flie update callback changes the displayed parameters
        """
        v = str(value.toString())
        if v!=self.text(2):
            QtGui.QTreeWidgetItem.setData(self, column, role, QVariant("Updating..."))
            self.requestValueChange(v)

    def getChildren(self):
        """ Gets all children - used to iterate through all items in the tree """
        return [self.child(x) for x in range(self.childCount())]






class ParamManager(QTreeWidget):
    """ Class that sends/receives parameters """
    sig_connected = pyqtSignal(str)
    sig_disconnected = pyqtSignal(str)
    sig_firmware = pyqtSignal(str, str)
    sig_baroFound = pyqtSignal(bool)
    sig_magFound = pyqtSignal(bool)
    sig_hover= pyqtSignal(bool) # true if hover mode active
    sig_test = pyqtSignal(str, bool) #sensor, bool

    # Emit signals with additional details from the RO firmware
    # Firmware / clean
    # Sensors found HMC5883L/MS5611
    # Sensor Tests MPU6050/HMC5883L/MS5611
    # Crazyradio version

    def __init__(self, cf, parent=None):
        super(ParamManager, self).__init__(parent)
        self.cf = cf
        self.fw = {}

        self.setColumnCount(3)
        self.setHeaderLabels(['Name', 'Type', 'Value'])
        self.setAlternatingRowColors(True)

        self.cf.connected.add_callback(self.populate)
        self.cf.disconnected.add_callback(self.uppopulate)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.itemDoubleClicked.connect(self.userStartEdit)

    def userStartEdit(self, item, col):
        if col == 2:
            self.editItem(item, col)


    def populate(self, uri):
        """Populate the model with data from the param TOC"""
        toc = self.cf.param.toc.toc

        # Sort into data[RW/RO][Group] = [p1, p2, ..., pN]
        data = {"RO":{}, "RW":{}}
        for group in sorted(toc.keys()):
            g = []
            for param in sorted(toc[group].keys()):
                g.append(toc[group][param])
            data[toc[group][param].get_readable_access()][group] = g

        # Add data recursively
        for key in data.keys():
            self.insertTopLevelItem(0,ParamAccessItem(key, data[key], self))

        # Listen to specific updates
        self.cf.param.add_update_callback(group="imu_sensors", cb=self.imuSensorsCB)
        self.cf.param.add_update_callback(group="imu_tests",   cb=self.imuSensorTestsCB)
        self.cf.param.add_update_callback(group="firmware",    cb=self.firmwareCB)
        self.cf.param.add_update_callback(group="flightmode",  cb=self.hoverCB)

        # Read TOC Values
        self.forceUpdate()



    def hoverCB(self, name, val):
        if 'althold' in name:
            self.sig_hover.emit(eval(val)!=0)

    def firmwareCB(self, name, val):
        if "revision0" in name:
            self.fw["v0"] = eval(val)
        if "revision1" in name:
            self.fw["v1"] = eval(val)
        if "modified" in name:
            self.fw["mod"] = "Modified" if eval(val) else "Clean"

        if len(self.fw)==3:
            firmware = "{:x}{:x}".format(self.fw["v0"], self.fw["v1"])
            self.sig_firmware.emit(firmware, self.fw["mod"])



    def imuSensorsCB(self, name, val):
        """Callback for sensor found paramters"""
        if "MS5611" in name:
            self.sig_baroFound.emit(eval(val))
        elif "HMC5883L" in name:
            self.sig_magFound.emit(eval(val))


    def imuSensorTestsCB(self, name, val):
        """Callback for sensor test parameters"""
        self.sig_test.emit(name[name.index('.') + 1:],eval(val))


    def uppopulate(self, uri):
        self.cf.param.remove_update_callbacks()
        self.clear()


    def forceUpdate(self):
        """ Update all Params from the flie """
        for a in [self.topLevelItem(x) for x in range(self.topLevelItemCount())]:
            for g in a.getChildren():
                for p in g.getChildren():
                    p.requestUpdate()

    def setWidths(self, ws):
        """ Set Column Widths """
        for i in range(min(len(ws), self.columnCount())):
            self.setColumnWidth(i,ws[i].toInt()[0])

    def getWidths(self):
        """ Get colum Width for restoring later """
        ws = []
        for i in range(self.columnCount()):
            ws.append(str(self.columnWidth(i)))
        return ws

    def getParams(self,RO=False):
        """ Get a list of group of list of params
        """
        #TODO
        return None

    def setRWParams(self, group, param, value):
        """ Sets a RW param """
        #TODO
        return None