import logging, os, time

from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot, QThread, QObject, QTimer, QSettings, QPoint, QSize, QVariant

from logManager import LogManager
from paramManager import ParamManager
from FlieManager import FlieControl, STATE
from rosTools import generateRosMessages, ROSNode
from ui.driverGUI import Ui_MainWindow
from cflib.crtp import scan_interfaces, init_drivers, get_interfaces_status

logger = logging.getLogger(__name__)




class MSGTYPE:
    """ Class to represent message types """
    NONE   = -1
    INFO   =  0
    WARN   =  1
    ERROR  =  2
    DEBUG  =  3



class Message:
    """ Struct class to hold a message and a beep type"""
    def __init__(self, msg="", msgtype = MSGTYPE.INFO, freq=1000, length=0, repeat=1 ):
        self.msg = msg
        self.f = freq
        self.l = length
        self.r = repeat
        self.beep = length>0
        self.type = msgtype




class DriverWindow(QtGui.QMainWindow ):
    """ Main window and application """

    sig_requestScan = pyqtSignal()
    sig_requestConnect = pyqtSignal(str)
    sig_requestDisconnect = pyqtSignal()

    def __init__(self):
        super(DriverWindow, self).__init__()
        #uic.loadUi('ui/driverGUI.ui', self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.tabWidget.setCurrentIndex(0)
        self.resetInfo()
        self.ui.labelTest = {"MS5611": self.ui.label_baroTest, "MPU6050": self.ui.label_MPUTest, "HMC5883L": self.ui.label_magTest}

        # FLIE and ROS members
        self.state = STATE.DISCONNECTED
        self.ros = ROSNode()
        self.flie = FlieControl()
        self.ui.checkBox_pktHZ.toggled.connect(lambda on: self.flie.setPacketUpdateSpeed(self.ui.spinBox_pktHZ.value() if on else 0 ))
        self.ui.spinBox_pktHZ.valueChanged.connect(self.flie.inKBPS.setHZ)
        self.ui.spinBox_pktHZ.valueChanged.connect(self.flie.outKBPS.setHZ)
        self.ui.spinBox_pktHZ.valueChanged.emit(self.ui.spinBox_pktHZ.value()) # force update

        # Set up ParamManager
        self.paramManager = ParamManager(self.flie.crazyflie, self)
        self.ui.tab_param.layout().addWidget(self.paramManager)

        # Set up LogManager
        self.logManager = LogManager(self.flie.crazyflie, self)
        self.ui.tab_log.layout().addWidget(self.logManager)
        self.ui.checkBox_logHZ.toggled.connect(self.logManager.setEstimateHzOn)
        self.ui.spinBox_logHZ.valueChanged.connect(self.logManager.setFreqMonitorFreq)




        # init previous settings
        self.readSettings()

        # Defaults according to settings within GUI
        self.beepOn = self.ui.checkBox_beep.isChecked()
        self.killOn = self.ui.checkBox_kill.isChecked()
        self.autoReconnectOn = self.ui.checkBox_reconnect.isChecked()
        self.startupConnectOn = self.ui.checkBox_startupConnect.isChecked()




        # Set up URI scanner
        self.scanner = ScannerThread()
        self.scanner.start()


        # Connections from GUI
        self.ui.pushButton_connect.clicked.connect(lambda : self.connectPressed(self.ui.comboBox_connect.currentText())) # Start button -> connect
        self.ui.comboBox_connect.currentIndexChanged.connect(self.uriSelected)

        self.ui.checkBox_beep.toggled.connect(self.setBeep)
        self.ui.checkBox_kill.toggled.connect(self.setKill)
        self.ui.checkBox_reconnect.toggled.connect(self.setAutoReconnect)
        self.ui.checkBox_startupConnect.toggled.connect(self.setStartupConnect)
        self.ui.pushButton_genRosMsg.clicked.connect(self.genRosMsg)


        # Connections to GUI
        self.flie.sig_packetSpeed.connect(self.updatePacketRate)
        self.flie.sig_flieLink.connect(self.ui.progressbar_link.setValue)
        self.paramManager.sig_baroFound.connect(lambda found: self.ui.label_baroFound.setText("Yes" if found else "No"))
        self.paramManager.sig_magFound.connect(lambda found: self.ui.label_magFound.setText("Yes" if found else "No"))
        self.paramManager.sig_test.connect(lambda name, p: self.ui.labelTest[str(name)].setText("Pass" if p else "FAIL"))
        self.paramManager.sig_firmware.connect(lambda fw, mod: self.ui.label_fw.setText(fw))
        self.paramManager.sig_firmware.connect(lambda fw, mod: self.ui.label_fwMod.setText(mod))
        self.logManager.sig_batteryUpdated.connect(self.ui.progressbar_bat.setValue)
        self.flie.inKBPS.sig_KBPS.connect(lambda hz: self.updatePacketRate(self.ui.progressBar_pktIn,hz))
        self.flie.outKBPS.sig_KBPS.connect(lambda hz: self.updatePacketRate(self.ui.progressBar_pktOut,hz))


        # Connections GUI to GUI


        # Connections Within
        self.scanner.sig_foundURI.connect(self.receiveScanURI)
        self.sig_requestScan.connect(self.scanner.scan)
        self.sig_requestConnect.connect(self.flie.requestConnect)
        self.sig_requestDisconnect.connect(self.flie.requestDisconnect)
        self.flie.sig_stateUpdate.connect(self.updateFlieState)
        self.flie.sig_console.connect(self.ui.console.insertPlainText)

        # Show window
        self.show()

        # Initiate an initial Scan
        init_drivers(enable_debug_driver=False)
        self.startScanURI()


    @pyqtSlot()
    def genRosMsg(self):
        #TODO: should do this in its own thread, or even as a progress bar
        self.ui.pushButton_genRosMsg.setEnabled(False)
        self.ui.pushButton_genRosMsg.setText("Generating")
        generateRosMessages(self.flie.getLogToC())
        self.ui.pushButton_genRosMsg.setEnabled(True)
        self.ui.pushButton_genRosMsg.setText("Generate ROS msgs from ToC")


    def closeEvent(self, event):
        """ Intercept shutdowns to cleanly disconnect from the flie and cleanly shut ROS down too """
        logger.info("Close Event")

        # Shut Down the Flie
        if self.state < STATE.GEN_DISCONNECTED:
            logger.info("Triggering flie disconnect for shutdown")
            self.ui.pushButton_connect.setText("Shutting Down...")
            self.flie.requestDisconnect()

        # Clean up ROS
        #TODO

        # Save State
        self.writeSettings()

        # Commence shutdown
        event.accept()

    def readSettings(self):
        """ Load setrtings from previous session """
        settings = QSettings("omwdunkley", "flieROS")
        self.resize(settings.value("size", QVariant(QSize(300, 500))).toSize())
        self.move(settings.value("pos", QVariant(QPoint(200, 200))).toPoint())

        self.ui.checkBox_reconnect.setChecked(settings.value("autoReconnect", QVariant(self.ui.checkBox_reconnect.isChecked())).toBool())
        self.ui.checkBox_beep.setChecked(settings.value("beepOn", QVariant(self.ui.checkBox_beep.isChecked())).toBool())
        self.ui.checkBox_startupConnect.setChecked(settings.value("startConnect", QVariant(self.ui.checkBox_startupConnect)).toBool())

        self.ui.checkBox_pktHZ.setChecked(settings.value("pktHzOn", QVariant(self.ui.checkBox_pktHZ.isChecked())).toBool())
        self.ui.checkBox_logHZ.setChecked(settings.value("logHzOn", QVariant(self.ui.checkBox_logHZ.isChecked())).toBool())
        self.ui.horizontalSlider_pktHZ.setValue(settings.value("pktHzVal", QVariant(self.ui.horizontalSlider_pktHZ.value())).toInt()[0])
        self.ui.horizontalSlider_logHZ.setValue(settings.value("logHzVal", QVariant(self.ui.horizontalSlider_logHZ.value())).toInt()[0])

        self.logManager.header().restoreState(settings.value("logTreeH", self.logManager.header().saveState()).toByteArray())
        self.paramManager.header().restoreState(settings.value("paramTreeH", self.paramManager.header().saveState()).toByteArray())

    def writeSettings(self):
        """ Write settings to load at next start up """
        settings = QSettings("omwdunkley", "flieROS")
        settings.setValue("pos", QVariant(self.pos()))
        settings.setValue("size", QVariant(self.size()))

        settings.setValue("autoReconnect", QVariant(self.ui.checkBox_reconnect.isChecked()))
        settings.setValue("beepOn", QVariant(self.ui.checkBox_beep.isChecked()))
        settings.setValue("startConnect", QVariant(self.ui.checkBox_startupConnect.isChecked()))

        settings.setValue("pktHzOn", QVariant(self.ui.checkBox_pktHZ.isChecked()))
        settings.setValue("logHzOn", QVariant(self.ui.checkBox_logHZ.isChecked()))
        settings.setValue("pktHzVal", QVariant(self.ui.horizontalSlider_pktHZ.value()))
        settings.setValue("logHzVal", QVariant(self.ui.horizontalSlider_logHZ.value()))

        settings.setValue("logTreeH",QVariant(self.logManager.header().saveState()))
        settings.setValue("paramTreeH",QVariant(self.paramManager.header().saveState()))

    @pyqtSlot(bool)
    def setBeep(self, on):
        self.beepOn = on
    @pyqtSlot(bool)
    def setKill(self, on):
        self.killOn = on
    @pyqtSlot(bool)
    def setAutoReconnect(self, on):
        self.autoReconnectOn = on
    @pyqtSlot(bool)
    def setStartupConnect(self, on):
        self.startupConnectOn = on

    def getCRTPStatus(self):
        interface_status =get_interfaces_status()
        for key in interface_status.keys():
            print key#,interface_status[key]
        self.ui.label_crv.setText(interface_status["radio"])

    def resetInfo(self):
        """ Resets the labels on the info tab """
        self.ui.label_baroFound.setText("")
        self.ui.label_magTest.setText("")
        self.ui.label_baroTest.setText("")
        self.ui.label_magFound.setText("")
        self.ui.label_MPUTest.setText("")
        self.ui.label_fw.setText("")
        self.ui.label_fwMod.setText("")
        self.ui.label_crv.setText("")

    @pyqtSlot(int,int)
    def updatePacketRate(self, pb, hz):
        """ Updates the number of pb with hz ensuring we adjust the maximum incase max<hz """
        if hz>pb.maximum():
            logger.warn("Maximum Packets Changed from %d/s -> %d/s", pb.maximum(), hz)
            pb.setMaximum(hz)
        pb.setValue(hz)


    def startScanURI(self):
        """ User Clicked Scan
            Remove all previously found URIs from dropdown
            Disable rescanning
        """
        self.ui.comboBox_connect.clear()

        self.ui.pushButton_connect.setText('Scanning')
        self.ui.pushButton_connect.setDisabled(True)

        #self.scanner.sig_requestScan.emit()
        self.sig_requestScan.emit()




    def receiveScanURI(self, uri):
        """ Results from URI scan
            Add them to dropdown
        """

        # None Found, enable rescanning
        self.ui.pushButton_connect.setDisabled(False)
        if not uri:
            self.ui.pushButton_connect.setText('Scan')
            return

        self.ui.pushButton_connect.setText('Connect')
        for i in uri:
            if len(i[1]) > 0:
                self.ui.comboBox_connect.addItem("%s - %s" % (i[0], i[1]))
            else:
                self.ui.comboBox_connect.addItem(i[0])

        self.ui.comboBox_connect.addItem("Rescan")

        if self.startupConnectOn:
            logger.info("Auto connecting to first found flie [ConnectOnStartUp = TRUE]")
            self.ui.pushButton_connect.clicked.emit(False)


    def uriSelected(self, uri):
        """ The user clicked on a URI or a scan request
            If SCAN was selected, initiate a scan
        """
        if self.ui.comboBox_connect.currentText() == "Rescan":
            self.startScanURI()

    def connectPressed(self, uri):
        """ The user pressed the connect button.
            Either rescan if there is no URI or
            Connect to the flie
        """
        # No URI Found
        if uri=="":
            self.startScanURI()
            return

        if self.state == STATE.CONNECTED:
            self.requestFlieDisconnect()
        else:
            self.requestFlieConnect(uri)

    def requestFlieConnect(self, uri):
        """ Request connection to the flie
        """
        self.ui.pushButton_connect.setText("Connecting...")
        self.ui.pushButton_connect.setEnabled(False)
        self.ui.comboBox_connect.setEnabled(False)
        self.sig_requestConnect.emit(uri)


    def requestFlieDisconnect(self):
        """ Request flie disconnect """

        self.ui.pushButton_connect.setText("Disconnecting...")
        self.sig_requestDisconnect.emit()
        #TODO: UI elements should be changed when the flie reports its disconnected, not when we press the button



    @pyqtSlot(int, str, str)
    def updateFlieState(self, state, uri, msg):
        """ Function that receives all the state updates from the flie.
        """
        self.state = state

        if state == STATE.CONNECTION_REQUESTED:
            self.beepMsg(Message(msg="Connection to [%s] requested" % uri))

        elif state == STATE.LINK_ESTABLISHED:
            self.beepMsg(Message(msg="Link to [%s] established" % uri, freq=3300, length=25, repeat=1))
            self.ui.pushButton_connect.setText("Download TOC...")

        elif state == STATE.CONNECTED:
            self.beepMsg(Message(msg="Connected to [%s]" % uri, freq=3500, length=10, repeat=2))
            self.ui.pushButton_connect.setText("Disconnect")
            self.ui.pushButton_connect.setEnabled(True)
            self.getCRTPStatus()
            self.ui.pushButton_genRosMsg.setEnabled(True)

        elif state == STATE.DISCONNECTED:
            self.beepMsg(Message(msg="Disconnected from [%s]" % uri, freq=120, length=0))
            self.ui.pushButton_connect.setText("Connect")
            self.ui.comboBox_connect.setEnabled(True)
            self.ui.pushButton_connect.setEnabled(True)
            self.resetInfo()

        elif state == STATE.CONNECTION_FAILED:
            self.beepMsg(Message(msgtype=MSGTYPE.WARN, msg="Connecting to [%s] failed: %s" % (uri, msg)))
            if self.autoReconnectOn:
                QTimer.singleShot(1000, lambda: self.ui.pushButton_connect.clicked.emit(False))
                self.ui.pushButton_connect.setText("Auto Retrying...")
            else:
                self.ui.pushButton_connect.setText("Connect")
                self.ui.comboBox_connect.setEnabled(True)
                self.ui.pushButton_connect.setEnabled(True)

        elif state == STATE.CONNECTION_LOST:
            self.beepMsg(Message(msgtype=MSGTYPE.WARN, msg="Connected lost from [%s]: %s" % (uri, msg), freq=1200, length=10, repeat=6))
            if self.autoReconnectOn:
                logger.info("Attempting to auto reconnect [autoReconnect = TRUE]")
                self.ui.pushButton_connect.clicked.emit(False)
            else:
                self.ui.pushButton_connect.setText("Connect")
                self.ui.comboBox_connect.setEnabled(True)
                self.ui.pushButton_connect.setEnabled(True)

        else:
            logger.error("Unknown State")

        if state>STATE.GEN_DISCONNECTED:
            self.ui.progressbar_bat.setValue(3000)
            self.ui.progressbar_link.setValue(0)
            self.ui.progressBar_pktIn.setValue(0)
            self.ui.progressBar_pktOut.setValue(0)
            self.ui.pushButton_genRosMsg.setEnabled(False)



    @pyqtSlot(object) # Message
    def beepMsg(self, msg):
        if self.beepOn and msg.beep:
            os.system("beep -f "+str(msg.f)+"-l "+str(msg.l)+" -r "+str(msg.r)+"&")
        if msg.msg != "":
            if msg.type == MSGTYPE.INFO:
                logger.info(msg.msg)
            elif msg.type == MSGTYPE.WARN:
                logger.warn(msg.msg)
            elif msg.type == MSGTYPE.ERROR:
                logger.error(msg.msg)
            elif msg.type == MSGTYPE.DEBUG:
                logger.debug(msg.msg)
            else:
                logger.error("UNKNOWN MESSAGE TYPE: %s", msg.msg)
            self.ui.statusbar.showMessage(msg.msg, 0)
            print msg.msg




class ScannerThread(QThread):
    """ A thread dedicated to scanning the interfaces for crazyflie URIs. """

    sig_foundURI = pyqtSignal(object)
    def __init__(self):
        QThread.__init__(self)
        self.moveToThread(self)

    @pyqtSlot()
    def scan(self):
        self.sig_foundURI.emit(scan_interfaces())