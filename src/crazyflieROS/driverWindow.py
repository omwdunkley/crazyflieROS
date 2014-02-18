import logging, os, time

from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot, QThread, QObject, QTimer, QSettings, QPoint, QSize, QVariant

from logManager import LogManager
from paramManager import ParamManager
from rosTools import generateRosMessages
from ui.driverGUI import Ui_MainWindow
from cflib.crtp import scan_interfaces, init_drivers, get_interfaces_status
from cflib.crazyflie import Crazyflie

import rospy


logger = logging.getLogger(__name__)

class STATE:
    """ Class to keep track of flie state.
         10 -> 1 -> 2 -> 3 -> 10 -> (12) # Disconnected -> Connected -> Disconnected / loose connection
         10 -> 1 -> 11      # Connection lost while trying to connect
         10 -> 1 -> 2 -> 12 # Connection lost while trying to connect
    """
    # < 0 -> unknown
    UNKNOWN              =-1 # fallback

    # < 0 < 10 -> Connecting or connected
    CONNECTION_REQUESTED = 1 # attempting to connect
    LINK_ESTABLISHED     = 2 # connected, downloading TOC
    CONNECTED            = 3 # connected, TOC downloads

    # >= 9 -> not connected
    GEN_DISCONNECTED     = 9
    DISCONNECTED         = 10 # not connected
    CONNECTION_FAILED    = 11 # Tried to connect but failed
    CONNECTION_LOST      = 12 # Unintentional Disconnect



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

        # Set up ParamManager
        self.paramManager = ParamManager(self.flie.crazyflie, self)
        self.ui.tab_param.layout().addWidget(self.paramManager)
        self.ui.checkBox_pktHZ.toggled.connect(lambda on: self.flie.setPacketUpdateSpeed(self.ui.spinBox_pktHZ.value() if on else 0 ))

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

        self.estimatedMaxPktHz = 100
        self.packetRateHZ = self.ui.spinBox_pktHZ.value()
        self.setPacketRateHZ(self.ui.spinBox_pktHZ.value())

        # Set up URI scanner
        self.scanner = ScannerThread()
        self.scanner.start()


        # Connections from GUI
        self.ui.pushButton_connect.clicked.connect(lambda : self.connectPressed(self.ui.comboBox_connect.currentText())) # Start button -> connect
        self.ui.comboBox_connect.currentIndexChanged.connect(self.uriSelected)
        self.ui.spinBox_pktHZ.valueChanged.connect(self.setPacketRateHZ) #TODO: disable elements if its 0

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

        # Intiate an initial Scan
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


    @pyqtSlot(int)
    def setPacketRateHZ(self, hz):
        """ Update our window for measuring packets / second and update the progress bars max values) """
        # Update the progress bars to the estimated packets per meaasurement (depends on hz)
        self.packetRateHZ = hz
        self.flie.setPacketUpdateSpeed(hz)
        m = self.estimatedMaxPktHz/hz
        self.ui.progressBar_pktIn.setMaximum(m)
        self.ui.progressBar_pktOut.setMaximum(m)



    @pyqtSlot(int,int)
    def updatePacketRate(self, inHZ, outHZ):
        """ Updates the two packet rate progress bars """
        #logger.info("In: %d | Out: %d", inHZ, outHZ)

        m = max(inHZ,outHZ)
        if m>self.ui.progressBar_pktIn.maximum():
            self.estimatedMaxPktHz = m*self.packetRateHZ
            mn = self.estimatedMaxPktHz/self.packetRateHZ
            logger.warn("Maximum Packets Changed from %d/s -> %d/s", self.ui.progressBar_pktIn.maximum()/self.packetRateHZ, mn)
            self.ui.progressBar_pktIn.setMaximum(mn)
            self.ui.progressBar_pktOut.setMaximum(mn)

        self.ui.progressBar_pktIn.setValue(inHZ)
        self.ui.progressBar_pktOut.setValue(outHZ)


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




class FlieControl(QObject):
    """ Class that andles the flie library """

    sig_console = pyqtSignal(str)     # Console messages from the flie - emiited for every console message
    sig_packetSpeed = pyqtSignal(int, int) # Packets in/out per second - emitted every self.updatePacketSpeed ms
    sig_stateUpdate = pyqtSignal(int,str,str) # Send state update and optional messages (stateNr, uri, errmsg)


    sig_flieLink = pyqtSignal(int)
    #sig_flieBattery = pyqtSignal()

    def __init__(self):
        super(FlieControl, self).__init__()

        # Temporary
        #cache_dir = os.path.dirname(os.path.realpath(__file__))
        #cache_dir =  cache_dir[0:cache_dir.find("src/crazyflieROS")]+"cache/"

        # Parameters
        self.updatePacketSpeed = 5# Window length in hz of packet rate estimation

        # Members
        self.console_cache = "" # Used to buffer crazyflie console messages that go over newlines
        self.crazyflie = Crazyflie()
        #self.crazyflie = Crazyflie(cache_dir+"/ro", cache_dir+"/rw")
        self.linkQuality = LinkQuality(window=50)
        self.counterPacketIn  = 0 # Counts packets within self.updatePacketSpeed window
        self.counterPacketOut = 0 # Counts packets within self.updatePacketSpeed window
        self.status = STATE.DISCONNECTED

        # Timers
        self.timerPacket = QTimer(self)
        self.timerPacket.timeout.connect(self.updatePacketCount)


        # Callbacks
        self.crazyflie.connected.add_callback(self.connectedCB)                      # Called when the link is established and the TOCs (that are not cached) have been downloaded
        self.crazyflie.disconnected.add_callback(self.disconnectedCB)                # Called on disconnect, no matter the reason
        self.crazyflie.connection_lost.add_callback(self.connectionLostCB)           # Called on unintentional disconnect only
        self.crazyflie.connection_failed.add_callback(self.connectionFailedCB)       # Called if establishing of the link fails (i.e times out)
        self.crazyflie.connection_requested.add_callback(self.connectionRequestedCB) # Called when the user requests a connection
        self.crazyflie.link_established.add_callback(self.linkEstablishedCB)         # Called when the first packet in a new link is received
        self.crazyflie.link_quality_updated.add_callback(self.linkQualityCB)         # Called when the link driver updates the link quality measurement
        self.crazyflie.packet_received.add_callback(self.packetReceived)             # Called for every packet received
        self.crazyflie.packet_sent.add_callback(self.packetSent)                     # Called for every packet sent
        self.crazyflie.console.receivedChar.add_callback(self.consoleCB)             # Called with console text


    ### SET PARAMS
    @pyqtSlot(int)
    def setPacketUpdateSpeed(self, hz):
        """ Sets the rate at which packet in/out rates are estimated. This defines the window length in hz
            Adds / Removes the packet update callbacks """
        if hz <= 0:
            # Turned off: stop timer, remove callbacks
            self.timerPacket.stop()
            #self.crazyflie.packet_received.remove_callback(self.packetReceived)
            #self.crazyflie.packet_sent.remove_callback(self.packetSent)
            logger.info("Stopped estimating packet rates")
        else:
            # Update timer and if turning off->on add callbacks
            if self.updatePacketSpeed<=0:
                # Turned on as it was off: add callbacks
                #self.crazyflie.packet_received.add_callback(self.packetReceived)
                #self.crazyflie.packet_sent.add_callback(self.packetSent)
                logger.info("Started estimating packet rates")

            # This makes sure we only start the timer if we supply the function with the same hz,
            # or we only change the value if its running already
            if self.timerPacket.isActive() or self.updatePacketSpeed == hz:
                self.updatePacketSpeed = hz
                self.timerPacket.start(1000./self.updatePacketSpeed)
            self.updatePacketSpeed = hz
            logger.info("Packet rate estimation window set to %.1fms", 1000/self.updatePacketSpeed)


    ### TIMER CALLBACKS
    def updatePacketCount(self):
        """ Counts packets per second going coming in """
        inHz  = self.counterPacketIn/self.updatePacketSpeed
        outHz = self.counterPacketOut/self.updatePacketSpeed
        self.counterPacketIn = 0
        self.counterPacketOut = 0
        self.sig_packetSpeed.emit(round(inHz), round(outHz))


    ### CRAZYFLIE CALLBACKS

    def connectedCB(self, uri, msg=""):
        """ Called when the link is established and the TOCs (that are not cached) have been downloaded """
        self.sig_stateUpdate.emit(STATE.CONNECTED, uri, msg)


    def disconnectedCB(self, uri, msg=""):
        """ Called on disconnect, no matter the reason """
        # stop counting packets
        self.setPacketUpdateSpeed(0)
        self.sig_stateUpdate.emit(STATE.DISCONNECTED, uri, msg)


    def connectionLostCB(self, uri, msg=""):
        """ Called on unintentional disconnect only """
        self.sig_stateUpdate.emit(STATE.CONNECTION_LOST, uri, msg)
        # TODO: try to reconnect?


    def connectionFailedCB(self, uri, msg=""):
        """ Called if establishing of the link fails (i.e times out) """
        # stop counting packets
        self.setPacketUpdateSpeed(0)
        self.sig_stateUpdate.emit(STATE.CONNECTION_FAILED, uri, msg)


    def connectionRequestedCB(self, uri, msg=""):
        """ Called when the user requests a connection """
        # Start counting packets
        self.setPacketUpdateSpeed(self.updatePacketSpeed)
        self.sig_stateUpdate.emit(STATE.CONNECTION_REQUESTED, uri, msg)


    def linkEstablishedCB(self, uri, msg=""):
        """ Called when the first packet in a new link is received """
        self.sig_stateUpdate.emit(STATE.LINK_ESTABLISHED, uri, msg)
        # TODO: set up logging


    def linkQualityCB(self, percentage):
        """ Called when the link driver updates the link quality measurement """
        #q = self.linkQuality.addMeasurementMin(percentage)
        #q = self.linkQuality.addMeasurementAvg(percentage)
        q = self.linkQuality.addMeasurementCount(percentage)
        if q is not None:
            self.sig_flieLink.emit(percentage) # TODO measure how fast this happens, cap at 30hz or so


    def packetReceived(self, pk=None):
        """ Called for every packet received """
        self.counterPacketIn += 1


    def packetSent(self, pk=None):
        """ Called for every packet sent """
        self.counterPacketOut += 1


    def consoleCB(self, msg):
        """ Crazyflie console messages are routed to this function. Newlines are detected and the strings joined. """
        # Join messages if they are max length
        if len(msg)==30:
            self.console_cache += msg
        else:            
            #CSI = "\x1b["
            #cyan = CSI+"36m"
            #reset = CSI+"m"
            #msg = cyan+(self.console_cache+msg).strip("\n")+reset
            msg = (self.console_cache+msg).strip("\n")+"\n"
            logger.info(msg)
            self.sig_console.emit(msg)
            self.console_cache = ""



    ### LOG CALLBACKS


    ### OUTGOING

    def sendCmd(self, cmd):
        """ Send the flie a control command
        """
        pass

    def sendParam(self, param):
        """ Send the flie an updated parameter
        """
        pass

    def setupLogging(self):
        """ Send the flie a logging request
        """
        pass

    def getLogToC(self):
        return self.crazyflie.log._toc.toc if self.crazyflie.log._toc else None

    ### USER INITIATED

    @pyqtSlot(str)
    def requestConnect(self, uri):
        """ Request connection to the flie """
        logger.info("Requesting connection to [%s]", uri)
        self.crazyflie.open_link(uri)

    @pyqtSlot()
    def requestDisconnect(self):
        """ Request shutdown to flie """
        logger.info("Requesting disconnect")
        self.crazyflie.close_link()





class LinkQuality:
    def __init__(self,window=10):
        self.w = window # window
        self.c = 0 # counter
        self.acc = 0 #accumulator

    def addMeasurementAvg(self, m):
        self.c +=1
        self.acc += m
        if self.w == self.c:
            r = float(self.acc)/self.c
            self.acc = 0
            self.c = 0
            return round(r)
        return None

    def addMeasurementMin(self, m):
        self.c +=1
        self.acc = min(self.acc, m)
        if self.w == self.c:
            r = self.acc
            self.acc = 100
            self.c = 0
            return r
        return None

    def addMeasurementCount(self, m):
        self.c +=1
        self.acc += 1 if m==100 else 0
        if self.w == self.c:
            r = 100. * self.acc / self.c
            self.acc = 0
            self.c = 0
            return round(r)
        return None






class ROSNode(QObject):
     def __init__(self):
         super(ROSNode, self).__init__()
         #rospy.init_node('CrazyflieDriver')






class ScannerThread(QThread):
    """ A thread dedicated to scanning the interfaces for crazyflie URIs. """

    sig_foundURI = pyqtSignal(object)
    def __init__(self):
        QThread.__init__(self)
        self.moveToThread(self)

    @pyqtSlot()
    def scan(self):
        self.sig_foundURI.emit(scan_interfaces())


















