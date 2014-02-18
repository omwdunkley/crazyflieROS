__author__ = 'ollie'
__all__=['FlieControl','STATE']


from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot, QObject, QTimer
from cflib.crazyflie import Crazyflie
import logging
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
