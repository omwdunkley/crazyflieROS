from PySide.QtGui import QScrollArea


__author__ = 'OMWDunkley'
__all__ = ['ParamManager']


from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot, QObject, QTimer
from PyQt4.QtGui import   QScrollArea, QWidget,  QPushButton

import rospy
import roslib
roslib.load_manifest("crazyflieROS")
import tf
from sensor_msgs.msg import Image as ImageMSG

from ui.trackerGUI import Ui_ScrollAreaTracker





class TrackManager(QScrollArea):
    """ External Flie tracking """
    def __init__(self, parent=None):
        super(TrackManager, self).__init__(parent)

        # Initialise gui
        self.ui = Ui_ScrollAreaTracker()
        self.ui.setupUi(self)

        # Ros stuff
        self.sub_tf = tf.TransformListener()
        self.pub_tf = tf.TransformBroadcaster()

        # Initialise Trackers
        self.trackers = [KinectTracker(self), QualisysTracker(self)]
        for t in self.trackers:
            t.sig_started.connect(self.started)
            t.sig_error.connect(self.startFailed)


        self.tracker = None
        self.tracking = False

        # Connections GUI to GUI
        self.ui.comboBox_tracker.clear()
        self.ui.comboBox_tracker.addItems([t.name for t in self.trackers])
        self.ui.comboBox_tracker.currentIndexChanged.connect(self.switchTracker)
        self.ui.comboBox_tracker.currentIndexChanged.emit(self.ui.comboBox_tracker.currentIndex()) # Force update

        self.readSettings()

        # Connections from GUI
        self.q01 = (self.ui.comboBox_q0.currentIndex(), self.ui.comboBox_q1.currentIndex())
        self.ui.pushButton_trackStart.clicked.connect(self.startStopClicked)
        self.ui.comboBox_q0.currentIndexChanged.connect(lambda x: self.qualisysFrameChanged(0,x))
        self.ui.comboBox_q1.currentIndexChanged.connect(lambda x: self.qualisysFrameChanged(1,x))





    def qualisysFrameChanged(self, frame, id):
        """ Make sure q1 and q0 do not have the same index. Swaps their previous values if they do
        """
        if self.ui.comboBox_q0.currentIndex() == self.ui.comboBox_q1.currentIndex():
            if frame == 0:
                self.ui.comboBox_q1.setCurrentIndex(self.q01[0])
            else:
                self.ui.comboBox_q0.setCurrentIndex(self.q01[1])
        self.q01 = (self.ui.comboBox_q0.currentIndex(), self.ui.comboBox_q1.currentIndex())





    @pyqtSlot()
    def startStopClicked(self):
        if self.tracking:
            self.stop()
        else:
            self.start()


    def start(self):
        self.ui.pushButton_trackStart.setText(self.tracker.getStartMsg())
        self.ui.pushButton_trackStart.setEnabled(False)
        self.ui.comboBox_tracker.setEnabled(False)
        self.repaint()
        self.tracking = True
        self.tracker.start()

    def stop(self):
        self.ui.pushButton_trackStart.setEnabled(True)
        self.ui.comboBox_tracker.setEnabled(True)
        self.ui.pushButton_trackStart.setText("Start Tracking")
        self.tracker.stop()
        self.tracking = False

    def started(self):
        self.ui.pushButton_trackStart.setEnabled(True)
        self.ui.pushButton_trackStart.setText("Stop Tracking")

    def startFailed(self, msg):
        self.ui.pushButton_trackStart.setEnabled(True)
        self.ui.comboBox_tracker.setEnabled(True)
        self.ui.pushButton_trackStart.setText("Start [%s]"%msg)
        self.tracking = False




    def readSettings(self):
        # TODO Read Configuration
        pass
    def writeSettings(self):
        # TODO save configuration
        pass


    @pyqtSlot(int)
    def switchTracker(self, idx=0):
        """ Changes current tracker. Only enabled if not tracking"""
        self.ui.pushButton_trackStart.setText("Start Tracking")

        self.tracker = self.trackers[idx]
        self.ui.groupBox_kinectFG.hide()
        self.ui.groupBox_kinectBG.hide()
        self.ui.groupBox_qualisys.hide()
        self.ui.groupBox_slam.hide()

        if idx == 0:
            self.ui.groupBox_kinectFG.show()
            self.ui.groupBox_kinectBG.show()
        elif idx == 1:
            self.ui.groupBox_qualisys.show()
        elif idx == 3:
            self.ui.groupBox_slam.show()
        else:
            rospy.logerr("Unknown Tracker Type Selected: %d", idx)





class Tracker(QObject):
    sig_error = pyqtSignal(str)
    sig_started = pyqtSignal()

    def __init__(self, parent):
        super(Tracker, self).__init__(parent)
        self.name = self.__class__.__name__

        # Ros Stuff
        self.sub_tf = parent.sub_tf
        self.pub_tf = parent.pub_tf

        # Timer that checks if we have failed to start
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.failed)
        self.timeout = 2000 #ms
        self.timer.setInterval(self.timeout)

    def started(self):
        """ Called when starting was possible """
        if self.timer.isActive():
            self.timer.stop()
            self.sig_started.emit()
            rospy.loginfo("Started [%s]" , self.name)



    def onStart(self):
        pass
    def getStartMsg(self):
        return "Default start msg.."
    def getFailMsg(self):
        return "Default fail msg"
    def onStop(self):
        pass


    def start(self):
        rospy.loginfo("Starting [%s]", self.name)
        self.timer.start()
        return self.onStart()

    def failed(self, msg=None):
        """ Called when starting not possible """
        if msg is None:
            msg = self.getFailMsg()
        self.sig_error.emit(msg)
        rospy.logerr("Failed to start [%s]: %s" , self.name, msg)
        self.stop()

    def stop(self):
        self.timer.stop()
        rospy.loginfo("Stopping [%s]", self.name)
        self.onStop()










class KinectTracker(Tracker):
    """ Listens to kinect images,
        builds a background, detects flie as foreground,
        computes location in 3d, sends out TF
    """

    def __init__(self, parent):
        super(KinectTracker, self).__init__(parent)
        self.name = self.__class__.__name__

        # Ros Stuff
        self.sub_depth = None
        self.pub_depth = rospy.Publisher("/camera/detector", ImageMSG)

    def onStart(self):
        self.sub_depth = rospy.Subscriber("/camera/depth_registered/image_raw", ImageMSG, self.incomingDepthData)

    def getFailMsg(self):
        return "No kinect data"
    def getStartMsg(self):
        return "Waiting for Kinect"


    def onStop(self):
       if self.sub_depth is not None:
            self.sub_depth.unregister()

    def incomingDepthData(self, data):
        self.started()




class QualisysTracker(Tracker):
    """ Listens to Qualisys transforms, sends out TF
    """
    def __init__(self, parent):
        super(QualisysTracker, self).__init__(parent)
        self.name = self.__class__.__name__


    def onStart(self):
        try:
            self.sub_tf.waitForTransform("/camera_depth_frame", "/camera_depth_optical_frame", rospy.Time.now(), rospy.Duration((self.timeout*0.9)/1000))
            #self.sub_tf.waitForTransform("/world", "/Q0", rospy.Time.now(), rospy.Duration((self.timeout*0.9)/1000))
        except:
            self.failed("No /Q0 Frame")
            return
        self.started()

    def onStop(self):
       pass