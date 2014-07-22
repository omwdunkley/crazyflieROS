
# need the following
# rosrun tf static_transform_publisher -1.5 0 1 0 0 0 /world /camera_link 10
# rosrun tf static_transform_publisher 0 0 0 0 0 0 /cf /cf_gt 10


__author__ = 'OMWDunkley'
__all__ = ['TrackManager']


from PyQt4 import QtGui, uic
from PyQt4.QtCore import Qt, pyqtSignal, pyqtSlot, QObject, QTimer
from PyQt4.QtGui import QScrollArea, QWidget,  QPushButton

from operator import itemgetter
import math

import rospy
import roslib
roslib.load_manifest("crazyflieROS")
import tf
from tf.transformations import quaternion_from_euler as rpy2quat,quaternion_inverse
from sensor_msgs.msg import Image as ImageMSG
from sensor_msgs.msg import CameraInfo as CamInfoMSG
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import image_geometry

from ui.trackerGUI import Ui_ScrollAreaTracker





class TrackManager(QScrollArea):
    """ External Flie tracking """
    sig_inView = pyqtSignal(bool) # True if in view, else False
    def __init__(self, sub_tf, pub_tf, parent=None):
        super(TrackManager, self).__init__(parent)



        # Initialise gui
        self.ui = Ui_ScrollAreaTracker()
        self.ui.setupUi(self)

        # Ros stuff
        self.sub_tf = sub_tf #tf.TransformListener()
        self.pub_tf = pub_tf #tf.TransformBroadcaster()

        # Initialise Trackers
        self.trackers = [KinectTracker(self), QualisysTracker(self)]
        for t in self.trackers:
            t.sig_started.connect(self.started)
            t.sig_error.connect(self.startFailed)


        # KINECT STUFF
        self.trackers[0].sig_inview.connect(self.sig_inView.emit)
        #self.ui.checkBox_autoCapture

        self.trackers[0].sig_backgroundStep.connect(self.updatePB)
        self.ui.comboBox_kinMethod.currentIndexChanged.connect(self.trackers[0].setMethod)
        self.ui.doubleSpinBox_kinMaxDepth.valueChanged.connect(self.trackers[0].setMaxDepth)
        self.ui.doubleSpinBox_kinFG.valueChanged.connect(self.trackers[0].setForegroundDist)
        self.ui.pushButton_reset.clicked.connect(lambda : self.trackers[0].resetBackground(self.ui.doubleSpinBox_kinObsTime.value()))


        self.ui.doubleSpinBox_kinDetSize.valueChanged.connect(self.trackers[0].setSize)
        self.ui.doubleSpinBox_tol.valueChanged.connect(self.trackers[0].setSizeTolerance)
        self.ui.spinBox_kernelSize.valueChanged.connect(self.trackers[0].setKernel)
        self.ui.spinBox_kernelIter.valueChanged.connect(self.trackers[0].setIterations)
        self.ui.comboBox_prio.currentIndexChanged.connect(self.trackers[0].setPriority)
        self.ui.comboBox_kinDepth.currentIndexChanged.connect(self.trackers[0].setDepthEstMode)

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


    def updatePB(self, val, maxval):
        """ Updates progress bar and hides background options while generating background """

        if self.ui.progressBar_background.maximum()!=maxval:
            self.ui.progressBar_background.setMaximum(maxval)
        self.ui.progressBar_background.setValue(val)
        if val == 0:
            self.ui.doubleSpinBox_kinObsTime.setEnabled(False)
            self.ui.comboBox_kinMethod.setEnabled(False)
            self.ui.doubleSpinBox_kinMaxDepth.setEnabled(False)
            self.ui.doubleSpinBox_kinFG.setEnabled(False)
        elif val==maxval:
            self.ui.doubleSpinBox_kinObsTime.setEnabled(True)
            self.ui.comboBox_kinMethod.setEnabled(True)
            self.ui.doubleSpinBox_kinMaxDepth.setEnabled(True)
            self.ui.doubleSpinBox_kinFG.setEnabled(True)






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
    sig_inview = pyqtSignal(bool) # True if in view, else False

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
    sig_background = pyqtSignal(bool) # 0 = doing background, 1 = done background
    sig_backgroundStep = pyqtSignal(int, int) # step/total

    def __init__(self, parent):
        super(KinectTracker, self).__init__(parent)
        self.name = self.__class__.__name__

        # Ros Stuff
        self.sub_depth = None #Initialised later
        self.bridge = CvBridge()
        self.pub_depth = rospy.Publisher("/camera/detector/image_raw", ImageMSG, queue_size=1)
        self.pub_camera = rospy.Publisher("/camera/detector/camera_info", CamInfoMSG, queue_size=1)


        # Parameters
        self.method = None
        self.setMethod(1)
        # Maximum depth we consider
        self.maxDepth = None
        self.setMaxDepth(m=4.5)
        # How far away we need to be from the background to be considered foreground
        self.bg_thresh = None
        self.setForegroundDist(cm=45)
        # Size of kernel for morphalogical operations
        self.kernel = None
        self.setKernel(2) #x*2+1
        # Number of morphalogical operation iterations
        self.morphIterations = None
        self.setIterations(1)
        # Camera matrix properties #TODO: use numpy and an intrinsic camera matrix to matrix-point multiply
        # FOR NOW USING ROS CAMERA MODEL
        #self.centerX = 319.5
        #self.centerY = 239.5
        #self.depthFocalLength = 525
        # Size restrictions of detected object (metric)
        self.estSize = None
        self.setSize(cm=6)
        self.estSizeTol = None
        self.setSizeTolerance(cm=4)
        self.priority = None # 0: blob size, 1:pixel coord, 2:x, 3:y, 4:z, 5: object width, 6:difference from background
        self.setPriority(0)
        self.depthEstMode = None
        self.setDepthEstMode(0)

        self.goal = [] # x,y,z in camera frame

        # Background History
        self.acc = None
        self.counterSteps = -1
        self.counter = 0
        #self.resetBackground(3.5)


        # Image buffers
        self.depth = None # current depth image [float32]
        self.show = None  # image to display and draw on [rgb8]
        self.method = 0 #0 = median, 1 = minimum

        self.cameraModel = image_geometry.PinholeCameraModel()
        self.cameraInfo = None


    def getCameraGoal(self):
        """ Set the goal in camera coordinates from world coordinates """
        now = rospy.Time.now()
        try:
            self.sub_tf.waitForTransform("/camera_depth_optical_frame","/goal", now, rospy.Duration(0.1))
            (self.goal, rot) = self.sub_tf.lookupTransform("/camera_depth_optical_frame","/goal", now)
            self.goal = list(self.goal)
            #self.pub_tf.sendTransform(self.goal, rot, now, "/goalCam", "/camera_depth_optical_frame")#TODO: debugging only
            return self.goal
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Could not get Goal position in the camera frame')
            pass

    def getGTDist(self):
        now = rospy.Time.now()
        try:
            self.sub_tf.waitForTransform("/camera_depth_optical_frame","/cf_gt", now, rospy.Duration(0.1))
            (t, r) = self.sub_tf.lookupTransform("/camera_depth_optical_frame","/cf_gt", now )
            return list(t)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Could not get ground truth position in the camera frame')
            pass

    def getKinectPoint(self):
        now = rospy.Time.now()
        try:
            self.sub_tf.waitForTransform("/world","/camera_depth_optical_frame", now, rospy.Duration(0.1))
            (t, r) = self.sub_tf.lookupTransform("/world","/camera_depth_optical_frame", now)
            return list(t), list(r)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Could not get ground kinect position in the world')
            pass


    def ros2cv(self, data, dt="passthrough"):
        """ Convert ros image to opencv image """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, dt)
        except CvBridgeError, err:
            msg = "Converting image error: %s" % err
            rospy.logerr(msg)
            self.sig_error.emit(msg)
            self.stop()
        return cv_image

    def onStart(self):
        """ starting = start listening to depth images """
        self.sub_depth = rospy.Subscriber("/camera/depth_registered/image_raw", ImageMSG, self.incomingDepthData)
        self.cameraInfo = rospy.wait_for_message('/camera/depth_registered/camera_info', CamInfoMSG)
        self.cameraModel.fromCameraInfo(self.cameraInfo)
        rospy.loginfo("Camera info received")

    def getFailMsg(self):
        return "No kinect data"

    def getStartMsg(self):
        return "Waiting for Kinect"

    def onStop(self):
       if self.sub_depth is not None:
            self.sub_depth.unregister()


    @pyqtSlot(float)
    def resetBackground(self, secs):
        """ Resets the background """
        self.counterSteps = int(round(secs*30))
        self.counter      = 0
        self.acc = np.zeros((480,640,self.counterSteps), dtype=np.float32)
        self.sig_background.emit(False)
        self.sig_backgroundStep.emit(self.counter, self.counterSteps)

    @pyqtSlot(int)
    def setKernel(self, ksize=3):
        """ Resets the background """
        self.kernel = np.ones((ksize*2+1, ksize*2+1), np.uint8)

    @pyqtSlot(int)
    def setMethod(self, m=0):
        """ Sets the method, 0 = Minimum, 1 = median """
        self.method = m

    @pyqtSlot(int)
    def setPriority(self, prio=0):
        """ Sets prio """
        self.priority = prio

    @pyqtSlot(int)
    def setIterations(self, i=1):
        """ Sets morph iterations """
        self.morphIterations = i

    @pyqtSlot(float)
    def setMaxDepth(self, m=4.5):
        """ Sets max considered depth """
        self.maxDepth = m

    @pyqtSlot(float)
    def setSize(self, cm=6):
        """ Sets estimated object size """
        self.estSize = cm/100.

    @pyqtSlot(float)
    def setSizeTolerance(self, cm=4):
        """ Size tolerance allowed +- """
        self.estSizeTol = cm/100.

    @pyqtSlot(float)
    def setForegroundDist(self, cm=4.5):
        """ Min dist from BG to be considered fg"""
        self.bg_thresh = cm/100.

    @pyqtSlot(float)
    def setDepthEstMode(self, mode=0):
        """ ONly mode 0 implemented: use center of blob """
        self.depthEstMode = mode

    #def projectRay(self, u,v,z):
    #    x = (u - self.centerX) * z / self.depthFocalLength
    #    y = (v - self.centerY) * z / self.depthFocalLength
    #    return [x,y,z]


    def projectRay(self, u,v,d):
        (x,y,z) = self.cameraModel.projectPixelTo3dRay((u,v))
        return [x*d/z,y*d/z,z*d/z]



    def incomingDepthData(self, data):
        self.started()

        t0 = rospy.Time.now()
        img = self.ros2cv(data, dt="passthrough")
        img = np.asarray(img, dtype=np.float32)
        img /= self.maxDepth*1000 # convert to 0-1 units, 1=maxDepth
        img[np.bitwise_not(np.isfinite(img))] = 0
        img[img<0.01] = 1
        img = np.clip(img, 0.0, 1.0)
        img = img.reshape((480,640))

        t1 = rospy.Time.now()

        # Only draw images if we actually display them
        showing = self.pub_depth.get_num_connections()>0 or self.pub_camera.get_num_connections()>0

        # Dont do anything until we first create a background

        if self.counterSteps >0:
            # gather background
            if self.counter < self.counterSteps:
                self.sig_backgroundStep.emit(self.counter, self.counterSteps)
                img[img<0.001] = 1
                self.acc[:, :, self.counter] = img

                if showing:
                    show = np.asarray( cv2.cvtColor(img*255, cv2.COLOR_GRAY2BGR), dtype=np.uint8)
                    cv2.putText(show, str(self.counterSteps-self.counter), (5, 30), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0))
                self.counter += 1

            # create background
            if self.counter == self.counterSteps:



                if self.method==0: # Median
                    med = np.median(self.acc, 2)
                    max = np.max(self.acc, 2)
                    std = np.std(self.acc, 2)

                    self.depth = max
                    for i in range(self.counterSteps):
                        local = self.acc[:, :, i]
                        mask = cv2.absdiff(med, local) < std
                        new_min = cv2.min(self.depth, local)
                        self.depth[mask] = new_min[mask]
                else:
                    min = np.min(self.acc, 2)
                    self.depth = min

                rospy.loginfo("Built background")
                self.sig_backgroundStep.emit(self.counter, self.counterSteps)
                self.sig_background.emit(True)
                self.counter += 1

            # extract foreground
            if self.counter > self.counterSteps:

                t2 = rospy.Time.now()
                img = np.clip(img, 0.0, 1.0)

                # Our BW image
                img = cv2.min(self.depth, img)

                # Binary image of close objects
                d = np.subtract(self.depth, img)
                d = cv2.morphologyEx(d, cv2.MORPH_OPEN, self.kernel, iterations=self.morphIterations)
                bm = cv2.threshold(d, self.bg_thresh/self.maxDepth, 255, cv2.THRESH_BINARY)[1]
                bm = np.asarray(bm, dtype=np.uint8)
                dsts = cv2.distanceTransform(bm, cv2.cv.CV_DIST_L2, 3)

                t3 = rospy.Time.now()

                if showing:
                    pass
                    show = np.asarray( cv2.cvtColor(img*255, cv2.COLOR_GRAY2BGR), dtype=np.uint8)
                    show[:, :, 2] = cv2.max(bm,show[:, :, 2])
                t4 = rospy.Time.now()


                # Detect possible flies
                flies = []
                while(True):
                    # Look for biggest blob
                    mn, dist, mnl, mxl = cv2.minMaxLoc(dsts, mask=bm)

                    if dist<3:
                        break
                    # Extract blob info

                    h, w = img.shape[:2]
                    mask = np.zeros((h+2, w+2), np.uint8)

                    retval, rect = cv2.floodFill(bm, mask, mxl, 0, flags=8 | cv2.FLOODFILL_FIXED_RANGE | cv2.FLOODFILL_MASK_ONLY)

                    # Get pose %TODO should compute average over whole blob
                    if self.depthEstMode == 0:
                        # Center Pixel Depth
                        x, y, z = self.projectRay(mxl[0], mxl[1], self.maxDepth*img[mxl[1], mxl[0]])
                    else:
                        x, y, z = self.projectRay(mxl[0], mxl[1], self.maxDepth*img[mxl[1], mxl[0]])
                        rospy.logerr("Not implemented depth estimation mode [%d], defaulting to center depth", self.depthEstMode)

                    #elif self.depthEstMode == 1:
                    #    # Median depth of blob
                    #elif self.depthEstMode == 2:
                    #    # Closest point of blob


                    # Estimate width
                    xD, yD, zD = self.projectRay(mxl[0]+dist, mxl[1]+dist, self.maxDepth*img[mxl[1], mxl[0]])
                    w = math.sqrt((x-xD)**2 + (y-yD)**2)*2

                    # Estimatea distance from background
                    b_diff = cv2.mean(d, mask=mask[1:-1, 1:-1])[0]*self.maxDepth

                    if abs((w-self.estSize))> self.estSizeTol:#if w<0.02 or w>0.10:
                        if showing:
                            cv2.circle(show, mxl, 1, (0, 200, 0))
                            cv2.circle(show, mxl, int(dist), (120, 0, 0), 1)
                        #cv2.putText(show, str(round(w*100, 1))+"cm", mxl, cv2.FONT_HERSHEY_PLAIN, 1.2, (255,0,0))
                    else:
                        flies.append([dist, mxl, x, y, z, w, b_diff])
                    bm -=  mask[1:-1, 1:-1]*255

                t5 = rospy.Time.now()

                if len(flies)>0:
                    # Sort by distance from background#
                    #TODO

                    if self.priority == 0:
                        # Distance from background
                        flies = sorted(flies, key=itemgetter(6),reverse=True)
                    elif self.priority == 1:
                        # Closest to goal
                        self.getCameraGoal()
                        flies = sorted([f+[(f[2]-self.goal[0])**2+(f[3]-self.goal[1])**2+(f[4]-self.goal[2])**2] for f in flies], key=itemgetter(7),reverse=False)
                    elif self.priority == 2:
                        # Depth
                        flies = sorted(flies, key=itemgetter(4),reverse=False)
                        # Closest to GT
                        #gt = self.getGTDist()
                        #flies = sorted([f+[(f[2]-gt[0])**2+(f[3]-gt[1])**2+(f[4]-gt[2])**2] for f in flies], key=itemgetter(7),reverse=False)

                    elif self.priority == 3:
                        # Estimated object size
                        flies = sorted([f+[abs(self.estSize-f[5])] for f in flies], key=itemgetter(7),reverse=False)
                    elif self.priority == 4:
                        # Blob Size
                        flies = sorted(flies, key=itemgetter(0),reverse=True)
                    else:
                        rospy.logerr("Unknown priority type: %d", self.priority)

                    if len(flies)>0:
                        dist, mxl, x, y, z, w, b_diff = flies[0][0:7]

                        # publish with level rotation
                        t,r, = self.getKinectPoint()
                        #self.pub_tf.sendTransform([z,-x-0.02,-y], (0,0,0,1), rospy.Time.now(), "cf_xyz", "camera_depth_frame")#TODO maybe we need to rotate 90" so x is aligned with optical axis
                        #self.pub_tf.sendTransform([x,y,z], (-r[0],-r[1],-r[2],r[3]), rospy.Time.now(), "cf_xyz", "camera_depth_optical_frame")#TODO maybe we need to rotate 90" so x is aligned with optical axis


                        # Point in world frame with R from crayzflie

                        self.pub_tf.sendTransform([x,y,z], quaternion_inverse(r), rospy.Time.now(), "cf_xyz", "camera_depth_optical_frame")#TODO maybe we need to rotate 90" so x is aligned with optical axis

                    t6 = rospy.Time.now()

                    if showing:
                        pass
                        for i, flie in enumerate(flies):
                            dist, mxl, x, y, z, w, b_diff = flies[i][0:7]
                            if i==0:
                                cv2.circle(show, mxl, int(dist), (0, 255, 0), 4)
                            else:
                                cv2.circle(show, mxl, int(dist), (12, 50, 12), 1)
                            #cv2.circle(show, mxl, 2, (0, 255, 0))
                            #cv2.circle(show, mxl, int(b_diff*10), (255, 0, 0))
                            #cv2.putText(show, str(i),                    (mxl[0]-30, mxl[1]),    cv2.FONT_HERSHEY_PLAIN, 1.4, (0, 255, 255))
                            #cv2.putText(show, str(round(z, 2))+"m",      (mxl[0]+14, mxl[1]+00), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 0))
                            #cv2.putText(show, str(round(w*100, 1))+"cm", (mxl[0]+14, mxl[1]+20), cv2.FONT_HERSHEY_PLAIN, 1.4, (255,0,255))
                            #cv2.putText(show, str(round(b_diff,2))+"m",  (mxl[0]+14, mxl[1]+40), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 0, 0))
                    t7 = rospy.Time.now()

                    rospy.loginfo("%7.1f %7.1f %7.1f %7.1f %7.1f %7.1f  = %10.1f ", 1000.*(t1-t0).to_sec(), 1000.*(t2-t1).to_sec(), 1000.*(t3-t2).to_sec(), 1000.*(t4-t3).to_sec(), 1000.*(t5-t4).to_sec(), 1000.*(t7-t6).to_sec(), 1000*(t7-t0).to_sec())
        else:
            show = np.asarray( cv2.cvtColor(img*255, cv2.COLOR_GRAY2BGR), dtype=np.uint8)



        if showing:
            pass
            try:
                img = self.bridge.cv2_to_imgmsg(show, "bgr8")
                self.cameraInfo.header.stamp = rospy.Time.now()
                img.header = self.cameraInfo.header
                self.pub_camera.publish(self.cameraInfo)
                self.pub_depth.publish(img)
            except CvBridgeError, e:
                rospy.logerr("Image sending problem: %s", e)

        rospy.loginfo(" Total = %10.1f", 1000.*(rospy.Time.now()-t0).to_sec())











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