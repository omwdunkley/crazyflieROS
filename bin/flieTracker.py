#!/usr/bin/env python
#ROS stuff

import numpy as np
import cv2
import cv2.cv as cv

import roslib;
roslib.load_manifest('crazyflieROS')
import rospy
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge, CvBridgeError
from operator import itemgetter

import tf
from tf.transformations import quaternion_from_euler as rpy2quat

import IPython
import time, math, sys, os

from scipy import matrix




WIN_RGB   = "RGB Window"

yaw = 0*math.pi # Crazyflie yaw to align crazyflie x axis with world camera x axis



    




class Localiser:
    """Definition
    """
    def __init__(self):
        
        rospy.init_node('flieLocaliser') 
        
        self.maxDepth = 4.5
        self.bg_thresh = 0.45
        self.kernel = np.ones((3, 3), np.uint8)
        self.centerX = 319.5
        self.centerY = 239.5
        self.depthFocalLength = 525

        # CV stuff
        self.bridge = CvBridge()

           
        # Subscribe
        #self.sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", ImageMSG, self.new_rgb_data)
        self.sub_depth = rospy.Subscriber("/camera/depth_registered/image_raw", ImageMSG, self.new_depth_data)
        self.pub_depth = rospy.Publisher("/camera/detector", ImageMSG)
        self.sub_tf = tf.TransformListener()

        # Publish
        self.pub_tf = tf.TransformBroadcaster()

        # Members
        self.depth = None
        self.show = None
        self.counter = 30*4

        self.acc = np.zeros((480,640,self.counter), dtype=np.float32)


        
        # Run ROS node
        rospy.loginfo('Node Started')
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down"
            cv2.destroyAllWindows()


    def projectRay(self, u,v,z):
        x = (u - self.centerX) * z / self.depthFocalLength
        y = (v - self.centerY) * z / self.depthFocalLength
        return [x,y,z]


    def ros2cv(self, data, dt="mono8"):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, dt)
        except CvBridgeError, err:
            rospy.logerr("Converting image error: %s", err)
            rospy.signal_shutdown("Converting image error: "+ str(err))
        return cv_image


    def new_rgb_data(self, data):
        img = self.ros2cv(data, dt="bgr8")
        #self.showImage(WIN_RGB, img)


    def new_depth_data(self, data):
        img = self.ros2cv(data, dt="passthrough")
        img = np.asarray(img, dtype=np.float32)
        img /= self.maxDepth*1000
        img[np.isnan(img)] = 0
        img[np.isinf(img)] = 0
        img[img<0.01] = 1
        np.clip(img, 0.0, 1.0)
        img = img.reshape((480,640))

        showing = self.pub_depth.get_num_connections()>0

        # gather background
        if self.counter > 0:
            img[img<0.001] = 1
            self.counter -= 1
            self.acc[:, :, self.counter] = img

            if showing:
                show = np.asarray( cv2.cvtColor(img*255, cv2.COLOR_GRAY2BGR), dtype=np.uint8)
                cv2.putText(show, str(self.counter), (5, 30), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0))

        # create background
        if self.counter == 0:
            self.counter -= 1
            rospy.loginfo("Building background")

            if False: # Median
                med = np.median(self.acc, 2)
                max = np.max(self.acc, 2)
                std = np.std(self.acc, 2)

                self.depth = max
                for i in range(30*4):
                    local = self.acc[:, :, i]
                    mask = cv2.absdiff(med, local) < std
                    new_min = cv2.min(self.depth, local)
                    self.depth[mask] = new_min[mask]
            else:
                min = np.min(self.acc, 2)
                self.depth = min

        # extract foreground
        if self.counter < 0:

            np.clip(img, 0.0, 1.0)
            # Our BW image
            img = cv2.min(self.depth, img)

            # Output image



            # Binary image of close objects
            d = self.depth-img
            #d = cv2.morphologyEx(d, cv2.MORPH_DILATE, self.kernel, iterations=1)
            d = cv2.morphologyEx(d, cv2.MORPH_OPEN, self.kernel, iterations=1)
            bm = cv2.threshold(d, self.bg_thresh/self.maxDepth, 255, cv2.THRESH_BINARY)[1]
            bm = np.asarray(bm, dtype=np.uint8)
            dsts = cv2.distanceTransform(bm, cv2.cv.CV_DIST_L2, 3)


            if showing:
                show = np.asarray( cv2.cvtColor(img*255, cv2.COLOR_GRAY2BGR), dtype=np.uint8)
                show[:, :, 2] = cv2.max(bm,show[:, :, 2])


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
                x, y, z = projectRay(mxl[0], mxl[1], self.maxDepth*img[mxl[1], mxl[0]])

                # Estimate width
                xD, yD, zD = projectRay(mxl[0]+dist, mxl[1]+dist, self.maxDepth*img[mxl[1], mxl[0]])
                w = math.sqrt((x-xD)**2 + (y-yD)**2)*2

                # Estimatea distance from background
                b_diff = cv2.mean(d, mask=mask[1:-1, 1:-1])[0]*self.maxDepth

                if w<0.02 or w>0.10:
                    if showing:
                        cv2.circle(show, mxl, 1, (0, 200, 0))
                        cv2.circle(show, mxl, int(dist), (120, 0, 0), 1)
                    #cv2.putText(show, str(round(w*100, 1))+"cm", mxl, cv2.FONT_HERSHEY_PLAIN, 1.2, (255,0,0))
                else:
                    flies.append((dist, mxl, x, y, z, w, b_diff))
                bm -=  mask[1:-1, 1:-1]*255



            # Sort by distance from background
            flies = sorted(flies, key=itemgetter(6),reverse=True)


            if len(flies)>0:
                dist, mxl, x, y, z, w, b_diff = flies[0]
                self.pub_tf.sendTransform([z,-x-0.02,-y], rpy2quat(0,0,yaw), rospy.Time.now(), "cf_xyz", "camera_depth_frame")

            if showing:
                for i, flie in enumerate(flies):
                    dist, mxl, x, y, z, w, b_diff = flies[i]
                    if i==0:
                        cv2.circle(show, mxl, int(dist), (0, 255, 0), 4)
                    else:
                        cv2.circle(show, mxl, int(dist), (12, 50, 12), 1)

                    cv2.circle(show, mxl, 2, (0, 255, 0))
                    cv2.circle(show, mxl, int(b_diff*10), (255, 0, 0))
                    cv2.putText(show, str(i),                    (mxl[0]-30, mxl[1]),    cv2.FONT_HERSHEY_PLAIN, 1.4, (0, 255, 255))
                    cv2.putText(show, str(round(z, 2))+"m",      (mxl[0]+14, mxl[1]+00), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 0))
                    cv2.putText(show, str(round(w*100, 1))+"cm", (mxl[0]+14, mxl[1]+20), cv2.FONT_HERSHEY_PLAIN, 1.4, (255,0,255))
                    cv2.putText(show, str(round(b_diff,2))+"m",  (mxl[0]+14, mxl[1]+40), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 0, 0))


        if showing:
            try:
                self.pub_depth.publish(self.bridge.cv2_to_imgmsg(show, "bgr8"))
            except CvBridgeError, e:
                rospy.logerr("Image sending problem: %s", e)


def run(args=None):
    Localiser()
    


if __name__ == "__main__":   
    run(sys.argv)
