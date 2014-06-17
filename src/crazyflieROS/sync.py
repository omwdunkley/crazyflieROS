#!/usr/bin/env python
# -*- coding: utf-8 -*-
from ctypes import c_float

__author__ = 'maxson'
__all__ = ['Sync']

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
import struct
from cflib.crazyflie import Crazyflie, State
import time
import rospy
from PyQt4.QtCore import pyqtSignal, QObject, QTimer
from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import pyqtSlot, pyqtSignal,QTime
import numpy


class Sync(QObject):
    """
    Used for synchronize base station and crazyflie and to estimate delay
    """

    sig_delayUp = pyqtSignal(float)
    sig_delayDown = pyqtSignal(float)
    sig_cpuTime = pyqtSignal(float)
    sig_flieTime = pyqtSignal(float)
    sig_diffTime = pyqtSignal(float)

    def __init__(self, crazyflie):
        QObject.__init__(self)
        self.cf = crazyflie
        self.cf.add_port_callback(CRTPPort.SYNC, self._new_packet_cb)
        self.dif = 0
        self.cf_time = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_synchronisation)
        self.timer.setInterval(1000/5) #5hz
        self.initialized = False
        self.counter = 0
        self.error = 0.
        self.cpuTime = 0.
        self.flieTime = 0.
        self.t0 = rospy.Time.now().to_nsec()

    @pyqtSlot(int)
    def setSyncRate(self,hz):
        if hz==0:
            rospy.loginfo('Sync Timer Turned Off')
            self.enable(False)
        else:
            rospy.loginfo('Sync Timer Freq updated %f', hz)
            self.timer.setInterval(1000./hz)



    def enable(self, on=True):
        if on:
            self.timer.start()
        else:
            self.timer.stop()
            self.sig_delayDown.emit(0)
            self.sig_delayUp.emit(0)



    def send_synchronisation(self):
        if self.cf.state == State.CONNECTED:
            pk = CRTPPacket()
            pk.port = CRTPPort.SYNC
            t = rospy.Time.now().to_nsec()
            pk.data = struct.pack('<Q',t) # timestamp in m
            self.cf.send_packet(pk)


    def _new_packet_cb(self, packet):
        self.counter += 1
        nowTime = rospy.Time.now().to_nsec()
        sendTime = struct.unpack('<Q', packet.data[0:8])[0]
        flieTime = struct.unpack('<Q', packet.data[8:16])[0]*1000 #Q = uint64, crazyflie time converted to nS
        delta = (nowTime - sendTime)/2 # ns assuming symmetric timing for up and down


        self.flieTime = flieTime - delta
        self.cpuTime = nowTime


        self.sig_cpuTime.emit(round((self.cpuTime-self.t0)/1e9 , 3))
        self.sig_flieTime.emit(round(self.flieTime/1e9, 3))
        self.sig_diffTime.emit(abs(round((self.flieTime-(self.cpuTime-self.t0))/1e9, 3)))






        down = flieTime - self.getCrazyflieTime()
        up = nowTime - sendTime - down

        #self.sig_delay.emit(up, down)#(delta/1000., delta/1000.)
        self.sig_delayDown.emit(down/1e6)
        self.sig_delayUp.emit(up/1e6)
        #print "up", up/1e6, "ms  down", down/1e6,"ms  delay", delta*2/1e6, "ms"


        self.sig_timeUpdate.emit(flieTime-delta)

        #if abs(flieTime/1000. - (delta/1000.) - self.getCrazyflieTime()) < 500:
        #    self.error += abs(flieTime/1000. - (delta/1000.) - self.getCrazyflieTime())


    def nonSymmetricDelay(self, departureTime, arrivalTime, cfTime): #all in ms
        down = self.getCrazyflieTime() - cfTime
        up = arrivalTime - departureTime - down
        return up, down

    def getCrazyflieTime(self):
        return self.flieTime + rospy.Time.now().to_nsec() - self.cpuTime #all in ns
