#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
Attitude indicator widget.
"""

__author__ = 'Bitcraze AB'
__all__ = ['AttitudeIndicator']

import sys
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import pyqtSlot, pyqtSignal, Qt, QPointF,QRectF
from PyQt4.QtGui import QColor, QBrush, QPen, QFont
from cameraInput import VideoPyGame
from commonTools import BAT_STATE, powerToPercentage

class AttitudeIndicator(QtGui.QWidget):
    """Widget for showing attitude"""

    sigDoubleClick = pyqtSignal()


    def __init__(self, parent, hz=30):
        super(AttitudeIndicator, self).__init__(parent)

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.hover = -1
        self.hoverASL = 0.0
        self.hoverTargetASL = 0.0
        self.motors = (0.0,0.0,0.0,0.0)
        self.thrust = 0.0
        self.power = -1
        self.temp = -1
        self.bat = -1
        self.cpu = -1
        self.calib = -1

        self.pixmap = None # Background camera image
        self.needUpdate = True
        self.killswitch = False # killswitch active
        self.recovery = False # recovery mode active

        self.msg = ""
        self.hz = hz

        self.freefall = 0
        self.crashed  = 0
        self.ff_acc   = 0
        self.ff_m     = 0
        self.ff_v     = 0

        self.setMinimumSize(30, 30)
        # self.setMaximumSize(240,240)

        # Update self at 30hz
        self.updateTimer = QtCore.QTimer(self)
        self.updateTimer.timeout.connect(self.updateAI)
        self.updateTimer.start(1000/self.hz)

        self.msgRemove = 0

        # Camera Related Stuff
        self.cam = VideoPyGame(self)
        self.cam.sigPixmap.connect(self.setPixmap)
        self.cam.sigPlaying.connect(self.setVideo)


    def drawModeBox(self, qp, xy, col, txt):
        #qp = QtGui.QPainter()
        qp.save()
        qp.translate(xy)
        qp.setBrush(col.dark())
        qp.setPen(QPen(col, 0.5, Qt.SolidLine))
        qp.setFont(QFont('Monospace', 14, QFont.Monospace))
        rh = 11
        rw = 20
        r=QRectF(-rw,-rh, 2*rw, 2*rh)
        qp.drawRect(r)
        qp.drawText(r, Qt.AlignCenter, txt)
        qp.restore()

    def mouseDoubleClickEvent (self, e):
        self.showFullScreen()

    def contextMenuEvent(self, event):
        menu = QtGui.QMenu(self)

        # Generate menu for ros topics with images
        submenuRos = QtGui.QMenu(menu)
        submenuRos.setTitle("ROS")
        a = submenuRos.addAction("Not Implemented")
        menu.addMenu(submenuRos)

        # Scan /dev/video/X
        submenuVid = QtGui.QMenu(menu)
        submenuVid.setTitle("Camera")

        for i,device in enumerate(self.cam.getDevices()):
            a = submenuVid.addAction(device)
            a.triggered.connect(lambda: self.startCam(device))
        menu.addMenu(submenuVid)


        action = menu.addAction("No Video")
        action.triggered.connect(lambda: self.cam.stop())
        menu.exec_(event.globalPos())

    def startCam(self, device):
        self.cam.start(device=device)

    def startRosImg(self):
        self.cam.stop()




    def setUpdateSpeed(self, hz=30):
        self.hz = hz
        self.updateTimer.setInterval(1000/self.hz)

    def reset(self):
        self.setRollPitchYaw(0,0,0)
        self.setHover(-1)
        self.setPower(-1)
        self.setBattery(-1)
        self.setRoll(0)
        self.setPitch(0)
        self.setYaw(0)
        self.setCPU(-1)
        self.setTemp(-1)
        self.setBaro(-10000) #baro off = <9999
        self.setCalib(-1)
        self.setMotors(0,0,0,0)
        self.setAccZ(1)
        self.msg = ""


    def setCalib(self, calibrated):
        self.calib = calibrated
        self.needUpdate = True

    def updateAI(self):
        if self.msgRemove>0:
            self.msgRemove -= 1
            if self.msgRemove <= 0:
                self.msg = ""
                self.needUpdate = True


        if self.freefall>0:
            self.freefall = self.freefall*5/6 -1
            self.needUpdate = True

        if self.crashed>0:
            self.crashed = self.crashed*5/6 -1
            self.needUpdate = True

        if self.isVisible() and self.needUpdate:
            self.needUpdate = False
            self.repaint()

    def setRoll(self, roll):
        self.roll = roll
        self.needUpdate = True

    def setPitch(self, pitch):
        self.pitch = pitch
        self.needUpdate = True

    def setYaw(self, yaw):
        self.yaw = yaw
        self.needUpdate = True
        
    def setHover(self, target):        
        self.hoverTargetASL = target
        if target>0:
            self.hover = 1 #know we are hovering, target set
        elif target<0:
            self.hover = -1 #dont know
        else:
            self.hover = 0 #know we are not hovering
        self.needUpdate = True
        
    def setBaro(self, asl):
        self.hoverASL = asl
        self.needUpdate = True

    def setRollPitchYaw(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.needUpdate = True

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        self.drawWidget(qp)
        qp.end()

    @pyqtSlot(QtGui.QPixmap)
    def setPixmap(self, pm):
        self.pixmap = pm
        self.needUpdate = True

    @pyqtSlot(bool)
    def setVideo(self, on):
        if not on:
            self.pixmap = None
            self.needUpdate = True

    @pyqtSlot(bool)
    def setKillSwitch(self, on):
        self.killswitch = on
        self.needUpdate = True

    @pyqtSlot()
    def setFreefall(self):
        self.setMsg("Free fall detected!")
        self.freefall = 250
        self.needUpdate = True

    @pyqtSlot(float)
    def setCrash(self, badness=1.):
        """ a bad crash has badnees = 1, severe = badness = 2, and everything between landing softly and a crash between 0 and 1"""
        self.crashed = 128+badness*128
        self.needUpdate = True

    @pyqtSlot(float, float, float)
    def setFFAccMeanVar(self, a, m ,v):
        self.setAccZ(a)
        self.ff_m = m
        self.ff_v = v
        self.needUpdate = True

    @pyqtSlot(float)
    def setAccZ(self, a):
        self.ff_acc = a
        self.needUpdate = True

    def setRecovery(self, on, msg=""):
        """ Set the AUTO caption """
        self.recovery = on
        self.needUpdate = True
        self.setMsg(msg)

    @pyqtSlot(str)
    def setMsg(self, msg, duration=2):
        """ Set a message to display at the bottom of the AI for duration seconds """
        self.msg = msg
        self.msgRemove = duration * self.hz

    @pyqtSlot(float, float, float, float)
    def setMotors(self, m0, m1,m2,m3):
        self.motors = (m2, m1,m0,m3) # f, r, b,l
        self.needUpdate = True

    @pyqtSlot(int)
    def setPower(self, power):
        self.power = power
        self.needUpdate = True

    @pyqtSlot(int)
    def setBattery(self, bat):
        self.bat = bat
        self.needUpdate = True

    @pyqtSlot(float)
    def setCPU(self, cpu):
        self.cpu = cpu
        self.needUpdate = True

    @pyqtSlot(float)
    def setTemp(self, temp):
        self.temp = temp
        self.needUpdate = True


    def drawState(self, qp):
        qp.resetTransform()
        w = self.width()
        h = self.height()



        # USB vs BAT
        posBat = QPointF(w*0.2, 30)
        posUsb = QPointF(w*0.3, 30)
        if self.power == BAT_STATE.BATTERY:
            col = QColor(0,255,0, 200)
            if self.bat > 0:
                col  = QColor(int(255-powerToPercentage(self.bat)/100.*255.),255,0, 200)
            self.drawModeBox(qp, posBat, col, 'BAT')
        elif self.power == BAT_STATE.CHARGING:
            self.drawModeBox(qp, posUsb, QColor(255,255,0, 200), 'USB')
        elif self.power == BAT_STATE.CHARGED:
            self.drawModeBox(qp, posUsb, QColor(0,255,0, 200), 'USB')
        elif self.power == BAT_STATE.LOWPOWER:
            self.drawModeBox(qp, posBat, QColor(0,0,255, 200), 'BAT')
        elif self.power == BAT_STATE.SHUTDOWN: #what is this?
            self.drawModeBox(qp, QPointF(w*0.4, 30), QColor(0,0,0, 128), 'WTF')

        # HOV vs MANUAL # TODO PID
        if self.hover == 0:
            self.drawModeBox(qp, QPointF(w*0.8, 30), QColor(0,255,0, 200), 'MAN')
        elif self.hover >0:
            self.drawModeBox(qp, QPointF(w*0.8, 30), QColor(0,255,0, 200), 'HOV')

        # Calibrated
        if self.calib == 0:
            self.drawModeBox(qp, QPointF(w*0.1, 30), QColor(255,0,0, 200), 'CAL')
            self.setMsg('NOT CALIBRATED',1./2.2)
        elif self.calib == 1:
            self.drawModeBox(qp, QPointF(w*0.1, 30), QColor(0,255,0, 200), 'CAL')

        if self.temp>50:
            self.drawModeBox(qp, QPointF(w*0.7, 30), QColor(255,0,0, 200), 'TMP')
        elif self.temp>40:
            self.drawModeBox(qp, QPointF(w*0.7, 30), QColor(255,255,0, 200), 'TMP')

        if self.cpu>90:
            self.drawModeBox(qp, QPointF(w*0.6, 30), QColor(255,0,0, 200), 'CPU')
        elif self.cpu>80:
            self.drawModeBox(qp, QPointF(w*0.6, 30), QColor(255,255,0, 200), 'CPU')
        # HOV vs MAN vs CTR

    def drawMotors(self, qp):
        # TODO Check if motor update is recent

        defaultCol = QColor(0,255,0, 200)

        #qp = QtGui.QPainter()
        qp.resetTransform()
        w = self.width()
        h = self.height()


        maxSize = min(w,h)*0.1
        minSize = maxSize/10.
        qp.translate(w- maxSize, h-maxSize)
        qp.translate(-10,-10)
        qp.rotate(45)



        lighter = defaultCol
        lighter.setAlphaF(0.1)
        qp.setBrush(lighter.dark())
        qp.setPen(lighter)

        # Draw background circle
        qp.drawEllipse(QPointF(0,0),maxSize,maxSize)

        # Draw Thrust Average
        spread = 2
        avg = sum(self.motors)/len(self.motors) /100. * (maxSize-minSize) + minSize
        lighter.setAlphaF(0.5)
        qp.setPen(lighter.lighter())
        qp.setBrush(QColor(0,0,0,0))
        qp.drawEllipse(QPointF(0,0),avg, avg)

        qp.setBrush(lighter.dark())
        lighter.setAlphaF(0.2)
        qp.setPen(lighter)

        qp.setPen(QPen(defaultCol))
        qp.setBrush(QBrush(defaultCol.dark()))

        for i in range(4):
            m = self.motors[i]*2/100. * (maxSize-minSize) + minSize
            qp.drawPie(QRectF(spread-m/2., spread-m/2., m, m), 0, -90*16)
            qp.rotate(-90)




    def drawWidget(self, qp):
        size = self.size()
        w = size.width()
        h = size.height()

        blue = QtGui.QColor(min(255,0+self.crashed), min(255,61+self.freefall),144, 255 if self.pixmap is None else 64)
        maroon = QtGui.QColor(min(255,59+self.crashed), min(255,41+self.freefall), 39, 255 if self.pixmap is None else 64)


        # Draw background image (camera)
        if self.pixmap is not None:
            qp.drawPixmap(0, 0, w, h, self.pixmap)

        qp.translate(w / 2, h / 2)
        qp.rotate(-self.roll)
        qp.translate(0, (self.pitch * h) / 50)
        qp.translate(-w / 2, -h / 2)
        qp.setRenderHint(qp.Antialiasing)



        font = QtGui.QFont('Serif', 7, QtGui.QFont.Light)
        qp.setFont(font)

        #Draw the blue
        qp.setPen(blue)
        qp.setBrush(blue)
        qp.drawRect(-w, h/2, 3*w, -3*h)

        #Draw the maroon
        qp.setPen(maroon)
        qp.setBrush(maroon)
        qp.drawRect(-w, h/2, 3*w, 3*h)

        pen = QtGui.QPen(QtGui.QColor(255, 255, 255), 1.5,
            QtCore.Qt.SolidLine)
        qp.setPen(pen)
        qp.drawLine(-w, h / 2, 3 * w, h / 2)







        # DRawing Horizon Compass
        labels = ["E", "|", "SE", "|", "S", "|", "SW", "|", "W", "|", "NW", "|", "N", "|", "NE", "|", "E", "|", "SE", "|", "S", "|", "SW", "|", "W"]

        font = QtGui.QFont('Serif', 16, QtGui.QFont.Light)
        qp.setFont(font)

        for i, j in enumerate(range(-540, 585, 45)):
            angle = j / 2 - self.yaw
            label = labels[i]
            qp.drawText(w/2 + w * angle / 180 - 5 * len(label), h/2 + 8, label)

        font = QtGui.QFont('Serif', 7, QtGui.QFont.Light)
        qp.setFont(font)






        # Drawing pitch lines
        for ofset in [-180, 0, 180]:
            for i in range(-900, 900, 25):
                pos = (((i / 10.0) + 25 + ofset) * h / 50.0)
                if i % 100 == 0:
                    length = 0.35 * w
                    if i != 0:
                        if ofset == 0:
                            qp.drawText((w / 2) + (length / 2) + (w * 0.06),
                                        pos, "{}".format(-i / 10))
                            qp.drawText((w / 2) - (length / 2) - (w * 0.08),
                                        pos, "{}".format(-i / 10))
                        else:
                            qp.drawText((w / 2) + (length / 2) + (w * 0.06),
                                        pos, "{}".format(i / 10))
                            qp.drawText((w / 2) - (length / 2) - (w * 0.08),
                                        pos, "{}".format(i / 10))
                elif i % 50 == 0:
                    length = 0.2 * w
                else:
                    length = 0.1 * w

                qp.drawLine((w / 2) - (length / 2), pos,
                            (w / 2) + (length / 2), pos)

        qp.setWorldMatrixEnabled(False)

        pen = QtGui.QPen(QtGui.QColor(0, 0, 0), 2,
            QtCore.Qt.SolidLine)
        qp.setBrush(QtGui.QColor(0, 0, 0))
        qp.setPen(pen)
        qp.drawLine(0, h / 2, w, h / 2)
        
        
        
        # Draw Hover vs Target
        
        qp.setWorldMatrixEnabled(False)
        
        pen = QtGui.QPen(QtGui.QColor(255, 255, 255), 2,
                         QtCore.Qt.SolidLine)
        qp.setBrush(QtGui.QColor(255, 255, 255))
        qp.setPen(pen)
        fh = max(7,h/50)
        font = QtGui.QFont('Sans', fh, QtGui.QFont.Light)
        qp.setFont(font)
        qp.resetTransform()


        qp.translate(0,h/2)
        # not hovering
        if self.hover<=0 and self.hoverASL>-9999:
            qp.drawText(w-fh*10, fh/2, str(round(self.hoverASL,2)))  # asl (center)
               
        
        elif self.hover>0 and self.hoverASL>-9999:
            qp.drawText(w-fh*10, fh/2, str(round(self.hoverTargetASL,2)))  # target asl (center)    
            diff = round(self.hoverASL-self.hoverTargetASL,2)
            pos_y = -h/6*diff
            
            # cap to +- 2.8m
            if diff<-2.8:
                pos_y = -h/6*-2.8
            elif diff>2.8:
                pos_y= -h/6*2.8
            else:
                pos_y = -h/6*diff
            qp.drawText(w-fh*3.8, pos_y+fh/2, str(diff)) # difference from target (moves up and down +- 2.8m)        
            qp.drawLine(w-fh*4.5,0,w-fh*4.5,pos_y) # vertical line     
            qp.drawLine(w-fh*4.7,0,w-fh*4.5,0) # left horizontal line
            qp.drawLine(w-fh*4.2,pos_y,w-fh*4.5,pos_y) #right horizontal line




         # FreeFall Detection
        qp.resetTransform()
        qp.translate(0,h/2)
        qp.drawText(fh*6, fh/2, str(round(self.ff_acc+1,2))+'VG')  # vertical acc

        pos_y = h/6*self.ff_acc

        # cap to +- 2.8m
        if self.ff_acc<-2.8:
            pos_y = -h/6*-2.8
        elif self.ff_acc>2.8:
            pos_y= -h/6*2.8
        else:
            pos_y = -h/6*self.ff_acc
        qp.drawLine(fh*4.5,0,fh*4.5,pos_y) # vertical line
        qp.drawLine(fh*4.7,0,fh*4.5,0) # left horizontal line
        qp.drawLine(fh*4.2,pos_y,fh*4.5,pos_y) #right horizontal line


        # Draw killswitch

        qp.resetTransform()
        if self.killswitch:
            pen = QtGui.QPen(QtGui.QColor(255, 0, 0, 200), 8, QtCore.Qt.SolidLine)
            qp.setBrush(QtGui.QColor(255, 0, 0,200))
            qp.setPen(pen)
            qp.drawLine(w/8., h/8., w/8.*7., h/8.*7.) # vertical line
            qp.drawLine(w/8., h/8.*7., w/8.*7., h/8.) # vertical line


        if self.msg != "":
            qp.drawText(0,0,w,h, QtCore.Qt.AlignBottom | QtCore.Qt.AlignHCenter, self.msg)

        if self.recovery:
            pen = QtGui.QPen(QtGui.QColor(255, 255, 0, 170), 8, QtCore.Qt.SolidLine)
            qp.setBrush(QtGui.QColor(255, 255, 0, 170))
            qp.setPen(pen)
            qp.setFont(QtGui.QFont('Sans', max(7,h/11), QtGui.QFont.DemiBold))
            qp.drawText(0,0,w,h, QtCore.Qt.AlignCenter, 'AUTO')


        self.drawMotors(qp)
        self.drawState(qp)


if __name__ == "__main__":
    class Example(QtGui.QWidget):

        def __init__(self):
            super(Example, self).__init__()

            self.initUI()

        def updatePitch(self, pitch):
            self.wid.setPitch(pitch - 90)

        def updateRoll(self, roll):
            self.wid.setRoll((roll / 10.0) - 180.0)
        
        def updateTarget(self, target):
            self.wid.setHover(500+target/10.)
        def updateBaro(self, asl):
            self.wid.setBaro(500+asl/10.)           
        
        
        def initUI(self):

            vbox = QtGui.QVBoxLayout()

            sld = QtGui.QSlider(QtCore.Qt.Horizontal, self)
            sld.setFocusPolicy(QtCore.Qt.NoFocus)
            sld.setRange(0, 3600)
            sld.setValue(1800)
            vbox.addWidget(sld)
            
            
            self.wid = AttitudeIndicator()

            sld.valueChanged[int].connect(self.updateRoll)
            vbox.addWidget(self.wid)

            hbox = QtGui.QHBoxLayout()
            hbox.addLayout(vbox)

            sldPitch = QtGui.QSlider(QtCore.Qt.Vertical, self)
            sldPitch.setFocusPolicy(QtCore.Qt.NoFocus)
            sldPitch.setRange(0, 180)
            sldPitch.setValue(90)
            sldPitch.valueChanged[int].connect(self.updatePitch)
            hbox.addWidget(sldPitch)
            
            sldASL = QtGui.QSlider(QtCore.Qt.Vertical, self)
            sldASL.setFocusPolicy(QtCore.Qt.NoFocus)
            sldASL.setRange(-200, 200)
            sldASL.setValue(0)
            sldASL.valueChanged[int].connect(self.updateBaro)
            
            sldT = QtGui.QSlider(QtCore.Qt.Vertical, self)
            sldT.setFocusPolicy(QtCore.Qt.NoFocus)
            sldT.setRange(-200, 200)
            sldT.setValue(0)
            sldT.valueChanged[int].connect(self.updateTarget)
            
            hbox.addWidget(sldT)  
            hbox.addWidget(sldASL)
                      

            self.setLayout(hbox)

            self.setGeometry(50, 50, 510, 510)
            self.setWindowTitle('Attitude Indicator')
            self.show()

        def changeValue(self, value):

            self.c.updateBW.emit(value)
            self.wid.repaint()

    def main():

        app = QtGui.QApplication(sys.argv)
        ex = Example()
        sys.exit(app.exec_())


    if __name__ == '__main__':
        main()
