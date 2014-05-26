__author__ = 'ollie'

from PyQt4.QtCore import pyqtSignal
from PyQt4 import QtGui, QtCore

import pygame
import pygame.camera


import logging
logger = logging.getLogger(__name__)


class VideoPyGame(QtCore.QObject):
    # Emits new image
    sigPixmap = pyqtSignal(QtGui.QPixmap)

    # Starting/Stopping
    sigPlaying = pyqtSignal(bool)

    def __init__(self,parent=None):
        super(QtCore.QObject, self).__init__(parent)

        pygame.init() #Do we need this? Or is it done already elsewhere
        pygame.camera.init()

        self.cam = None
        self.buffer = None
        self.size = (1280, 720)


        # Get new image from camera at cam fps
        self.camTimer = QtCore.QTimer(self)
        self.camTimer.timeout.connect(self.emitNextFrame)



    def getDevices(self):
        return pygame.camera.list_cameras()



    def start(self, device, maxSize=(1280,720)):
        #cam = self.getCam()
        if self.camTimer.isActive():
            self.stop()

        logger.info("Using camera: %s", device)

        # Open Camera
        self.cam = pygame.camera.Camera(device, maxSize)

        # Start Camera
        self.cam.start()

        # Get actual capture size
        self.size = self.cam.get_size()

        # Init surface buffer
        self.buffer = pygame.surface.Surface(self.size)
        logger.info("Detected size: %d*%d", self.size[0], self.size[1])

        # Start polling at 50hz
        self.setMaxFPS(fps=50)
        self.sigPlaying.emit(True)

    def stop(self):
        """Start running and polling the camera"""
        # Stop polling camera
        self.camTimer.stop()

        # Turn off camera
        if self.cam is not None:
            self.cam.stop()
            self.sigPlaying.emit(False)

    def setMaxFPS(self, fps=50):
        """ Sets the camera polling rate. How fast images are emitted ultimately depends on the camera."""
        self.camTimer.start(1000/50)

    def emitNextFrame(self):
        """ capture frame and reverse RBG BGR and return opencv image """
        if self.cam.query_image():
            self.buffer = self.cam.get_image(self.buffer)
            data = pygame.image.tostring(self.buffer, "RGBA")
            image = QtGui.QImage(data, self.size[0], self.size[1], QtGui.QImage.Format_ARGB32).rgbSwapped() # This seems to make a copy, it would be good if we could just change the header?
            pixmap = QtGui.QPixmap.fromImage(image)
            self.sigPixmap.emit(pixmap)