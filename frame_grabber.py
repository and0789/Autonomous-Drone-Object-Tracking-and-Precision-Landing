#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
frame_grabber.py
----------------
Modul ini menyediakan kelas FrameGrabber yang bertugas mengambil frame dari kamera secara terus-menerus menggunakan threading.
"""

import threading
import cv2

class FrameGrabber:
    def __init__(self, camera):
        self.camera = camera
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            frame = self.camera.capture_array()
            if frame is not None:
                with self.lock:
                    self.frame = frame.copy()

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
