#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kalman_filter.py
----------------
Modul ini menyediakan kelas KalmanFilter2D untuk memprediksi dan menyaring
posisi objek menggunakan filter Kalman sederhana.
"""

import numpy as np

class KalmanFilter2D:
    def __init__(self, dt=0.1):
        self.dt = dt
        # Vektor state: [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        # Matriks transisi state
        self.A = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1,  0],
                           [0, 0, 0,  1]])
        # Matriks pengukuran: kita hanya mengukur posisi (x, y)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        # Matriks kovarians error
        self.P = np.eye(4) * 1000.0
        self.Q = np.eye(4) * 0.01   # noise proses
        self.R = np.eye(2) * 5.0    # noise pengukuran
        self.initialized = False

    def predict(self):
        self.x = self.A.dot(self.x)
        self.P = self.A.dot(self.P).dot(self.A.T) + self.Q
        return self.x

    def update(self, z):
        y = z - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(self.H.T) + self.R
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        I = np.eye(self.A.shape[0])
        self.P = (I - K.dot(self.H)).dot(self.P)
        return self.x

    def step(self, z):
        if not self.initialized:
            # Inisialisasi state dengan pengukuran awal
            self.x[0, 0] = z[0, 0]
            self.x[1, 0] = z[1, 0]
            self.initialized = True
        self.predict()
        self.update(z)
        return self.x
