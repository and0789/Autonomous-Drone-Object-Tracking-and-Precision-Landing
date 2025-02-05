#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
config.py
---------
File ini berisi konfigurasi umum untuk proyek precision landing, meliputi:
- Koneksi dan sistem (DRONE_CONNECTION_STRING, altitude saat takeoff)
- Parameter landing (ALTITUDE_TARGET, ALTITUDE_TOLERANCE, dsb.)
- Parameter PID
- Konfigurasi kamera
- Logging level
"""

import logging

# --- Koneksi dan Sistem ---
DRONE_CONNECTION_STRING = "udp:127.0.0.1:14550"
# DRONE_CONNECTION_STRING = "/dev/ttyAMA0"
# DRONE_CONNECTION_STRING = "/dev/ttyACM0"

TAKEOFF_ALTITUDE = 10  # meter, ketinggian saat takeoff

# --- Parameter Landing ---
LANDING_TARGET_ALTITUDE = 0.4  # meter, ketinggian target untuk landing
ALTITUDE_TOLERANCE = 0.1       # meter, toleransi ketinggian saat landing
LATERAL_ERROR_THRESHOLD = 20
FINAL_LANDING_THRESHOLD = 0.4

INVERSION_FORWARD = 1   # digunakan untuk vel_x
INVERSION_LATERAL = -1  # digunakan untuk vel_y

# Parameter step descent dan kecepatan descent konstan
DESCENT_STEP = 0.5           # meter, penurunan setiap step
CONSTANT_DESCENT_RATE = 0.3  # meter per detik

# --- Parameter Kontrol Lateral (PID) ---
# PID untuk kontrol maju/mundur (menggunakan error vertikal frame)
PID_FORWARD_KP = 0.005
PID_FORWARD_KI = 0.0
PID_FORWARD_KD = 0.001

# PID untuk kontrol kiri/kanan (menggunakan error horizontal frame)
PID_LATERAL_KP = 0.005
PID_LATERAL_KI = 0.0
PID_LATERAL_KD = 0.001

# Sample time untuk PID (detik)
PID_SAMPLE_TIME = 0.1

# Batas output untuk PID lateral (m/s)
PID_OUTPUT_LIMITS = (-0.5, 0.5)

# --- Konfigurasi Kamera ---
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# --- Logging ---
LOG_LEVEL = logging.INFO
