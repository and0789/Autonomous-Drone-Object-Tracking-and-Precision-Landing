#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
config.py
---------
File ini berisi konfigurasi umum untuk proyek precision landing, meliputi:
- Koneksi dan sistem (DRONE_CONNECTION_STRING, TAKEOFF_ALTITUDE)
- Mode deteksi tracking (TRACKING_MODE)
- Parameter landing (LANDING_TARGET_ALTITUDE, ALTITUDE_TOLERANCE, dsb.)
- Konstanta untuk logika tracking dan landing (CENTERED_TIME_THRESHOLD, SCALING_FACTOR, dsb.)
- Parameter PID untuk kontrol pergerakan drone
- Konfigurasi tampilan kamera (CAMERA_WIDTH, CAMERA_HEIGHT)
- Pengaturan logging (LOG_LEVEL)
"""

import logging

# --- Koneksi dan Sistem ---

# String koneksi untuk menghubungkan ke drone.
# Gunakan metode koneksi sesuai dengan sistem yang Anda miliki.

# DRONE_CONNECTION_STRING = "udp:127.0.0.1:14550"
# DRONE_CONNECTION_STRING = "/dev/ttyAMA0"
DRONE_CONNECTION_STRING = "/dev/ttyACM0"

# Ketinggian (dalam meter) saat drone melakukan takeoff.
TAKEOFF_ALTITUDE = 30  # meter; drone akan lepas landas pada ketinggian ini

# Mode Deteksi Tracking
# Menentukan algoritma tracking yang akan digunakan untuk mendeteksi dan melacak objek.
# Pilihan yang umum: "CSRT", "TLD", "KCF", "MIL", "BOOSTING", "MEDIANFLOW", "GOTURN", "MOSSE".
TRACKING_MODE = "CSRT"


# --- Parameter Landing ---

# Ketinggian target untuk landing. Drone akan berusaha mencapai ketinggian ini sebelum mendarat.
LANDING_TARGET_ALTITUDE = 1.0  # meter

# Toleransi ketinggian saat landing. Jika perbedaan ketinggian kurang dari nilai ini,
# drone dianggap berada dalam rentang yang aman untuk mendarat.
ALTITUDE_TOLERANCE = 0.50  # meter

# Batas ketinggian untuk memicu proses pendaratan final.
FINAL_LANDING_THRESHOLD = 1.5  # meter

# Parameter waktu minimum (dalam detik) dimana objek harus berada di pusat frame
# agar dianggap stabil untuk melakukan landing.
# Catatan: Meskipun awalnya didefinisikan sebagai 0.001, nilai yang digunakan adalah 0.5 detik.
CENTERED_TIME_THRESHOLD = 0.05  # detik


# --- Konstanta untuk Logika Tracking dan Landing ---

# Faktor konversi error posisi dari satuan pixel ke meter.
# Misalnya, jika error 1 pixel setara dengan pergeseran 0.01 meter.
SCALING_FACTOR = 0.01

# Batas error posisi (dalam pixel) agar objek dianggap telah berada di tengah frame.
# Jika error posisi kurang dari 20 pixel, objek dianggap "tengah".
PIXEL_ERROR_THRESHOLD =80.0  # pixel

# Batas jumlah frame yang kehilangan tracking sebelum sistem menganggap tracking gagal.
MAX_TRACK_LOSS = 5  # frame

# Faktor inversi untuk pergerakan maju/mundur (vel_x).
# Nilai 1 berarti tidak ada inversi, sedangkan -1 akan membalik arah pergerakan.
INVERSION_FORWARD = 1

# Faktor inversi untuk pergerakan lateral (vel_y).
# Nilai -1 berarti arah lateral dibalik, sesuai dengan konfigurasi sistem.
INVERSION_LATERAL = -1

# Parameter untuk pengendalian penurunan ketinggian saat landing.
# DESCENT_STEP menentukan penurunan per langkah (dalam meter) untuk menjaga kestabilan.
DESCENT_STEP = 0.3  # meter per langkah

# CONSTANT_DESCENT_RATE adalah kecepatan penurunan konstan (dalam meter per detik).
CONSTANT_DESCENT_RATE = 10  # meter per detik


# --- PID Tuning Baru ---
# Parameter PID digunakan untuk mengkoreksi pergerakan drone berdasarkan error posisi pada frame.
# Terdapat dua set parameter, satu untuk kontrol maju/mundur (vertikal) dan satu untuk kontrol kiri/kanan (horizontal).

# PID untuk kontrol maju/mundur (error vertikal frame):
PID_FORWARD_KP = 0.075  # Koefisien proporsional; nilai lebih kecil menghasilkan respons yang lebih halus.
PID_FORWARD_KI = 0.000  # Koefisien integral; disetting nol agar tidak terlalu agresif.
PID_FORWARD_KD = 0.01  # Koefisien derivatif; membantu merespons perubahan error dengan cepat.

# PID untuk kontrol kiri/kanan (error horizontal frame):
PID_LATERAL_KP = 0.075  # Koefisien proporsional untuk error horizontal; membantu mengurangi osilasi.
PID_LATERAL_KI = 0.000  # Koefisien integral; disetting nol agar tidak menyebabkan drift.
PID_LATERAL_KD = 0.01  # Koefisien derivatif; membantu sistem cepat merespons perubahan.

# Waktu sampling untuk perhitungan PID (dalam detik).
# Semakin kecil nilai ini, semakin cepat sistem merespons perubahan error.
PID_SAMPLE_TIME = 0.01  # detik

# Batas output untuk PID lateral, dalam satuan meter per detik (m/s).
# Membatasi perintah agar tidak terlalu agresif.
PID_OUTPUT_LIMITS = (-0.4, 0.4) # Default di 0.4


# --- Konfigurasi Kamera ---
# Resolusi tampilan kamera untuk keperluan display pada layar.
CAMERA_WIDTH = 480   # Lebar tampilan (pixel)
CAMERA_HEIGHT = 480  # Tinggi tampilan (pixel)

# Alternatif resolusi (jika diperlukan tampilan dengan resolusi lebih tinggi):
# CAMERA_WIDTH = 2304
# CAMERA_HEIGHT = 1296


# --- Logging ---
# Level logging untuk mengatur pesan yang ditampilkan.
# Contoh level: DEBUG, INFO, WARNING, ERROR, CRITICAL.
LOG_LEVEL = logging.CRITICAL
