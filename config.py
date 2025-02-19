#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
config.py
---------
Konfigurasi umum untuk proyek precision landing.
Pengelompokan parameter berdasarkan fungsinya:

1. Koneksi dan Sistem:
   - DRONE_CONNECTION_STRING: String koneksi untuk menghubungkan ke drone.
     Efek: Menentukan metode koneksi (misalnya UDP, serial) untuk komunikasi dengan drone.
   - TAKEOFF_ALTITUDE: Ketinggian takeoff (meter).
     Efek: Drone lepas landas pada ketinggian ini.

2. Mode Tracking:
   - TRACKING_MODE: Algoritma tracking yang digunakan (contoh: "CSRT", "KCF", dll).
     Efek: Memilih metode deteksi dan pelacakan objek yang mempengaruhi keakuratan tracking.

3. Konstanta Adaptif untuk Logika Tracking:
   - DYNAMIC_ERROR_THRESHOLD: Batas error (pixel) untuk meningkatkan gain/output.
     Efek: Jika error melebihi nilai ini, sistem akan meningkatkan respons koreksi secara agresif.
   - DEADBAND_THRESHOLD: Batas error (pixel) di bawahnya dianggap nol.
     Efek: Mencegah osilasi akibat noise kecil pada tracking.
   - K_FF: Konstanta feedforward.
     Efek: Menambahkan komponen laju perubahan error untuk mempercepat respon koreksi.

4. Parameter Landing:
   - LANDING_TARGET_ALTITUDE: Ketinggian target (meter) untuk landing.
     Efek: Drone berusaha mencapai ketinggian ini sebelum mendarat.
   - ALTITUDE_TOLERANCE: Toleransi ketinggian (meter) saat landing.
     Efek: Jika perbedaan ketinggian kurang dari nilai ini, drone dianggap dalam zona aman untuk landing.
   - FINAL_LANDING_THRESHOLD: Batas ketinggian untuk memicu pendaratan final (meter).
     Efek: Menentukan titik akhir pendaratan, misalnya untuk menghindari pendaratan saat terlalu dekat ke tanah.
   - CENTERED_TIME_THRESHOLD: Waktu minimum (detik) objek harus berada di tengah frame untuk memulai descent.
     Efek: Mencegah perintah descent diberikan terlalu cepat jika objek belum stabil di tengah.

5. Konstanta Logika Tracking dan Landing:
   - SCALING_FACTOR: Faktor konversi error (pixel ke meter).
     Efek: Mengubah error posisi dari piksel menjadi pergeseran dalam meter, yang lebih relevan untuk perintah gerak drone.
   - PIXEL_ERROR_THRESHOLD: Batas error (pixel) agar objek dianggap "tengah".
     Efek: Jika error kurang dari nilai ini, drone tidak mengeluarkan perintah pergeseran horizontal.
   - MAX_TRACK_LOSS: Batas jumlah frame yang kehilangan tracking.
     Efek: Jika tracker gagal mendeteksi objek selama jumlah frame ini, tracker akan di-reset.
   - INVERSION_FORWARD: Faktor inversi untuk kontrol pergerakan maju/mundur.
     Efek: Menentukan apakah perintah maju/mundur perlu dibalik (1: normal, -1: dibalik).
   - INVERSION_LATERAL: Faktor inversi untuk kontrol pergerakan lateral.
     Efek: Menentukan apakah perintah lateral perlu dibalik (1: normal, -1: dibalik).

6. Parameter Penurunan (Descent):
   - DESCENT_STEP: Penurunan per langkah (meter).
     Efek: Menentukan seberapa besar penurunan ketinggian pada tiap langkah (biasanya digunakan untuk descent terukur).
     Keterangan: Jika tidak digunakan, bisa diabaikan.
   - CONSTANT_DESCENT_RATE: Kecepatan penurunan konstan (meter per detik).
     Efek: Digunakan saat perintah descent dikerjakan, menentukan kecepatan penurunan drone.

7. Parameter PID (Tuning):
   - PID_FORWARD_KP, PID_FORWARD_KI, PID_FORWARD_KD:
     Efek: Parameter PID untuk koreksi pergerakan maju/mundur (vertical error pada frame).
     Penjelasan:
       • KP: Mempengaruhi seberapa cepat respon terhadap error.
       • KI: Mengakumulasi error untuk mengatasi offset yang persisten (biasanya diset 0 agar tidak terlalu agresif).
       • KD: Merespons perubahan error secara cepat.
   - PID_LATERAL_KP, PID_LATERAL_KI, PID_LATERAL_KD:
     Efek: Parameter PID untuk koreksi pergerakan lateral (horizontal error pada frame).
   - PID_SAMPLE_TIME: Waktu sampling untuk perhitungan PID (detik).
     Efek: Semakin kecil waktu sampling, semakin cepat respon sistem terhadap perubahan error.
   - PID_OUTPUT_LIMITS: Batas output untuk perintah PID (m/s).
     Efek: Membatasi perintah agar tidak terlalu agresif. (Misalnya, untuk lateral, default (-0.4, 0.4) m/s)

8. Konfigurasi Kamera:
   - CAMERA_WIDTH, CAMERA_HEIGHT:
     Efek: Menentukan resolusi tampilan kamera untuk keperluan display atau pemrosesan.
     Keterangan: Alternatif resolusi dapat digunakan jika diperlukan tampilan dengan resolusi lebih tinggi.

9. Pengaturan Logging:
   - LOG_LEVEL:
     Efek: Menentukan level logging (DEBUG, INFO, WARNING, ERROR, CRITICAL) untuk mengontrol pesan log yang ditampilkan.
"""

import logging

# --- 1. Koneksi dan Sistem ---
DRONE_CONNECTION_STRING = "udp:127.0.0.1:14550"
# DRONE_CONNECTION_STRING = "/dev/ttyAMA0"
# DRONE_CONNECTION_STRING = "/dev/ttyACM0"


TAKEOFF_ALTITUDE = 30  # meter

# --- 2. Mode Tracking ---
# BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN, MOSSE, CSRT
TRACKING_MODE = "CSRT"

# --- 3. Konstanta Adaptif untuk Logika Tracking ---
DYNAMIC_ERROR_THRESHOLD = 50   # pixel; meningkatkan gain/output jika error > nilai ini
DEADBAND_THRESHOLD = 5         # pixel; error di bawah nilai ini diabaikan (nol) untuk mencegah osilasi
K_FF = 0.1                     # konstanta feedforward; menambahkan komponen perubahan error

# --- 4. Parameter Landing ---
LANDING_TARGET_ALTITUDE = 1.0  # meter; target ketinggian landing
ALTITUDE_TOLERANCE = 0.50      # meter; toleransi perbedaan ketinggian
FINAL_LANDING_THRESHOLD = 1.5  # meter; batas untuk pendaratan final
CENTERED_TIME_THRESHOLD = 0.05  # detik; waktu minimal objek harus di tengah untuk mulai descent

# --- 5. Konstanta untuk Logika Tracking dan Landing ---
SCALING_FACTOR = 0.01          # konversi pixel ke meter (misal, 1 pixel = 0.01 m)
PIXEL_ERROR_THRESHOLD = 50.0   # pixel; error di bawah nilai ini dianggap objek sudah di tengah
MAX_TRACK_LOSS = 5             # frame; batas kehilangan tracking sebelum tracker di-reset

INVERSION_FORWARD = 1          # 1 = normal; -1 = membalik perintah maju/mundur
INVERSION_LATERAL = -1         # 1 = normal; -1 = membalik perintah lateral

# --- 6. Parameter Penurunan (Descent) ---
DESCENT_STEP = 0.3             # meter per langkah; (opsional, jika tidak digunakan bisa diabaikan)
CONSTANT_DESCENT_RATE = 5      # meter per detik; kecepatan penurunan konstan saat descent

# --- 7. Parameter PID (Tuning) ---
# Untuk kontrol maju/mundur (vertical error)
PID_FORWARD_KP = 0.100         # gain proporsional
PID_FORWARD_KI = 0.000         # gain integral (biasanya nol untuk respons halus)
PID_FORWARD_KD = 0.070         # gain derivatif

# Untuk kontrol lateral (horizontal error)
PID_LATERAL_KP = 0.100         # gain proporsional
PID_LATERAL_KI = 0.000         # gain integral
PID_LATERAL_KD = 0.070         # gain derivatif

PID_SAMPLE_TIME = 0.05         # detik; waktu sampling untuk perhitungan PID
PID_OUTPUT_LIMITS = (-0.4, 0.4)  # m/s; batas output untuk perintah PID (misalnya untuk kontrol lateral)

# --- 8. Konfigurasi Kamera ---
CAMERA_WIDTH = 640             # pixel; lebar tampilan kamera
CAMERA_HEIGHT = 480            # pixel; tinggi tampilan kamera
# Alternatif resolusi:
# CAMERA_WIDTH = 2304
# CAMERA_HEIGHT = 1296

# --- 9. Pengaturan Logging ---
LOG_LEVEL = logging.CRITICAL
