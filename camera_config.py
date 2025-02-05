#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
camera_config.py
----------------
Modul ini berisi fungsi untuk menginisialisasi dan mengkonfigurasi Picamera2
dengan resolusi tertentu serta format RGB888. Kamera juga dirotasi 180 derajat 
menggunakan transformasi bawaan libcamera.

Dependensi:
- picamera2
- libcamera
"""

from picamera2 import Picamera2
from libcamera import Transform


def init_camera(width=640, height=480):
    """
    Inisialisasi Picamera2 dengan resolusi dan format RGB888 yang ditentukan.
    Hasil tangkapan kamera dirotasi 180 derajat.

    :param width: Lebar frame video (piksel). Default 640 piksel.
    :type width: int
    :param height: Tinggi frame video (piksel). Default 480 piksel.
    :type height: int
    :return: Objek Picamera2 yang sudah dikonfigurasi dan dimulai.
    :rtype: Picamera2
    """
    picam2 = Picamera2()

    # Membuat konfigurasi video dengan transformasi rotasi 180 derajat
    video_config = picam2.create_video_configuration(
        main={
            "size": (width, height),
            "format": "RGB888"
        },
        transform=Transform(rotation=180)
    )

    # Terapkan konfigurasi
    picam2.configure(video_config)

    # Mulai menangkap video
    picam2.start()

    return picam2
