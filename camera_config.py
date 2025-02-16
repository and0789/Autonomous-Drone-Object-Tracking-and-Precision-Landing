#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_config.py
----------------
Modul ini berisi fungsi untuk menginisialisasi dan mengonfigurasi Picamera2
dengan resolusi tertentu serta format RGB888. Hasil tangkapan kamera diambil
dalam resolusi penuh (misalnya 4068×2592) tanpa cropping, dan dilakukan flip horizontal
dan vertical (tanpa crop) sesuai kebutuhan.

Dependensi:
- picamera2
- libcamera
"""

import logging
from picamera2 import Picamera2
from libcamera import Transform

def init_camera(width: int = 2304, height: int = 1296, rotation: int = 180) -> Picamera2:
    """
    Inisialisasi Picamera2 dengan resolusi dan format RGB888 yang ditentukan.
    Hasil tangkapan kamera akan di-flip (hflip dan vflip) tanpa crop.
    
    :param width: Lebar frame video (piksel). Default 4068 piksel.
    :param height: Tinggi frame video (piksel). Default 2592 piksel.
    :param rotation: Sudut rotasi (derajat) untuk frame video. (Parameter ini
                     dapat digunakan jika diperlukan, saat ini hanya flip yang diterapkan.)
    :return: Objek Picamera2 yang sudah dikonfigurasi dan dimulai.
    """
    try:
        # Inisialisasi objek Picamera2
        picam2 = Picamera2()
        logging.info("Picamera2 berhasil diinisialisasi.")
    except Exception as e:
        logging.error("Gagal menginisialisasi Picamera2: %s", e)
        raise

    try:
        # Buat konfigurasi video dengan resolusi penuh tanpa cropping.
        # Transformasi hanya melakukan flip (hflip dan vflip) tanpa crop.
        video_config = picam2.create_video_configuration(
            main={
                "size": (width, height),
                "format": "RGB888"
            },
            transform=Transform(hflip=True, vflip=True)
        )

        # Terapkan konfigurasi dan mulai kamera
        picam2.configure(video_config)
        logging.info("Konfigurasi video diterapkan: ukuran %dx%d, format RGB888, rotasi %d°", width, height, rotation)

        picam2.start()
        logging.info("Picamera2 mulai menangkap video.")
    except Exception as e:
        logging.error("Gagal mengonfigurasi atau memulai Picamera2: %s", e)
        raise

    return picam2
