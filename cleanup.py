#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cleanup.py
----------
Modul ini menyediakan fungsi untuk membersihkan (clean up) sistem setelah
pengoperasian drone dan kamera selesai. Proses meliputi mendaratkan drone,
menutup koneksi MAVLink, menghentikan kamera, serta menutup jendela OpenCV.
"""

import cv2
import logging


def cleanup(drone, camera):
    """
    Membersihkan sistem setelah pemrosesan utama selesai.

    - Memanggil perintah pendaratan drone (land_drone()).
    - Menutup koneksi MAVLink.
    - Menghentikan kamera (camera.stop()).
    - Menutup semua jendela OpenCV (cv2.destroyAllWindows()).

    :param drone: Objek DroneController yang menyediakan land_drone() dan close_connection().
    :param camera: Objek kamera (misalnya Picamera2) yang memiliki method stop().
    """
    try:
        logging.info("Menjalankan perintah land_drone()...")
        drone.land_drone()
    except Exception as e:
        logging.error(f"Terjadi kesalahan saat mendaratkan drone: {e}")
    finally:
        drone.close_connection()
        camera.stop()
        cv2.destroyAllWindows()
        logging.info("Cleanup complete. Program finished.")
