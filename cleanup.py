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
from typing import Any

def cleanup(drone: Any, camera: Any) -> None:
    """
    Membersihkan sistem setelah pemrosesan utama selesai.

    Prosedur:
      - Memanggil perintah pendaratan drone (land_drone()) jika objek drone valid.
      - Menutup koneksi MAVLink.
      - Menghentikan kamera.
      - Menutup semua jendela OpenCV.

    :param drone: Objek DroneController yang menyediakan land_drone() dan close_connection().
    :param camera: Objek kamera (misalnya Picamera2) yang memiliki method stop().
    """
    # Coba mendaratkan drone terlebih dahulu
    try:
        logging.info("Menjalankan perintah land_drone()...")
        if drone is not None:
            drone.land_drone()
        else:
            logging.warning("Objek drone tidak valid; tidak dapat melakukan land.")
    except Exception as e:
        logging.error("Terjadi kesalahan saat mendaratkan drone: %s", e)
    finally:
        # Tutup koneksi MAVLink pada drone
        try:
            if drone is not None:
                drone.close_connection()
                logging.info("Koneksi drone berhasil ditutup.")
        except Exception as e:
            logging.error("Gagal menutup koneksi drone: %s", e)
        
        # Hentikan kamera
        try:
            if camera is not None:
                camera.stop()
                logging.info("Kamera telah dihentikan.")
        except Exception as e:
            logging.error("Gagal menghentikan kamera: %s", e)
        
        # Tutup semua jendela OpenCV
        try:
            cv2.destroyAllWindows()
            logging.info("Semua jendela OpenCV telah ditutup.")
        except Exception as e:
            logging.error("Gagal menutup jendela OpenCV: %s", e)
        
        logging.info("Cleanup complete. Program finished.")
