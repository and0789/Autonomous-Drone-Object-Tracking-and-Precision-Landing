#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
main.py
-------
Modul utama untuk menginisialisasi sistem (drone, kamera, ROI selector, dan tracker),
melakukan proses tracking dan precision landing, serta membersihkan sistem (cleanup)
setelah proses selesai.
"""

import logging
from init_system import initialize_system
from tracking import process_tracking
from cleanup import cleanup


def main():
    """
    Fungsi utama yang menginisialisasi sistem, menjalankan proses tracking,
    dan melakukan pembersihan sistem (cleanup) saat selesai.
    """
    # Inisialisasi semua komponen sistem
    drone, camera, roi_selector, tracker = initialize_system()
    
    try:
        # Mulai proses tracking dan kontrol landing
        process_tracking(drone, camera, roi_selector, tracker)
    finally:
        # Lakukan pembersihan sistem: landing, tutup koneksi, dll.
        cleanup(drone, camera)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s"
    )
    main()
