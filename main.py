#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py
-------
Modul utama untuk menginisialisasi sistem (drone, kamera, ROI selector, dan tracker),
melakukan proses tracking dan precision landing, serta membersihkan sistem (cleanup)
setelah proses selesai.
"""

# Impor konfigurasi logging terlebih dahulu, sehingga settingnya diterapkan ke seluruh modul
# import logging_config

import logging
import sys
from initializer import initialize_system
from tracking import process_tracking
from cleanup import cleanup


def main() -> None:
    """
    Fungsi utama yang menginisialisasi sistem, menjalankan proses tracking,
    dan melakukan pembersihan sistem (cleanup) saat selesai.
    """
    try:
        # Inisialisasi semua komponen sistem
        drone, camera, roi_selector, tracker = initialize_system()
        logging.info("System initialization complete.")
    except Exception as e:
        logging.error("Gagal menginisialisasi sistem: %s", e)
        sys.exit(1)

    try:
        # Mulai proses tracking dan kontrol landing
        process_tracking(drone, camera, roi_selector, tracker)
    except Exception as e:
        logging.error("Terjadi kesalahan selama proses tracking: %s", e)
    finally:
        # Lakukan pembersihan sistem: landing, tutup koneksi, dll.
        try:
            cleanup(drone, camera)
            logging.info("Cleanup complete.")
        except Exception as cleanup_e:
            logging.error("Gagal saat proses cleanup: %s", cleanup_e)


if __name__ == "__main__":
    main()
