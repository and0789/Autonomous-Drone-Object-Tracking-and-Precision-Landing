#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
roi_selector.py
---------------
Modul ini menyediakan kelas ROISelector untuk memilih dan mengelola
Region of Interest (ROI) menggunakan input mouse pada frame OpenCV.

Fitur utama:
1. Mode "live": 
   - Pilih ROI saat video berjalan (klik & drag di jendela video). 
   - Setelah mouse dilepas, ROI langsung ditetapkan dan bisa digunakan oleh tracker.
2. Mode "paused": 
   - Video di-pause (misalnya dengan menekan tombol 'p' di skrip utama).
   - Fungsi cv2.selectROI dipanggil untuk memilih ROI, lalu ROI di-set untuk tracker.
"""

import cv2
import logging
from typing import Tuple, Optional

class ROISelector:
    """
    Kelas untuk memilih dan mengelola Region of Interest (ROI).
    Dilengkapi dengan dua mode:
      - "live":   Pilih ROI secara langsung pada video yang berjalan (klik-drag).
      - "paused": Video di-pause, lalu pilih ROI menggunakan jendela selectROI OpenCV.
    """
    def __init__(self) -> None:
        """
        Inisialisasi atribut ROISelector dengan nilai default.
        """
        # Koordinat awal dan akhir (x, y) saat seleksi ROI "live" (mouse-drag).
        self.roi_start: Tuple[int, int] = (0, 0)
        self.roi_end: Tuple[int, int] = (0, 0)

        # Flag untuk menandakan apakah user sedang klik-drag
        self.selecting_roi: bool = False

        # Flag jika ROI sudah selesai dipilih (mouse dilepas / selesai selectROI)
        self.roi_selected: bool = False

        # Menyimpan bounding box ROI. Format: (x, y, w, h)
        self.bbox: Optional[Tuple[int, int, int, int]] = None

        # Flag untuk menandakan ada ROI baru dipilih 
        # (di-set True setelah mouse dilepas atau setelah selectROI).
        self.new_roi_selected: bool = False

        # Mode default: "live"
        self.mode: str = "live"

    def set_mode(self, mode: str) -> None:
        """
        Mengatur mode ROI Selector. Mode harus 'live' atau 'paused'.

        :param mode: Mode yang diinginkan ('live' atau 'paused').
        :raises ValueError: Jika mode tidak valid.
        """
        if mode in ["live", "paused"]:
            self.mode = mode
            logging.info("Mode ROISelector diatur ke: %s", mode)
        else:
            raise ValueError("Mode harus 'live' atau 'paused'.")

    def select_roi_callback(self, event: int, x: int, y: int, flags, param) -> None:
        """
        Callback mouse untuk memilih/mengubah ROI. Hanya aktif jika mode == 'live'.

        :param event: Jenis event (misalnya, cv2.EVENT_LBUTTONDOWN).
        :param x: Koordinat x dari event mouse.
        :param y: Koordinat y dari event mouse.
        :param flags: Flags tambahan dari event (tidak digunakan).
        :param param: Parameter tambahan (tidak digunakan).
        """
        if self.mode != "live":
            # Jika mode 'paused', callback ini tidak melakukan apa-apa
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            # Mulai klik-drag
            self.roi_start = (x, y)
            self.roi_end = (x, y)
            self.selecting_roi = True
            self.roi_selected = False
            self.bbox = None
            self.new_roi_selected = False
            logging.info("Mulai memilih ROI (live mode).")

        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_roi:
            # Update koordinat akhir saat drag
            self.roi_end = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            # Mouse dilepas, seleksi selesai
            self.roi_end = (x, y)
            self.selecting_roi = False
            self.roi_selected = True

            # Hitung bounding box
            x_min = min(self.roi_start[0], self.roi_end[0])
            y_min = min(self.roi_start[1], self.roi_end[1])
            w = max(self.roi_end[0], self.roi_start[0]) - x_min
            h = max(self.roi_end[1], self.roi_start[1]) - y_min

            # Jika lebar dan tinggi > 0, berarti ROI valid
            if w > 0 and h > 0:
                self.bbox = (x_min, y_min, w, h)
                self.new_roi_selected = True
                logging.info("ROI dipilih (live): %s", self.bbox)
            else:
                # Jika invalid (width/height <= 0), reset
                self.bbox = None
                self.roi_selected = False
                self.new_roi_selected = False
                logging.warning("ROI tidak valid. Silakan pilih kembali.")

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Klik kanan untuk membatalkan ROI
            self.cancel_roi()

    def select_roi_paused(self, frame) -> None:
        """
        Memungkinkan pemilihan ROI pada frame yang di-pause dengan
        menggunakan fungsi cv2.selectROI dari OpenCV.

        :param frame: Frame gambar statis yang akan digunakan untuk pemilihan ROI.
        """
        bbox = cv2.selectROI("Select ROI (Paused Mode)", frame,
                             showCrosshair=True, fromCenter=False)
        cv2.destroyWindow("Select ROI (Paused Mode)")

        if bbox[2] > 0 and bbox[3] > 0:
            # ROI valid
            self.bbox = bbox  # bbox -> (x, y, w, h)
            self.roi_selected = True
            self.new_roi_selected = True
            logging.info("ROI dipilih (paused): %s", bbox)
        else:
            # ROI invalid
            self.bbox = None
            self.roi_selected = False
            self.new_roi_selected = False
            logging.warning("ROI tidak valid (paused mode).")

    def cancel_roi(self) -> None:
        """
        Membatalkan ROI yang telah dipilih dan mereset semua flag terkait.
        """
        self.roi_start = (0, 0)
        self.roi_end = (0, 0)
        self.selecting_roi = False
        self.roi_selected = False
        self.bbox = None
        self.new_roi_selected = False
        logging.info("ROI dibatalkan.")

    def draw_temporary_roi(self, frame) -> None:
        """
        Menggambar ROI sementara (kotak biru) pada frame saat sedang diseleksi di mode "live".

        :param frame: Frame gambar tempat ROI sementara akan digambar.
        """
        if self.selecting_roi:
            cv2.rectangle(frame, self.roi_start, self.roi_end, (255, 0, 0), 2)
