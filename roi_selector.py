#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
roi_selector.py
---------------
Modul ini menyediakan kelas ROISelector untuk memilih dan mengelola
Region of Interest (ROI) menggunakan input mouse pada frame OpenCV.
"""

import cv2


class ROISelector:
    """
    Kelas untuk memilih dan mengelola Region of Interest (ROI).

    Atribut:
        roi_start (tuple): Titik awal seleksi ROI.
        roi_end (tuple): Titik akhir seleksi ROI.
        selecting_roi (bool): Flag yang menunjukkan bahwa ROI sedang dipilih.
        roi_selected (bool): Flag yang menunjukkan bahwa ROI telah dipilih.
        bbox (tuple): Bounding box ROI dalam format (x, y, w, h).
        new_roi_selected (bool): Flag yang menandakan bahwa ROI baru telah dipilih.
    """

    def __init__(self):
        """Inisialisasi atribut ROISelector dengan nilai default."""
        self.roi_start = (0, 0)
        self.roi_end = (0, 0)
        self.selecting_roi = False
        self.roi_selected = False
        self.bbox = None  # Format: (x, y, w, h)
        self.new_roi_selected = False  # Flag untuk ROI baru

    def select_roi_callback(self, event, x, y, flags, param):
        """
        Callback untuk memilih atau mengubah ROI menggunakan mouse.

        - Klik kiri dan drag untuk memilih ROI.
        - Jika ROI sudah dipilih, klik kiri lagi akan memulai seleksi ROI baru.
        - Klik kanan untuk membatalkan ROI dan tracking (misal, pendaratan dibatalkan).

        :param event: Event yang terjadi (misalnya, cv2.EVENT_LBUTTONDOWN).
        :param x: Koordinat x dari event mouse.
        :param y: Koordinat y dari event mouse.
        :param flags: Flags tambahan dari event.
        :param param: Parameter tambahan (tidak digunakan).
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            # Mulai memilih ROI baru (meskipun sudah ada ROI sebelumnya)
            self.roi_start = (x, y)
            self.roi_end = (x, y)
            self.selecting_roi = True
            self.roi_selected = False
            self.bbox = None
            self.new_roi_selected = False
            print("Mulai memilih ROI.")

        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_roi:
            # Update titik akhir ROI saat drag
            self.roi_end = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            # Selesai memilih ROI
            self.roi_end = (x, y)
            self.selecting_roi = False
            self.roi_selected = True

            # Hitung bounding box
            x_min = min(self.roi_start[0], self.roi_end[0])
            y_min = min(self.roi_start[1], self.roi_end[1])
            w = max(self.roi_end[0], self.roi_start[0]) - x_min
            h = max(self.roi_end[1], self.roi_start[1]) - y_min

            if w > 0 and h > 0:
                self.bbox = (x_min, y_min, w, h)
                self.new_roi_selected = True
                print(f"ROI dipilih: {self.bbox}")
            else:
                # ROI tidak valid, reset
                self.bbox = None
                self.roi_selected = False
                self.new_roi_selected = False
                print("ROI tidak valid. Silakan pilih kembali.")

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Klik kanan untuk membatalkan ROI/tracking (pendaratan dibatalkan)
            self.cancel_roi()

    def cancel_roi(self):
        """
        Membatalkan ROI yang telah dipilih dan mereset semua flag.
        Digunakan untuk membatalkan pendaratan atau tracking.
        """
        self.roi_start = (0, 0)
        self.roi_end = (0, 0)
        self.selecting_roi = False
        self.roi_selected = False
        self.bbox = None
        self.new_roi_selected = False
        print("ROI dibatalkan. Tracking/pendaratan dibatalkan.")

    def draw_temporary_roi(self, frame):
        """
        Menggambar ROI sementara pada frame saat sedang diseleksi.

        :param frame: Frame gambar tempat ROI akan digambar.
        """
        if self.selecting_roi:
            cv2.rectangle(frame, self.roi_start, self.roi_end, (255, 0, 0), 2)
