#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
object_tracker.py
-----------------
Modul ini menyediakan kelas ObjectTracker yang bertanggung jawab untuk
menginisialisasi, meng-update, dan mereset tracker objek menggunakan OpenCV.
"""

import cv2


class ObjectTracker:
    """
    Kelas untuk melakukan pelacakan objek (object tracking) menggunakan OpenCV.
    """

    def __init__(self):
        """
        Inisialisasi ObjectTracker dengan tracker kosong.
        """
        self.tracker = None

    def init_tracker(self, frame, bbox):
        """
        Inisialisasi tracker dengan frame dan bounding box baru.

        :param frame: Frame gambar saat ini.
        :param bbox: Bounding box dalam format (x, y, width, height).
        """
        # Reset tracker jika sudah ada
        if self.tracker is not None:
            self.reset_tracker()

        # Coba inisialisasi tracker CSRT dari cv2.legacy, jika gagal gunakan versi tanpa legacy.
        try:
            self.tracker = cv2.legacy.TrackerCSRT_create()
        except AttributeError:
            self.tracker = cv2.TrackerCSRT_create()

        self.tracker.init(frame, bbox)
        print(f"Tracker diinisialisasi dengan bbox: {bbox}")

    def update_tracker(self, frame):
        """
        Update tracker dengan frame terbaru.

        :param frame: Frame gambar saat ini.
        :return: Tuple (success, bbox) di mana 'success' adalah boolean dan 'bbox'
                 adalah bounding box yang terdeteksi (jika tracking berhasil).
        """
        if self.tracker is None:
            return False, None

        success, bbox = self.tracker.update(frame)
        return success, bbox

    def reset_tracker(self):
        """
        Reset tracker dengan mengatur tracker menjadi None.
        """
        self.tracker = None
        print("Tracker di-reset.")
