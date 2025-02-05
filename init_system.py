#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
initializer.py
--------------
Modul ini bertanggung jawab untuk inisialisasi semua komponen sistem:
- DroneController: koneksi MAVLink, mode, arming, dan takeoff.
- Kamera: inisialisasi Picamera2 (atau kamera lain) dengan resolusi tertentu.
- ROISelector dan ObjectTracker: untuk pelacakan objek visual.
"""

import sys
import logging

from drone_controller import DroneController
from camera_config import init_camera
from roi_selector import ROISelector
from object_tracker import ObjectTracker
from config import (
    DRONE_CONNECTION_STRING,
    TAKEOFF_ALTITUDE,
    CAMERA_WIDTH,
    CAMERA_HEIGHT
)


def initialize_system():
    """
    Inisialisasi semua komponen sistem.

    - Drone: terhubung via DRONE_CONNECTION_STRING, set mode GUIDED, arm, dan takeoff.
    - Kamera: inisialisasi dengan resolusi dari config (CAMERA_WIDTH, CAMERA_HEIGHT).
    - ROI Selector dan Object Tracker: untuk pelacakan visual.

    :return: Tuple (drone, camera, roi_selector, tracker)
    :rtype: (DroneController, Any, ROISelector, ObjectTracker)
    """
    # Inisialisasi drone
    drone = DroneController(connection_string=DRONE_CONNECTION_STRING)
    drone.set_mode_guided()
    drone.arm_drone()
    drone.takeoff(target_altitude=TAKEOFF_ALTITUDE)
    logging.info("Drone takeoff complete.")

    # Inisialisasi kamera
    try:
        camera = init_camera(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
        logging.info("Camera successfully initialized.")
    except Exception as e:
        logging.error(f"Error initializing camera: {e}")
        drone.land_drone()
        drone.close_connection()
        sys.exit(1)

    # Inisialisasi ROI Selector dan Object Tracker
    roi_selector = ROISelector()
    tracker = ObjectTracker()

    return drone, camera, roi_selector, tracker


if __name__ == "__main__":
    # Logging dasar untuk pengujian
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s"
    )

    drone, camera, roi_selector, tracker = initialize_system()
    logging.info("System initialization complete.")
