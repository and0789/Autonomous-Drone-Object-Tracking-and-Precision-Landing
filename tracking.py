#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracking Module for Precision Landing
---------------------------------------
Modul ini menggabungkan:
  - Pengambilan frame secara multithreading.
  - Pemilihan ROI secara live atau melalui mode pause (tekan 'p').
  - Inisialisasi ulang tracker secara dinamis apabila ROI diubah.
  - Penggunaan Kalman Filter untuk stabilisasi estimasi posisi objek.
  - PID Controller untuk koreksi pergerakan drone berdasarkan error posisi.
  - Konversi error pixel ke meter agar perintah gerak lebih logis.
  - Logika descent dengan threshold waktu ketika objek berada di tengah,
    sehingga drone dapat mendarat secara presisi di atas objek yang terdeteksi.
"""

import threading
import cv2
import time
import logging
import numpy as np

from pid_controller import PIDController
from config import (
    LANDING_TARGET_ALTITUDE, ALTITUDE_TOLERANCE, FINAL_LANDING_THRESHOLD,
    CONSTANT_DESCENT_RATE, 
    PID_FORWARD_KP, PID_FORWARD_KI, PID_FORWARD_KD,
    PID_LATERAL_KP, PID_LATERAL_KI, PID_LATERAL_KD,
    INVERSION_FORWARD, INVERSION_LATERAL,
    CAMERA_WIDTH, CAMERA_HEIGHT, CENTERED_TIME_THRESHOLD, SCALING_FACTOR,
    PIXEL_ERROR_THRESHOLD, MAX_TRACK_LOSS
)
from kalman_filter import KalmanFilter2D  
from frame_grabber import FrameGrabber


def process_tracking(drone, camera, roi_selector, tracker):
    """
    Fungsi utama untuk tracking objek dan pengendalian drone untuk precision landing.

    Proses kerja:
      1. Setelah kamera dibuka, pengguna dapat memilih objek (ROI) secara langsung
         melalui interaksi mouse pada live video atau dengan menekan 'p' untuk mode pause.
      2. Apabila ROI dipilih atau diubah, tracker akan di-reset dan diinisialisasi ulang
         dengan data ROI terbaru.
      3. Selanjutnya, sistem masuk ke mode tracking, di mana error posisi dihitung
         antara pusat frame dengan posisi objek (yang telah diproses melalui Kalman Filter).
      4. Berdasarkan error yang terukur, PID Controller menghitung offset pergerakan drone.
      5. Jika objek berada di tengah (dengan error di bawah threshold) selama periode
         tertentu, perintah penurunan ketinggian (descent) diberikan sehingga drone dapat mendarat.
    """
    # Nonaktifkan rotasi yaw otomatis drone
    try:
        drone.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
        drone.vehicle.flush()
        time.sleep(2)
        logging.info("WP_YAW_BEHAVIOR diset ke 0. Drone tidak akan berputar otomatis.")
    except Exception as e:
        logging.warning(f"Gagal meng-set WP_YAW_BEHAVIOR: {e}")

    # Inisialisasi PID Controller untuk pergerakan forward dan lateral
    pid_forward = PIDController(PID_FORWARD_KP, PID_FORWARD_KI, PID_FORWARD_KD,
                                0.0, 0.1, (-0.5, 0.5))
    pid_lateral = PIDController(PID_LATERAL_KP, PID_LATERAL_KI, PID_LATERAL_KD,
                                0.0, 0.1, (-0.5, 0.5))

    # Inisialisasi Kalman Filter untuk memperbaiki estimasi posisi objek
    kf = KalmanFilter2D(dt=0.1)

    # Mulai pengambilan frame secara multithreading
    frame_grabber = FrameGrabber(camera).start()

    # Variabel untuk menyimpan ketinggian terakhir dan waktu objek berada di tengah
    last_altitude = None
    centered_start = None
    track_loss_count = 0

    # Atur ROI selector ke mode live secara default
    roi_selector.set_mode("live")

    # Buat window untuk tampilan tracking dan set callback untuk pemilihan ROI
    cv2.namedWindow("Tracking")
    cv2.setMouseCallback("Tracking", roi_selector.select_roi_callback)

    logging.info("Proses tracking dimulai. Tekan 'q' untuk keluar, 'p' untuk pause dan pilih ROI.")

    while True:
        frame = frame_grabber.read()
        if frame is None:
            continue
        # Sesuaikan ukuran frame sesuai konfigurasi
        original_height, original_width = frame.shape[:2]
        target_width = CAMERA_WIDTH
        scale_factor = target_width / original_width
        target_height = int(original_height * scale_factor)
        frame = cv2.resize(frame, (target_width, target_height))

        display_frame = frame.copy()

        # Gambar crosshair pada pusat frame
        center_x, center_y = CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 255), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 255), 1)

        # Periksa input keyboard
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            logging.info("Perintah quit diterima. Menghentikan tracking.")
            break
        elif key == ord('p'):
            # Mode pause: izinkan pengguna memilih ROI pada frame beku
            logging.info("Mode pause aktif. Silakan pilih ROI pada frame yang di-pause.")
            roi_selector.set_mode("paused")
            paused_frame = display_frame.copy()
            roi_selector.select_roi_paused(paused_frame)
            if roi_selector.new_roi_selected and roi_selector.bbox is not None:
                logging.info(f"ROI baru dipilih: {roi_selector.bbox}")
                tracker.reset_tracker()
                tracker.init_tracker(paused_frame, roi_selector.bbox)
                kf.initialized = False  # Reset Kalman Filter untuk tracker baru
                roi_selector.new_roi_selected = False
            roi_selector.set_mode("live")

        # Jika pengguna memilih ROI secara langsung (live mode) melalui callback mouse
        if roi_selector.mode == "live" and roi_selector.new_roi_selected:
            if roi_selector.bbox is not None:
                logging.info(f"ROI dipilih di live mode: {roi_selector.bbox}")
                tracker.reset_tracker()
                tracker.init_tracker(frame, roi_selector.bbox)
                kf.initialized = False
            else:
                logging.warning("ROI tidak valid pada live mode.")
            roi_selector.new_roi_selected = False

        # Perbarui informasi ketinggian drone
        current_alt = drone.get_altitude()
        if current_alt is not None:
            last_altitude = current_alt
        else:
            current_alt = last_altitude if last_altitude is not None else 10.0

        # Tampilkan informasi ketinggian pada frame output
        altitude_text = f"Altitude: {last_altitude:.2f} m" if last_altitude is not None else "Altitude: N/A"
        cv2.putText(display_frame, altitude_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Proses tracking dengan tracker yang telah diinisialisasi
        success, bbox = tracker.update_tracker(frame)
        if success and bbox is not None:
            track_loss_count = 0
            (x, y, w, h) = [int(v) for v in bbox]
            # Hitung titik tengah dari bounding box
            measured_center = np.array([[x + w // 2], [y + h // 2]])
            # Perbaiki estimasi posisi dengan Kalman Filter
            predicted_center = kf.step(measured_center)
            # Hitung error posisi (dalam pixel) antara objek dengan pusat frame
            error_x = int(predicted_center[0, 0]) - center_x
            error_y = int(predicted_center[1, 0]) - center_y
            # Konversi error dari pixel ke meter
            scaled_error_x = error_x * SCALING_FACTOR
            scaled_error_y = error_y * SCALING_FACTOR

            # Gambar bounding box dan titik pusat prediksi pada tampilan
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(display_frame, (int(predicted_center[0, 0]), int(predicted_center[1, 0])),
                       5, (0, 0, 255), -1)

            # Hitung perintah offset menggunakan PID Controller
            forward_offset = INVERSION_FORWARD * pid_forward.compute(scaled_error_y)
            lateral_offset = INVERSION_LATERAL * pid_lateral.compute(scaled_error_x)
            vertical_command = 0

            # Tentukan apakah objek sudah cukup "tengah" (error di bawah threshold)
            avg_error = (abs(error_x) + abs(error_y)) / 2.0
            if avg_error < PIXEL_ERROR_THRESHOLD:
                if centered_start is None:
                    centered_start = time.time()
                elif time.time() - centered_start >= CENTERED_TIME_THRESHOLD:
                    # Jika objek tetap di tengah selama waktu yang ditentukan, perintah descent dijalankan
                    if current_alt > (LANDING_TARGET_ALTITUDE + ALTITUDE_TOLERANCE):
                        vertical_command = CONSTANT_DESCENT_RATE
                    elif current_alt <= FINAL_LANDING_THRESHOLD:
                        logging.info("Kondisi pendaratan final terpenuhi. Menginisiasi pendaratan.")
                        drone.land_drone()
                        break
                    # Opsi: jika objek sudah tepat di tengah, offset horizontal dihilangkan
                    forward_offset = 0.0
                    lateral_offset = 0.0
            else:
                centered_start = None

            # Kirim perintah pergerakan ke drone
            drone.send_body_offset_position_xy(forward_offset, lateral_offset, vertical_command, 0.1)
            # logging.info(f"Drone Command - Forward: {forward_offset:.2f}, Lateral: {lateral_offset:.2f}, "
            #              f"Vertical: {vertical_command:.2f}")
        else:
            # Jika tracking gagal, naikkan counter kehilangan tracking
            track_loss_count += 1
            if track_loss_count >= MAX_TRACK_LOSS:
                logging.warning("Tracking hilang. Tracker di-reset. Silakan pilih ROI kembali.")
                tracker.reset_tracker()
                roi_selector.bbox = None
                kf.initialized = False
                track_loss_count = 0

        # Tampilkan frame akhir dengan overlay
        cv2.imshow("Tracking", display_frame)

    # Lakukan clean-up pada saat keluar dari loop
    frame_grabber.stop()
    cv2.destroyAllWindows()
    logging.info("Proses tracking telah selesai.")

if __name__ == "__main__":
    # Untuk pengujian standalone, pastikan modul roi_selector dan tracker telah diimplementasikan.
    from roi_selection import ROISelector
    from object_tracker import ObjectTracker

    # Inisialisasi objek dummy untuk pengujian
    # class DummyDrone:
    #     def __init__(self):
    #         self.vehicle = type("Vehicle", (), {"parameters": {}, "flush": lambda self: None})
    #     def get_altitude(self):
    #         return 15.0  # Contoh ketinggian (meter)
    #     def send_body_offset_position_xy(self, forward, lateral, vertical, duration):
    #         print(f"Send Command => Forward: {forward}, Lateral: {lateral}, Vertical: {vertical}")
    #     def land_drone(self):
    #         print("Drone landing initiated.")

    # # Contoh inisialisasi kamera (bisa menggunakan index kamera 0)
    # camera = cv2.VideoCapture(0)
    # drone = DummyDrone()
    # roi_selector = ROISelector()
    # tracker = ObjectTracker()

    # process_tracking(drone, camera, roi_selector, tracker)

    # # Contoh _FrameGrabber_ sederhana untuk Picamera2
    # class Picamera2FrameGrabber:
    #     def __init__(self, picam2):
    #         self.picam2 = picam2
    #         self.running = True
    #     def start(self):
    #         return self
    #     def read(self):
    #         return self.picam2.capture_array()
    #     def stop(self):
    #         self.running = False

    # camera = Picamera2FrameGrabber(picam2)
    # drone = DummyDrone()
    # roi_selector = ROISelector()
    # tracker = ObjectTracker()

    # process_tracking(drone, camera, roi_selector, tracker)
