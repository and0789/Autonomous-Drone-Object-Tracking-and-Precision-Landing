#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracking Module for Precision Landing (Improved)
---------------------------------------
Modul ini menggabungkan:
  - Pengambilan frame secara multithreading.
  - Pemilihan ROI secara live atau melalui mode pause (tekan 'p').
  - Inisialisasi ulang tracker secara dinamis apabila ROI diubah.
  - Penggunaan Kalman Filter untuk stabilisasi estimasi posisi objek.
  - Adaptive PID Controller (dari pid_controller.py) dengan dynamic gain scheduling, feedforward control, deadband, dan anti-windup.
  - Konversi error pixel ke meter agar perintah gerak lebih logis.
  - Logika descent dengan threshold waktu ketika objek berada di tengah, sehingga drone dapat mendarat secara presisi.
"""

import cv2
import time
import logging
import numpy as np

from pid_controller import AdaptivePIDController
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

# Konstanta tambahan untuk strategi adaptif
DYNAMIC_ERROR_THRESHOLD = 50   # Jika error (dalam pixel) melebihi nilai ini, gain dan output diperbesar
DEADBAND_THRESHOLD = 5         # Error di bawah nilai ini dianggap nol untuk menghindari osilasi
K_FF = 0.1                     # Konstanta feedforward untuk mengantisipasi laju perubahan error

def process_tracking(drone, camera, roi_selector, tracker):
    """
    Fungsi utama untuk tracking dan precision landing.
    
    Implementasi strategi:
      1. Adaptive Gain Scheduling: Mengubah gain dan output limit berdasarkan besar error.
      2. Deadband: Mengabaikan error kecil agar perintah tidak sensitif terhadap noise.
      3. Output Saturation & Anti-Windup: Batas output PID dan pembatasan akumulasi error (integral).
      4. Feedforward Control: Menambahkan komponen berdasarkan laju perubahan error.
    """
    # Nonaktifkan rotasi yaw otomatis drone
    try:
        drone.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
        drone.vehicle.flush()
        time.sleep(2)
        logging.info("WP_YAW_BEHAVIOR diset ke 0. Drone tidak akan berputar otomatis.")
    except Exception as e:
        logging.warning(f"Gagal meng-set WP_YAW_BEHAVIOR: {e}")

    # Inisialisasi Adaptive PID Controller untuk pergerakan forward dan lateral
    pid_forward = AdaptivePIDController(PID_FORWARD_KP, PID_FORWARD_KI, PID_FORWARD_KD,
                                        setpoint=0.0, sample_time=0.1,
                                        output_limits=(-0.5, 0.5), integral_limits=(-5, 5))
    pid_lateral = AdaptivePIDController(PID_LATERAL_KP, PID_LATERAL_KI, PID_LATERAL_KD,
                                        setpoint=0.0, sample_time=0.1,
                                        output_limits=(-0.5, 0.5), integral_limits=(-5, 5))

    # Inisialisasi Kalman Filter untuk estimasi posisi objek
    kf = KalmanFilter2D(dt=0.1)

    # Mulai pengambilan frame secara multithreading
    frame_grabber = FrameGrabber(camera).start()

    last_altitude = None
    centered_start = None
    track_loss_count = 0

    # Set ROI selector ke mode live dan atur callback mouse
    roi_selector.set_mode("live")
    cv2.namedWindow("Tracking")
    cv2.setMouseCallback("Tracking", roi_selector.select_roi_callback)

    logging.info("Proses tracking dimulai. Tekan 'q' untuk keluar, 'p' untuk pause dan pilih ROI.")

    # Variabel untuk feedforward: menyimpan error dan waktu sebelumnya
    prev_time = time.time()
    prev_scaled_error_x = 0.0
    prev_scaled_error_y = 0.0

    while True:
        frame = frame_grabber.read()
        if frame is None:
            continue

        # Sesuaikan ukuran frame untuk display
        original_height, original_width = frame.shape[:2]
        target_width = CAMERA_WIDTH
        scale_factor = target_width / original_width
        target_height = int(original_height * scale_factor)
        frame = cv2.resize(frame, (target_width, target_height))

        display_frame = frame.copy()
        center_x, center_y = CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 255), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 255), 1)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            logging.info("Perintah quit diterima. Menghentikan tracking.")
            break
        elif key == ord('p'):
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

        # Jika ROI dipilih secara live melalui callback mouse
        if roi_selector.mode == "live" and roi_selector.new_roi_selected:
            if roi_selector.bbox is not None:
                logging.info(f"ROI dipilih di live mode: {roi_selector.bbox}")
                tracker.reset_tracker()
                tracker.init_tracker(frame, roi_selector.bbox)
                kf.initialized = False
            else:
                logging.warning("ROI tidak valid pada live mode.")
            roi_selector.new_roi_selected = False

        # Update informasi ketinggian drone
        current_alt = drone.get_altitude()
        if current_alt is not None:
            last_altitude = current_alt
        else:
            current_alt = last_altitude if last_altitude is not None else 10.0

        altitude_text = f"Altitude: {last_altitude:.2f} m" if last_altitude is not None else "Altitude: N/A"
        cv2.putText(display_frame, altitude_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Proses tracking dengan tracker yang telah diinisialisasi
        success, bbox = tracker.update_tracker(frame)
        if success and bbox is not None:
            track_loss_count = 0
            (x, y, w, h) = [int(v) for v in bbox]
            # Hitung titik tengah bounding box dan perbaiki dengan Kalman Filter
            measured_center = np.array([[x + w // 2], [y + h // 2]])
            predicted_center = kf.step(measured_center)
            error_x = int(predicted_center[0, 0]) - center_x
            error_y = int(predicted_center[1, 0]) - center_y

            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(display_frame, (int(predicted_center[0, 0]), int(predicted_center[1, 0])), 5, (0, 0, 255), -1)

            # Hitung rata-rata error sebagai dasar adaptive gain scheduling
            avg_error = (abs(error_x) + abs(error_y)) / 2.0

            # Adaptive Gain Scheduling & Output Saturation
            if avg_error > DYNAMIC_ERROR_THRESHOLD:
                pid_forward.output_limits = (-1.0, 1.0)
                pid_lateral.output_limits = (-1.0, 1.0)
                pid_forward.Kp = PID_FORWARD_KP * 1.5
                pid_lateral.Kp = PID_LATERAL_KP * 1.5
            else:
                pid_forward.output_limits = (-0.5, 0.5)
                pid_lateral.output_limits = (-0.5, 0.5)
                pid_forward.Kp = PID_FORWARD_KP
                pid_lateral.Kp = PID_LATERAL_KP

            # Deadband: jika error sangat kecil, nol-kan error
            if avg_error < DEADBAND_THRESHOLD:
                error_x = 0
                error_y = 0

            # Konversi error dari pixel ke meter
            scaled_error_x = error_x * SCALING_FACTOR
            scaled_error_y = error_y * SCALING_FACTOR

            # Feedforward: hitung laju perubahan error
            current_time = time.time()
            dt = current_time - prev_time if (current_time - prev_time) > 0 else 0.1
            error_rate_x = (scaled_error_x - prev_scaled_error_x) / dt
            error_rate_y = (scaled_error_y - prev_scaled_error_y) / dt
            feedforward_x = K_FF * error_rate_x
            feedforward_y = K_FF * error_rate_y

            prev_scaled_error_x = scaled_error_x
            prev_scaled_error_y = scaled_error_y
            prev_time = current_time

            # Hitung output PID dan tambahkan feedforward control
            forward_output = pid_forward.compute(scaled_error_y)
            lateral_output = pid_lateral.compute(scaled_error_x)
            forward_offset = INVERSION_FORWARD * (forward_output + feedforward_y)
            lateral_offset = INVERSION_LATERAL * (lateral_output + feedforward_x)
            vertical_command = 0

            # Logika descent: jika error cukup kecil, drone mulai turun
            if avg_error < PIXEL_ERROR_THRESHOLD:
                if centered_start is None:
                    centered_start = time.time()
                elif time.time() - centered_start >= CENTERED_TIME_THRESHOLD:
                    if current_alt > (LANDING_TARGET_ALTITUDE + ALTITUDE_TOLERANCE):
                        vertical_command = CONSTANT_DESCENT_RATE
                    elif current_alt <= FINAL_LANDING_THRESHOLD:
                        logging.info("Kondisi pendaratan final terpenuhi. Menginisiasi pendaratan.")
                        drone.land_drone()
                        break
                    # Saat landing, perintah horizontal di-nol-kan
                    forward_offset = 0.0
                    lateral_offset = 0.0
            else:
                centered_start = None

            # Kirim perintah gerak ke drone
            drone.send_body_offset_position_xy(forward_offset, lateral_offset, vertical_command, 0.1)
        else:
            # Jika tracking gagal, tingkatkan counter kehilangan tracking
            track_loss_count += 1
            if track_loss_count >= MAX_TRACK_LOSS:
                logging.warning("Tracking hilang. Tracker di-reset. Silakan pilih ROI kembali.")
                tracker.reset_tracker()
                roi_selector.bbox = None
                kf.initialized = False
                track_loss_count = 0

        cv2.imshow("Tracking", display_frame)

    frame_grabber.stop()
    cv2.destroyAllWindows()
    logging.info("Proses tracking telah selesai.")

if __name__ == "__main__":
    # Untuk pengujian standalone, pastikan modul roi_selector dan tracker telah diimplementasikan.
    from roi_selection import ROISelector
    from object_tracker import ObjectTracker

    # Contoh pengujian dengan dummy drone dan kamera lokal
    # class DummyDrone:
    #     def __init__(self):
    #         self.vehicle = type("Vehicle", (), {"parameters": {}, "flush": lambda self: None})
    #     def get_altitude(self):
    #         return 15.0
    #     def send_body_offset_position_xy(self, forward, lateral, vertical, duration):
    #         print(f"Send Command => Forward: {forward:.2f}, Lateral: {lateral:.2f}, Vertical: {vertical:.2f}")
    #     def land_drone(self):
    #         print("Drone landing initiated.")
    # 
    # camera = cv2.VideoCapture(0)
    # drone = DummyDrone()
    # roi_selector = ROISelector()
    # tracker = ObjectTracker()
    # process_tracking(drone, camera, roi_selector, tracker)
