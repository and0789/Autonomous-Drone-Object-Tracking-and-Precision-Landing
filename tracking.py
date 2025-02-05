#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
tracking.py
------------
Modul ini menangani pengambilan frame, pelacakan ROI, dan pengiriman perintah
velocity (lateral dan vertical) untuk precision landing tanpa mengubah heading (yaw)
drone.
"""

import cv2
import time
import logging

from pid_controller import PIDController
from config import (
    LANDING_TARGET_ALTITUDE, ALTITUDE_TOLERANCE, FINAL_LANDING_THRESHOLD,
    CONSTANT_DESCENT_RATE, LATERAL_ERROR_THRESHOLD,
    PID_FORWARD_KP, PID_FORWARD_KI, PID_FORWARD_KD,
    PID_LATERAL_KP, PID_LATERAL_KI, PID_LATERAL_KD,
    INVERSION_FORWARD, INVERSION_LATERAL
)


def process_tracking(drone, camera, roi_selector, tracker):
    """
    Melakukan pengambilan frame, pelacakan ROI, dan mengirim perintah velocity untuk landing.
    Drone akan mempertahankan heading dan hanya bergerak lateral (x,y) tanpa yaw.

    Parameter:
        drone         : Objek DroneController (harus mendukung set/get parameter & send_velocity_command).
        camera        : Objek kamera (misalnya, dari init_camera()).
        roi_selector  : Objek ROISelector untuk pemilihan area ROI.
        tracker       : Objek ObjectTracker untuk pelacakan ROI secara kontinu.
    """
    # Atur agar autopilot tidak otomatis memutar yaw (WP_YAW_BEHAVIOR = 0)
    try:
        drone.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
        drone.vehicle.flush()
        time.sleep(2)  # Beri jeda agar parameter benar-benar ter-set
        logging.info("WP_YAW_BEHAVIOR set to 0. Drone will not rotate automatically during flight.")
    except Exception as e:
        logging.warning(f"Gagal set WP_YAW_BEHAVIOR: {e}")

    # Inisialisasi PID untuk gerak depan/belakang (forward) dan kiri/kanan (lateral)
    pid_forward = PIDController(
        Kp=PID_FORWARD_KP,
        Ki=PID_FORWARD_KI,
        Kd=PID_FORWARD_KD,
        setpoint=0.0,
        sample_time=0.1,
        output_limits=(-0.5, 0.5)
    )
    pid_lateral = PIDController(
        Kp=PID_LATERAL_KP,
        Ki=PID_LATERAL_KI,
        Kd=PID_LATERAL_KD,
        setpoint=0.0,
        sample_time=0.1,
        output_limits=(-0.5, 0.5)
    )

    # Variabel untuk menyimpan altitude terakhir
    last_altitude = None

    logging.info("Mulai proses tracking. Tekan 'q' untuk keluar.")

    # Buat jendela tampilan dan atur callback mouse untuk ROI
    cv2.namedWindow("Camera Feed")
    cv2.setMouseCallback("Camera Feed", roi_selector.select_roi_callback)

    while True:
        # Ambil frame dari kamera
        try:
            frame = camera.capture_array()
        except Exception as e:
            logging.error(f"Error saat menangkap frame: {e}")
            break

        # Dapatkan dimensi frame dan hitung pusat frame
        H, W, _ = frame.shape
        center_x, center_y = W // 2, H // 2

        # Gambar crosshair di pusat frame (penanda visual)
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 255), 1)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 255), 1)

        # Jika ada ROI baru, inisialisasi tracker
        if roi_selector.new_roi_selected:
            if roi_selector.bbox is not None:
                tracker.init_tracker(frame, roi_selector.bbox)
                logging.info(f"Tracker diinisialisasi dengan ROI: {roi_selector.bbox}")
            roi_selector.new_roi_selected = False

        # Set kecepatan default (nol) jika tracking gagal atau tidak ada aksi
        vel_x, vel_y, vel_z = 0, 0, 0

        # Update tracker dengan frame terbaru
        success, bbox = tracker.update_tracker(frame)
        if success:
            (x, y, w, h) = [int(v) for v in bbox]
            target_x = x + w // 2
            target_y = y + h // 2

            if w > 0 and h > 0:
                # Gambar bounding box pada objek yang dilacak
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)

                # Hitung error pixel dari pusat frame
                error_x = target_x - center_x  # Berhubungan dengan pergerakan kiri/kanan => vel_y
                error_y = target_y - center_y  # Berhubungan dengan pergerakan maju/mundur => vel_x

                # Hitung perintah kecepatan menggunakan PID
                vel_x = INVERSION_FORWARD * pid_forward.compute(error_y)   # Maju/mundur
                vel_y = INVERSION_LATERAL * pid_lateral.compute(error_x)     # Kiri/kanan

                logging.info(
                    "PID: err_x=%.2f, err_y=%.2f => vx=%.2f, vy=%.2f",
                    error_x, error_y, vel_x, vel_y
                )

                # Kalkulasi error gabungan untuk menentukan apakah objek sudah berada di tengah
                lateral_error = (abs(error_x) + abs(error_y)) / 2.0

                # Ambil data altitude dari drone
                alt_msg = drone.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                current_alt = None
                if alt_msg:
                    # Asumsi: alt_msg.relative_alt dalam cm; jika benar, bagi dengan 1000.0 untuk meter
                    current_alt = alt_msg.relative_alt / 1000.0  
                    last_altitude = current_alt
                else:
                    current_alt = last_altitude if last_altitude is not None else 10.0

                logging.debug(f"Current altitude = {current_alt:.2f} m (Last alt: {last_altitude})")

                # Logic precision landing
                if current_alt < 0.3:
                    logging.debug("Altitude < 0.3m, skip precision landing logic.")
                else:
                    if lateral_error < LATERAL_ERROR_THRESHOLD:
                        if current_alt > (LANDING_TARGET_ALTITUDE + ALTITUDE_TOLERANCE):
                            # Jika di atas target, drone mulai turun secara kontinu (vel_z > 0)
                            vel_z = CONSTANT_DESCENT_RATE
                            logging.info(
                                "Drone sedang turun: alt=%.2f, vel_z=%.2f",
                                current_alt, vel_z
                            )
                        else:
                            # Jika mendekati target, cek final landing
                            if current_alt <= FINAL_LANDING_THRESHOLD:
                                logging.info(
                                    "Altitude %.2f <= %.2f => Final LAND",
                                    current_alt, FINAL_LANDING_THRESHOLD
                                )
                                drone.land_drone()
                                logging.info("Mode diubah menjadi LAND. Menunggu drone mendarat...")
                                break
                            else:
                                vel_z = 0
                    else:
                        vel_z = 0

                # Kirim perintah kecepatan ke drone
                drone.send_velocity_command(vel_x, vel_y, vel_z)
                logging.info("Velocity command: vx=%.2f, vy=%.2f, vz=%.2f", vel_x, vel_y, vel_z)
            else:
                logging.debug("BBox invalid (width/height nol). Skip.")
        else:
            logging.warning("Tracking hilang. Silakan pilih ROI kembali.")
            tracker.reset_tracker()
            roi_selector.bbox = None

        # Gambar ROI sementara saat seleksi berlangsung
        roi_selector.draw_temporary_roi(frame)

        # Tampilkan informasi altitude pada frame
        altitude_text = "Altitude: N/A"
        if last_altitude is not None:
            altitude_text = f"Altitude: {last_altitude:.2f} m"
        cv2.putText(
            frame,
            altitude_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            1
        )

        cv2.imshow("Camera Feed", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            logging.info("User menekan 'q'. Keluar program.")
            break

    logging.info("Proses tracking selesai.")


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s"
    )
    logging.info("Module tracking dijalankan langsung.")
