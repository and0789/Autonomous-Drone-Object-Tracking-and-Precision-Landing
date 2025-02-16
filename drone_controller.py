#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_controller.py
-------------------
Modul ini berisi kelas DroneController yang menyediakan fungsi-fungsi
untuk berinteraksi dengan drone melalui MAVLink, termasuk:
- RC override (move_left_right, move_forward_backward, dll.)
- Pengiriman perintah kecepatan (send_velocity_command)
- Pengaturan mode penerbangan (GUIDED, STABILIZE, ALT_HOLD)
- Arm/disarm, takeoff, dan landing
- Penutupan koneksi MAVLink
- Pembacaan altitude secara kontinu melalui thread
"""

import time
import sys
import logging
import math
import threading
from typing import Optional

import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect


class DroneController:
    """
    Kelas untuk mengontrol drone melalui MAVLink.
    Memungkinkan berbagai aksi seperti override RC, pengaturan mode, arm/takeoff, landing,
    dan juga memiliki thread untuk terus memperbarui nilai ketinggian (altitude).
    """

    def __init__(self, connection_string: str = "udp:127.0.0.1:14550") -> None:
        """
        Inisialisasi DroneController dengan koneksi MAVLink.
        
        :param connection_string: Alamat koneksi MAVLink, misalnya "udp:127.0.0.1:14550" atau "/dev/ttyACM0"
        """
        logging.info(f"Mencoba terhubung ke {connection_string}...")
        self.vehicle = utility.mavlink_connection(device=connection_string)

        try:
            self.vehicle.wait_heartbeat(timeout=10)
            logging.info("Heartbeat diterima!")
        except Exception as e:
            logging.error("Gagal menerima heartbeat: %s", e)
            self.vehicle.close()
            sys.exit(1)

        logging.info("Terhubung ke system: %s, component: %s",
                     self.vehicle.target_system, self.vehicle.target_component)

        # Inisialisasi nilai channel RC default (neutral = 1500)
        self.channels = {1: 1500, 2: 1500, 3: 1500, 4: 1500}
        self.min_pwm = 1000
        self.max_pwm = 2000

        # Inisialisasi thread untuk pembacaan altitude
        self.latest_altitude: Optional[float] = None
        self._altitude_thread_running = True
        self._altitude_thread = threading.Thread(target=self._altitude_listener, daemon=True)
        self._altitude_thread.start()

    def _altitude_listener(self) -> None:
        """
        Thread yang terus-menerus membaca pesan GLOBAL_POSITION_INT secara non-blocking
        dan memperbarui nilai latest_altitude (dalam meter, dikonversi dari milimeter).
        """
        while self._altitude_thread_running:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                try:
                    # Konversi dari milimeter ke meter
                    self.latest_altitude = msg.relative_alt / 1000.0
                except Exception as e:
                    logging.warning("[Altitude Listener] Error saat konversi altitude: %s", e)
            time.sleep(0.1)  # Cek setiap 100 ms

    def get_latest_altitude(self) -> Optional[float]:
        """
        Mengembalikan nilai altitude terbaru (relative altitude) dalam meter.
        
        :return: Altitude (meter) atau None jika belum tersedia.
        """
        return self.latest_altitude

    def get_altitude(self) -> Optional[float]:
        """
        Meminta dan mengambil data ketinggian (altitude) dari drone dengan perintah MAV_CMD_REQUEST_MESSAGE
        untuk GLOBAL_POSITION_INT, kemudian mengembalikan altitude relatif dalam meter.
        
        :return: Altitude relatif (meter) atau None jika data tidak tersedia.
        """
        request_message_command = dialect.MAVLink_command_long_message(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_REQUEST_MESSAGE,
            0,  # confirmation
            dialect.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0   # param7
        )
        self.vehicle.mav.send(request_message_command)
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
        logging.debug("Pesan GLOBAL_POSITION_INT: %s", msg)
        if msg:
            alt_relative = msg.relative_alt / 1000.0
            return alt_relative
        else:
            logging.warning("Tidak menerima data ketinggian (GLOBAL_POSITION_INT).")
            return None

    # ----------------- RC Override dan Gerakan Dasar -----------------

    def safe_pwm(self, channel: int, offset: int) -> None:
        """
        Menyesuaikan nilai PWM pada channel tertentu dengan memastikan nilai
        tetap dalam rentang [min_pwm, max_pwm].
        
        :param channel: Nomor channel (1..4)
        :param offset: Offset yang ditambahkan ke nilai channel
        """
        new_val = self.channels[channel] + offset
        new_val = max(self.min_pwm, min(new_val, self.max_pwm))
        self.channels[channel] = new_val
        logging.debug("Channel %d override: %d", channel, self.channels[channel])

    def override_channels(self) -> None:
        """
        Mengirim perintah RC override untuk channel 1-4.
        """
        temp = [65535] * 18  # Inisialisasi semua channel dengan nilai default
        for i in range(1, 5):
            temp[i - 1] = self.channels[i]
        msg = dialect.MAVLink_rc_channels_override_message(
            self.vehicle.target_system,
            self.vehicle.target_component,
            *temp
        )
        self.vehicle.mav.send(msg)

    def update_channels(self) -> None:
        """
        Mengirim nilai channel override secara rutin.
        """
        self.override_channels()

    def move_left_right(self, pwm_offset: int) -> None:
        """
        Menggerakkan drone ke kiri/kanan via RC override (Channel 1 = Roll).
        
        :param pwm_offset: Offset PWM (+ untuk kanan, - untuk kiri)
        """
        self.safe_pwm(channel=1, offset=pwm_offset)
        self.override_channels()

    def move_forward_backward(self, pwm_offset: int) -> None:
        """
        Menggerakkan drone maju/mundur via RC override (Channel 2 = Pitch).
        
        :param pwm_offset: Offset PWM (+ untuk maju, - untuk mundur)
        """
        self.safe_pwm(channel=2, offset=pwm_offset)
        self.override_channels()

    def throttle_up_down(self, pwm_offset: int) -> None:
        """
        Menggerakkan drone naik/turun via RC override (Channel 3 = Throttle).
        
        :param pwm_offset: Offset PWM (+ untuk naik, - untuk turun)
        """
        self.safe_pwm(channel=3, offset=pwm_offset)
        self.override_channels()

    def yaw_left_right(self, pwm_offset: int) -> None:
        """
        Menggerakkan drone yaw kiri/kanan via RC override (Channel 4 = Yaw).
        
        :param pwm_offset: Offset PWM (+ untuk yaw kanan, - untuk yaw kiri)
        """
        self.safe_pwm(channel=4, offset=pwm_offset)
        self.override_channels()

    # ----------------- Pengaturan Mode Penerbangan -----------------

    def _set_mode(self, custom_mode: int, mode_name: str) -> None:
        """
        Metode internal untuk mengubah mode penerbangan.
        
        :param custom_mode: Nilai custom_mode untuk mode yang diinginkan.
        :param mode_name: Nama mode (untuk keperluan logging).
        """
        base_mode = utility.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        logging.info("Mengatur mode ke %s...", mode_name)
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            base_mode,
            custom_mode
        )
        timeout = 5
        start_time = time.time()
        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
            if msg:
                if msg.custom_mode == custom_mode:
                    logging.info("Mode diubah ke %s.", mode_name)
                    break
                else:
                    logging.debug("Mode saat ini: %s. Menunggu mode %s...", msg.custom_mode, mode_name)
            else:
                logging.warning("Timeout menunggu heartbeat mode change.")
                break

            if time.time() - start_time > timeout:
                logging.warning("Timeout total tercapai saat menunggu mode %s.", mode_name)
                break

            time.sleep(0.5)

    def set_mode_guided(self) -> None:
        """
        Mengubah mode penerbangan menjadi GUIDED (custom_mode = 4 untuk ArduCopter).
        """
        self._set_mode(custom_mode=4, mode_name="GUIDED")

    def set_mode_stabilize(self) -> None:
        """
        Mengubah mode penerbangan menjadi STABILIZE (custom_mode = 0 untuk ArduCopter).
        """
        self._set_mode(custom_mode=0, mode_name="STABILIZE")

    def set_mode_alt_hold(self) -> None:
        """
        Mengubah mode penerbangan menjadi ALT_HOLD (custom_mode = 3 untuk ArduCopter).
        """
        self._set_mode(custom_mode=3, mode_name="ALT_HOLD")

    # ----------------- Perintah ARM, Takeoff, dan Landing -----------------

    def arm_drone(self) -> None:
        """
        Men-arm drone agar motor aktif dan siap takeoff.
        """
        logging.info("Menjalankan perintah ARM...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and (msg.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logging.info("Drone sudah ARM.")
                break
            logging.info("Menunggu drone ARM...")
            time.sleep(0.5)

    def takeoff(self, target_altitude: float = 10) -> None:
        """
        Memerintahkan drone untuk lepas landas (takeoff) ke ketinggian tertentu.
        
        :param target_altitude: Ketinggian target dalam meter.
        """
        logging.info("Takeoff ke %s meter...", target_altitude)
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude
        )

    def land_drone(self) -> None:
        """
        Memerintahkan drone untuk mendarat (LAND) dan menunggu hingga drone otomatis disarm.
        """
        logging.info("Memulai proses pendaratan (LAND)...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            utility.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and not (msg.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logging.info("Drone telah mendarat dan disarm.")
                break
            logging.info("Menunggu drone mendarat dan disarm...")
            time.sleep(1)

    # ----------------- Pengiriman Perintah Kecepatan / Posisi -----------------

    def send_velocity_command(self, vel_x: float, vel_y: float, vel_z: float) -> None:
        """
        Mengirim setpoint kecepatan dalam koordinat BODY_NED.
        
        :param vel_x: Kecepatan sumbu X (m/s), positif = maju.
        :param vel_y: Kecepatan sumbu Y (m/s), positif = ke kanan.
        :param vel_z: Kecepatan sumbu Z (m/s), positif = turun.
        """
        msg = self.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms (tidak digunakan)
            self.vehicle.target_system,
            self.vehicle.target_component,
            9,  # MAV_FRAME_BODY_OFFSET_NED
            0x0DC7,  # type_mask: hanya kecepatan yang diatur
            0, 0, 0,  # posisi (abaikan)
            vel_x, vel_y, vel_z,  # kecepatan (m/s)
            0, 0, 0,  # akselerasi (abaikan)
            0, 0     # yaw, yaw_rate (abaikan)
        )
        self.vehicle.mav.send(msg)

    def send_body_offset_position_xy(self, offset_x: float, offset_y: float, z_offset: float, duration: float = 0.1) -> None:
        """
        Mengirim perintah offset posisi relatif terhadap posisi dan heading saat ini
        menggunakan MAV_FRAME_BODY_OFFSET_NED.
        
        :param offset_x: Offset sumbu X (meter, positif = maju)
        :param offset_y: Offset sumbu Y (meter, positif = ke kanan)
        :param z_offset: Offset vertikal (meter, positif = turun)
        :param duration: Durasi perintah (detik) sebelum perintah berikutnya dikirim.
        """
        msg = self.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms (tidak digunakan)
            self.vehicle.target_system,
            self.vehicle.target_component,
            9,  # MAV_FRAME_BODY_OFFSET_NED
            0b0000111111111000,  # type_mask: hanya posisi yang diatur
            offset_x, offset_y, z_offset,  # Offset posisi (meter)
            0, 0, 0,  # kecepatan (abaikan)
            0, 0, 0,  # akselerasi (abaikan)
            0, 0     # yaw, yaw_rate (abaikan)
        )
        self.vehicle.mav.send(msg)
        time.sleep(duration)

    def close_connection(self) -> None:
        """
        Menutup koneksi MAVLink dan menghentikan thread pembaca altitude.
        """
        self._altitude_thread_running = False
        self._altitude_thread.join(timeout=1)
        self.vehicle.close()
        logging.info("Koneksi MAVLink ditutup.")


if __name__ == "__main__":
    # Contoh inisialisasi dan pengujian
    logging.basicConfig(level=logging.DEBUG,
                        format="%(asctime)s [%(levelname)s] %(message)s")
    drone = DroneController(connection_string="udp:127.0.0.1:14550")
    
    # Uji pembacaan altitude selama 10 detik
    for _ in range(100):
        alt = drone.get_latest_altitude()
        if alt is not None:
            logging.info(f"Altitude: {alt:.2f} m")
        else:
            logging.info("Altitude belum tersedia.")
        time.sleep(0.1)
    
    drone.close_connection()
