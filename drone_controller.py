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
"""

import time
import sys
import logging

import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect


class DroneController:
    """
    Kelas untuk mengontrol drone melalui MAVLink. Memungkinkan berbagai aksi
    seperti override RC, set mode, arm/drone, takeoff, dan land.
    """

    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        """
        Inisialisasi drone controller dengan string koneksi MAVLink.

        :param connection_string: Alamat/koneksi MAVLink
                                 (mis. "udp:127.0.0.1:14550" atau "/dev/ttyACM0")
        """
        logging.info(f"Mencoba terhubung ke {connection_string}...")
        self.vehicle = utility.mavlink_connection(device=connection_string)
        try:
            self.vehicle.wait_heartbeat(timeout=10)
            logging.info("Heartbeat diterima!")
        except Exception as e:
            logging.error(f"Gagal menerima heartbeat: {e}")
            self.vehicle.close()
            sys.exit(1)

        logging.info(
            f"Terhubung ke system: {self.vehicle.target_system}, "
            f"component: {self.vehicle.target_component}"
        )

        # Channel defaults (neutral = 1500)
        self.channels = {1: 1500, 2: 1500, 3: 1500, 4: 1500}

        # Batas PWM
        self.min_pwm = 1000
        self.max_pwm = 2000

    def safe_pwm(self, channel, offset):
        """
        Menyesuaikan nilai PWM pada channel tertentu dengan memastikan nilai
        tetap di dalam rentang [min_pwm, max_pwm].

        :param channel: Nomor channel (1..4)
        :param offset: Jumlah offset yang ditambahkan ke channel
        """
        new_val = self.channels[channel] + offset
        new_val = max(self.min_pwm, min(new_val, self.max_pwm))
        self.channels[channel] = new_val
        logging.debug(f"Channel {channel} override: {self.channels[channel]}")

    def override_channels(self):
        """
        Mengirim perintah RC override untuk channel 1-4.
        Channel 5..18 akan diabaikan (65535 = ignore).
        """
        temp = [65535] * 18
        for i in range(1, 5):
            temp[i - 1] = self.channels[i]

        msg = dialect.MAVLink_rc_channels_override_message(
            self.vehicle.target_system,
            self.vehicle.target_component,
            *temp
        )
        self.vehicle.mav.send(msg)

    def update_channels(self):
        """
        Mengirim nilai channel terakhir (override) secara rutin.
        Dipanggil misalnya di setiap iterasi loop utama.
        """
        self.override_channels()

    # ----------------- Gerakan Dasar (RC Override) -----------------
    def move_left_right(self, pwm_offset):
        """
        Menggerakkan drone ke kiri/kanan via RC override (Channel 1 = Roll).
        :param pwm_offset: Offset PWM (+ untuk kanan, - untuk kiri)
        """
        self.safe_pwm(channel=1, offset=pwm_offset)
        self.override_channels()

    def move_forward_backward(self, pwm_offset):
        """
        Menggerakkan drone maju/mundur via RC override (Channel 2 = Pitch).
        :param pwm_offset: Offset PWM (+ untuk maju, - untuk mundur)
        """
        self.safe_pwm(channel=2, offset=pwm_offset)
        self.override_channels()

    def throttle_up_down(self, pwm_offset):
        """
        Menggerakkan drone naik/turun via RC override (Channel 3 = Throttle).
        :param pwm_offset: Offset PWM (+ untuk naik, - untuk turun)
        """
        self.safe_pwm(channel=3, offset=pwm_offset)
        self.override_channels()

    def yaw_left_right(self, pwm_offset):
        """
        Menggerakkan drone yaw kiri/kanan via RC override (Channel 4 = Yaw).
        :param pwm_offset: Offset PWM (+ untuk yaw kanan, - untuk yaw kiri)
        """
        self.safe_pwm(channel=4, offset=pwm_offset)
        self.override_channels()

    # ----------------- Velocity Control -----------------
    def send_velocity_command(self, vel_x, vel_y, vel_z):
        """
        Mengirim setpoint kecepatan dalam koordinat BODY_NED.

        :param vel_x: Kecepatan sumbu X (m/s), positif = maju
        :param vel_y: Kecepatan sumbu Y (m/s), positif = ke kanan
        :param vel_z: Kecepatan sumbu Z (m/s), positif = turun
        """
        # MAV_FRAME_BODY_NED = 8 => kecepatan relatif heading drone
        msg = self.vehicle.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            self.vehicle.target_system,
            self.vehicle.target_component,
            8,  # MAV_FRAME_BODY_NED
            0x0DC7,  # type_mask: kontrol kecepatan saja
            0, 0, 0,              # posisi (ignored)
            vel_x, vel_y, vel_z,  # kecepatan (m/s)
            0, 0, 0,              # akselerasi (ignored)
            0,                    # yaw (ignored)
            0                     # yaw_rate (ignored)
        )
        self.vehicle.mav.send(msg)

    # ----------------- Mode & Arm/Disarm -----------------
    def set_mode_guided(self):
        """
        Mengubah mode penerbangan menjadi GUIDED (ArduCopter custom_mode = 4).
        """
        base_mode = utility.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = 4  # GUIDED untuk ArduCopter
        logging.info("Mengatur mode ke GUIDED...")
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            base_mode,
            custom_mode
        )

        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                if msg.custom_mode == custom_mode:
                    logging.info("Mode diubah ke GUIDED.")
                    break
                else:
                    logging.info(f"Mode saat ini: {msg.custom_mode}. Menunggu GUIDED...")
            else:
                logging.warning("Timeout menunggu heartbeat mode change.")
                break
            time.sleep(0.5)

    def set_mode_stabilize(self):
        """
        Mengubah mode penerbangan menjadi STABILIZE (ArduCopter custom_mode = 0).
        Mode ini memungkinkan input RC manual.
        """
        base_mode = utility.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = 0  # STABILIZE
        logging.info("Mengubah mode ke STABILIZE...")
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            base_mode,
            custom_mode
        )

        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and msg.custom_mode == custom_mode:
                logging.info("Mode diubah ke STABILIZE.")
                break
            time.sleep(0.5)

    def set_mode_alt_hold(self):
        """
        Mengubah mode penerbangan menjadi ALT_HOLD (ArduCopter custom_mode = 3).
        Mode ini membuat autopilot mempertahankan ketinggian secara otomatis.
        """
        base_mode = utility.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = 3  # ALT_HOLD
        logging.info("Mengubah mode ke ALT_HOLD...")
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            base_mode,
            custom_mode
        )

        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and msg.custom_mode == custom_mode:
                logging.info("Mode diubah ke ALT_HOLD.")
                break
            time.sleep(0.5)

    def arm_drone(self):
        """
        Men-arm drone agar motor aktif dan siap takeoff.
        """
        logging.info("Menjalankan perintah ARM...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 => ARM
            0, 0, 0, 0, 0, 0
        )

        while True:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and (msg.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logging.info("Drone sudah ARM.")
                break
            logging.info("Menunggu drone ARM...")
            time.sleep(0.5)

    def takeoff(self, target_altitude=10):
        """
        Memerintahkan drone untuk lepas landas (takeoff) ke ketinggian tertentu.

        :param target_altitude: Ketinggian target (meter)
        """
        logging.info(f"Takeoff ke {target_altitude} meter...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude
        )

    def land_drone(self):
        """
        Memerintahkan drone untuk mendarat (LAND). Menunggu hingga disarm otomatis.
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
            if msg:
                # MAV_MODE_FLAG_SAFETY_ARMED = 128 => base_mode & 128 = 0 => disarmed
                if not (msg.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    logging.info("Drone telah mendarat dan disarm.")
                    break
            logging.info("Menunggu drone mendarat dan disarm...")
            time.sleep(1)

    def close_connection(self):
        """
        Menutup koneksi MAVLink dengan drone.
        """
        self.vehicle.close()
        logging.info("Koneksi MAVLink ditutup.")
