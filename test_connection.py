#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script untuk menghubungkan ke drone menggunakan dronekit.

Kode ini memeriksa versi Python, melakukan patch terhadap modul collections
jika diperlukan (untuk kompatibilitas Python 3.10 ke atas), dan menghubungkan
ke drone melalui port serial.
"""

import sys

# Patch untuk kompatibilitas Python 3.10+ jika collections.MutableMapping tidak ada
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    from collections.abc import MutableMapping
    setattr(collections, "MutableMapping", MutableMapping)

import dronekit
import time

# Atur string koneksi; Uncomment sesuai kebutuhan
# connection_string = 'udp:127.0.0.1:14550'
connection_string = '/dev/ttyAMA0'
baud_rate = 57600  # Typically used for serial; adjust if necessary

print("Connecting to STIL on: {}".format(connection_string))

# Hubungkan ke vehicle dengan parameter koneksi dan baud rate yang telah ditentukan
vehicle = dronekit.connect(ip=connection_string, baud=baud_rate, wait_ready=True)

# Konfirmasi koneksi dengan menampilkan beberapa atribut vehicle
print("Vehicle Mode: {}".format(vehicle.mode.name))
print("Global Location: {}".format(vehicle.location.global_frame))

# Contoh: Mengubah mode ke GUIDED (Uncomment jika diperlukan)
# from dronekit import VehicleMode
# vehicle.mode = VehicleMode("GUIDED")
# time.sleep(2)
# print("Mode changed to: {}".format(vehicle.mode.name))

# Pastikan untuk menutup koneksi setelah selesai
vehicle.close()
