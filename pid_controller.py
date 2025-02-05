#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pid_controller.py
-----------------
Modul ini menyediakan kelas PIDController untuk mengimplementasikan algoritma
kontrol PID (Proportional-Integral-Derivative) yang umum digunakan dalam sistem kontrol.
"""


class PIDController:
    """
    Implementasi PID Controller.

    Atribut:
        Kp (float): Koefisien proporsional.
        Ki (float): Koefisien integral.
        Kd (float): Koefisien derivatif.
        setpoint (float): Nilai target yang diinginkan.
        sample_time (float): Waktu sampling dalam detik.
        output_limits (tuple): Batas bawah dan atas output (None berarti tidak ada batas).
    """

    def __init__(self, Kp, Ki, Kd, setpoint=0, sample_time=0.1, output_limits=(None, None)):
        """
        Inisialisasi PIDController dengan parameter PID dan setpoint.

        :param Kp: Koefisien proporsional.
        :param Ki: Koefisien integral.
        :param Kd: Koefisien derivatif.
        :param setpoint: Nilai target yang diinginkan (default: 0).
        :param sample_time: Waktu sampling dalam detik (default: 0.1).
        :param output_limits: Tuple (lower, upper) untuk batas output (default: (None, None)).
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.output_limits = output_limits
        self._last_error = 0
        self._integral = 0

    def compute(self, measurement):
        """
        Menghitung output PID berdasarkan kesalahan antara setpoint dan nilai pengukuran.

        Langkah perhitungan:
          1. Hitung error = setpoint - measurement.
          2. Perbarui nilai integral dengan error dikali sample_time.
          3. Hitung derivatif sebagai perubahan error per satuan waktu.
          4. Hitung output berdasarkan gabungan komponen P, I, dan D.
          5. Terapkan batas output sesuai dengan output_limits.

        :param measurement: Nilai pengukuran saat ini.
        :return: Output PID yang telah dikalkulasi.
        """
        error = self.setpoint - measurement
        self._integral += error * self.sample_time
        derivative = (error - self._last_error) / self.sample_time if self.sample_time > 0 else 0

        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
        self._last_error = error

        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        return output
