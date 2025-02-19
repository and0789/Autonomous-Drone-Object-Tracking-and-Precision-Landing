#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_controller.py
-----------------
Modul ini menyediakan kelas PIDController untuk mengimplementasikan algoritma
kontrol PID (Proportional-Integral-Derivative) yang umum digunakan dalam sistem kontrol.
Selain itu, modul ini juga menyediakan kelas AdaptivePIDController yang merupakan ekstensi
dari PIDController dengan fitur anti-windup dan adaptive gain scheduling.
"""

from typing import Tuple, Optional

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

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        setpoint: float = 0,
        sample_time: float = 0.1,
        output_limits: Tuple[Optional[float], Optional[float]] = (None, None)
    ) -> None:
        """
        Inisialisasi PIDController dengan parameter PID dan setpoint.

        :param Kp: Koefisien proporsional.
        :param Ki: Koefisien integral.
        :param Kd: Koefisien derivatif.
        :param setpoint: Nilai target yang diinginkan (default: 0).
        :param sample_time: Waktu sampling dalam detik (default: 0.1).
        :param output_limits: Tuple (lower, upper) untuk batas output (default: (None, None)).
        """
        self.Kp: float = Kp
        self.Ki: float = Ki
        self.Kd: float = Kd
        self.setpoint: float = setpoint
        self.sample_time: float = sample_time
        self.output_limits: Tuple[Optional[float], Optional[float]] = output_limits
        self._last_error: float = 0.0
        self._integral: float = 0.0

    def compute(self, measurement: float) -> float:
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
        error: float = self.setpoint - measurement

        # Update integral (dengan sample_time sebagai waktu pengali)
        self._integral += error * self.sample_time

        # Hitung nilai derivatif
        derivative: float = (error - self._last_error) / self.sample_time if self.sample_time > 0 else 0.0

        # Hitung output PID
        output: float = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        self._last_error = error

        # Terapkan batas output jika didefinisikan
        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        return output

    def reset(self) -> None:
        """
        Reset nilai integral dan last error ke 0.
        Metode ini berguna ketika ingin menghapus history error sebelumnya,
        misalnya saat terjadi perubahan setpoint atau kondisi reset sistem.
        """
        self._integral = 0.0
        self._last_error = 0.0


class AdaptivePIDController(PIDController):
    """
    Ekstensi PIDController dengan mekanisme anti-windup dan adaptive gain scheduling.

    Atribut tambahan:
        integral_limits (tuple): Batas bawah dan atas nilai integral untuk mencegah windup.
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        setpoint: float = 0,
        sample_time: float = 0.1,
        output_limits: Tuple[Optional[float], Optional[float]] = (None, None),
        integral_limits: Tuple[float, float] = (-10, 10)
    ) -> None:
        """
        Inisialisasi AdaptivePIDController dengan parameter PID, setpoint, dan batas integral.

        :param Kp: Koefisien proporsional.
        :param Ki: Koefisien integral.
        :param Kd: Koefisien derivatif.
        :param setpoint: Nilai target yang diinginkan (default: 0).
        :param sample_time: Waktu sampling dalam detik (default: 0.1).
        :param output_limits: Tuple (lower, upper) untuk batas output (default: (None, None)).
        :param integral_limits: Tuple (lower, upper) untuk batas nilai integral (default: (-10, 10)).
        """
        super().__init__(Kp, Ki, Kd, setpoint, sample_time, output_limits)
        self.integral_limits: Tuple[float, float] = integral_limits

    def compute(self, measurement: float) -> float:
        """
        Menghitung output PID dengan mekanisme anti-windup.

        Proses perhitungan:
          1. Hitung error = setpoint - measurement.
          2. Perbarui nilai integral dengan error dikali sample_time.
          3. Lakukan clamping (pembatasan) pada nilai integral agar tidak melebihi batas yang ditentukan.
          4. Hitung derivatif sebagai perubahan error per satuan waktu.
          5. Hitung output sebagai gabungan komponen P, I, dan D.
          6. Terapkan batas output sesuai dengan output_limits.

        :param measurement: Nilai pengukuran saat ini.
        :return: Output Adaptive PID yang telah dikalkulasi.
        """
        error = self.setpoint - measurement
        self._integral += error * self.sample_time
        # Anti-windup: clamp nilai integral
        lower_int, upper_int = self.integral_limits
        self._integral = max(lower_int, min(self._integral, upper_int))
        derivative = (error - self._last_error) / self.sample_time if self.sample_time > 0 else 0.0
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        self._last_error = error

        # Terapkan batas output
        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        return output
