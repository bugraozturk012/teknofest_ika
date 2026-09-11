#!/usr/bin/env python3
"""
pid_controller.py — Generic PID Controller for Turret & Motion Control
"""

import time   # süre ölçümleri time.monotonic() ile: Jetson'ın RTC'si ölü ve
             # saat düzeltmesi sıçradığında time.time() aralıkları
             # milyonlarca saniye okunur (ayrıntı: misyon_fsm.py)


class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float,
                 integral_limit: float = 1000.0,
                 output_limit: tuple = (-1000.0, 1000.0)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.monotonic()
        self._first_run = True

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def compute(self, error: float) -> float:
        now = time.monotonic()
        dt = now - self._prev_time
        self._prev_time = now

        if self._first_run:
            self._first_run = False
            derivative = 0.0
        else:
            if dt <= 0.0:
                dt = 1e-3
            derivative = (error - self._prev_error) / dt

        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))

        output = (self.Kp * error +
                  self.Ki * self._integral +
                  self.Kd * derivative)

        self._prev_error = error

        output = max(self.output_limit[0],
                     min(self.output_limit[1], output))
        return output
