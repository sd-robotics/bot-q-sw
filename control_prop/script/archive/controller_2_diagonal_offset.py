#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import Adafruit_PCA9685
import argparse
from inputs import get_gamepad
import threading
import time

class ElecomGamepad:
    BUTTON_MAP = {
        "BTN_C": "A",
        "BTN_NORTH": "B",
        "BTN_SOUTH": "X",
        "BTN_EAST": "Y",
        "BTN_WEST": "LB",
        "BTN_TL": "LT",
        "BTN_Z": "RB",
        "BTN_TR": "RT",
        "BTN_TL2": "LS",
        "BTN_TR2": "RS",
        "BTN_SELECT": "BACK",
        "BTN_START": "START",
        "BTN_MODE": "GUIDE"
    }
    AXIS_MAP = {
        "ABS_X": "LX",
        "ABS_Y": "LY",
        "ABS_RZ": "RX",
        "ABS_Z": "RY",

        "ABS_RX": "RX",
        "ABS_RY": "RY",
        "ABS_HAT0X": "CX",
        "ABS_HAT0Y": "CY",
    }

    def __init__(self):
        self.buttons = {
            "A": 0,
            "B": 0,
            "X": 0,
            "Y": 0,
            "LB": 0,
            "LT": 0,
            "RB": 0,
            "RT": 0,
            "RS": 0,
            "RS": 0,
            "BACK": 0,
            "START": 0,
            "GUIDE": 0,

        }
        self.axes = {
            "LX": 0,
            "LY": 0,
            "RX": 0,
            "RY": 0,
            "CX": 0,
            "CY": 0,
        }
        self._t_last_update = time.time()
        self._running = True

        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()
    def _update_loop(self):
        while self._running:
            events = get_gamepad()
            for event in events:
                # print(event.code, event.state)
                self._t_last_update = time.time()
                if event.ev_type == 'Key':
                    self.buttons[self.BUTTON_MAP[event.code]] = event.state
                elif event.ev_type == 'Absolute':
                    event_axis = self.AXIS_MAP[event.code]
                    if event_axis in ["LX", "LY", "RX", "RY"]:
                        self.axes[event_axis] = event.state/256 - 0.5
                    else:
                        self.axes[event_axis] = event.state/256

    def get_button(self, name):
        code = self.buttons.get(name)
        return self.buttons.get(code, 0)

    def get_axis(self, name):
        code = self.axes.get(name)
        return self.axes.get(code, 0)

    def stop(self):
        self._running = False
        self._thread.join()

    def time_since_last_input(self):
        return round(time.time() - self._t_last_update, 3)

class FanControl:
    DUTY_MIN = 205
    DUTY_MID = 300
    DUTY_MAX = 410

    POWER_TYP = 10

    # ALL_MOTORS = [0, 1, 2, 3, 4, 5, 6, 7]

    N_MOTOR = 8

    POWER_LIMIT = 1

    MOTOR_MAP = {
        "forward": [1, 3, 5, 7],  # forward
        "left": [4, 5, 6, 7],  # left
        "back": [0, 2, 4, 6],  # back
        "right": [0, 1, 3, 2],  # right

        "turn_right": [0, 2, 5, 7],  # turn_right
        "turn_left": [1, 3, 4, 6],  # turn_left

    }

    DIAGONAL_MOTOR_SETS = [(0,5), (1,4), (2,7), (3,6)]

    def __init__(self, controller=None):
        self.x = 0
        self.y = 0
        self.yaw = 0

        self.motor_powers = [0] * self.N_MOTOR

        self.controller = controller or ElecomGamepad()

        self._pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=7)
        self._pwm.set_pwm_freq(50)

        time.sleep(2)
        self.drive_all_motor(1)
        time.sleep(4)
        self.drive_all_motor(0)
        time.sleep(4)

    def __del__(self):
        print("[DEL] FanControl.")
        self.drive_all_motor(0)

    # 0 < power_ratio < 1

    def diagonal_offset(self):
        for motor_set in self.DIAGONAL_MOTOR_SETS:
            duplicated_power = self.motor_powers[motor_set[0]] if self.motor_powers[motor_set[0]] < self.motor_powers[motor_set[1]] else self.motor_powers[motor_set[1]]
            self.motor_powers[motor_set[0]] -= duplicated_power
            self.motor_powers[motor_set[1]] -= duplicated_power

    def drive_motors(self, motors, power_ratio):
        pass
        # duty = int( self.DUTY_MIN + (self.DUTY_MAX-self.DUTY_MIN) * power_ratio )
        # for i_motor in motors:
        #     _pwm.set_pwm(int(i_motor), 0, duty)
    def drive_all_motor(self, power_ratio):
        for i in range(self.N_MOTOR):
            self.drive_motor(i, power_ratio)

    def drive_motor(self, motor, power_ratio):
        duty = int( self.DUTY_MIN + (self.DUTY_MAX-self.DUTY_MIN) * power_ratio )
        self._pwm.set_pwm(int(motor), 0, duty)

    def control(self):
        for i in range(self.N_MOTOR):
            self.motor_powers[i] = 0

        self.x = -self.controller.axes["LY"]
        self.y = -self.controller.axes["LX"]
        self.yaw = -self.controller.axes["RX"]

        if 0 < self.x:
            for i_motor in self.MOTOR_MAP["forward"]:
                self.motor_powers[i_motor] += self.x / 3
        else:
            for i_motor in self.MOTOR_MAP["back"]:
                self.motor_powers[i_motor] -= self.x / 3

        if 0 < self.y:
            for i_motor in self.MOTOR_MAP["left"]:
                self.motor_powers[i_motor] += self.y / 3
        else:
            for i_motor in self.MOTOR_MAP["right"]:
                self.motor_powers[i_motor] -= self.y / 3

        if 0 < self.yaw:
            for i_motor in self.MOTOR_MAP["turn_left"]:
                self.motor_powers[i_motor] += self.yaw / 3
        else:
            for i_motor in self.MOTOR_MAP["turn_right"]:
                self.motor_powers[i_motor] -= self.yaw / 3

        self.diagonal_offset()

        print("UNIX time:", time.time())
        print("\nx, y, yaw:", self.x, self.y, self.yaw)
        print("motor_powers:", *self.motor_powers)

        for i_motor in range(self.N_MOTOR):
            self.drive_motor(i_motor, self.motor_powers[i_motor]*self.POWER_LIMIT)


def _main():
    parser = argparse.ArgumentParser(description="explanation of this code")
    parser.add_argument('-num',  type=int, default=0, help='int number')
    args = parser.parse_args()

    # pad = ElecomGamepad()

    fan_control = FanControl()

    while True:
        #         print("BUT :", pad.buttons)
        #         print("AXIS:", pad.axes)
        #         print("since:", pad.time_since_last_input())
        #         print()
        #
        fan_control.control()

        time.sleep(0.1)


if __name__ == "__main__":
    _main()
