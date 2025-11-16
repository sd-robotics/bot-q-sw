#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import Adafruit_PCA9685
import time

# PCA9685初期設定
pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=7)
# pwm.set_pwm_freq(60)
pwm.set_pwm_freq(50)

ALL_MOTORS = [0, 1, 2, 3, 4, 5, 6, 7]
KEY_MAP = {
    #    "w": [1, 2, 5, 7],  # forward
    #    "a": [4, 5, 6, 7],  # left
    #    "s": [0, 3, 4, 6],  # back
    #    "d": [0, 1, 2, 3],  # right

    #    "z": [0, 3, 5, 7],  # turn_right
    #    "x": [1, 2, 4, 6],  # turn_left

    "w": [0, 2, 5, 7],  # forward
    "a": [4, 5, 6, 7],  # left
    "s": [1, 3, 4, 6],  # back
    "d": [0, 1, 3, 2],  # right

    "z": [1, 3, 5, 7],  # turn_right
    "x": [0, 2, 4, 6],  # turn_left
}


DUTY_MIN = 200
DUTY_MID = 300
DUTY_MAX = 400

POWER_TYP = 25


def drive_motors(motors, power_percent):
    duty = int( DUTY_MIN + (DUTY_MAX-DUTY_MIN) * power_percent/100 )
    for i_motor in motors:
        pwm.set_pwm(int(i_motor), 0, duty)

def _main():
    time.sleep(2)
    drive_motors(ALL_MOTORS, 100)
    time.sleep(4)
    drive_motors(ALL_MOTORS,   0)
    time.sleep(4)

    while True:
        print()
        print("press key.")
        print("1. [(+/-)0-7]    -> Test each motor. +:MAX, -:OFF, netral:", str(POWER_TYP)+"%")
        print("2. [w,a,s,d,z,x] -> Move")
        print("3. [h,l]         -> All motor Max(h) or OFF(l)")
        print()

        try:

            pressed_key = input("input:")

            if pressed_key.isdecimal():
                print("Test motor:", pressed_key[0])
                drive_motors([pressed_key], POWER_TYP)

            elif pressed_key[0] == "+" and pressed_key[1].isdecimal():
                print("MAX motor:", pressed_key[1])
                drive_motors([pressed_key[1]], 100)

            elif pressed_key[0] == "-" and pressed_key[1].isdecimal():
                print("stop motor:", pressed_key[1])
                drive_motors([pressed_key[1]],   0)

            elif pressed_key == "h":
                drive_motors(ALL_MOTORS, 100)

            elif pressed_key == "l":
                drive_motors(ALL_MOTORS,   0)

            elif pressed_key in KEY_MAP:
                drive_motors(ALL_MOTORS,   0)
                drive_motors(KEY_MAP[pressed_key], POWER_TYP)

            else:
                print("wrong input")
                print("\n\n")

        except Exception:
            drive_motors(ALL_MOTORS,   0)
            print("Quit")
            break


if __name__ == "__main__":
    _main()
