#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time

# BCM 번호 기준
PUL_PIN = 11  # pul_pin (STEP/CLK) bcm 31
DIR_PIN = 13   # dir_pin+ bcm 33

STEP_DELAY = 0.001  # 펄스 간 간격 (초); 모터/드라이버에 맞춰 조정
STEPS = 200         # 한 방향으로 줄 펄스 수

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)

def step(direction_high: bool):
    GPIO.output(DIR_PIN, GPIO.HIGH if direction_high else GPIO.LOW)
    for _ in range(STEPS):
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(STEP_DELAY / 2)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(STEP_DELAY / 2)

try:
    while True:
        print("Clockwise")
        step(direction_high=True)   # 시계 방향 (드라이버에 따라 HIGH/LOW 의미가 반대일 수 있음)
        time.sleep(0.5)

        print("Counter-clockwise")
        step(direction_high=False)  # 반시계 방향
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up")