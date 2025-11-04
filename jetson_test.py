#!/usr/bin/env python3
import lgpio
import time

# BCM 번호 기준
PUL_PIN = 11  # pul_pin (STEP/CLK) bcm 11
DIR_PIN = 13  # dir_pin+ bcm 13

STEP_DELAY = 0.001  # 펄스 간 간격 (초); 모터/드라이버에 맞춰 조정
STEPS = 200         # 한 방향으로 줄 펄스 수

# GPIO 칩 열기 (라즈베리파이의 기본 GPIO 칩은 0번)
chip = lgpio.gpiochip_open(0)

# 출력 핀 설정 (initial=0은 LOW를 의미)
lgpio.gpio_claim_output(chip, PUL_PIN, initial=0)
lgpio.gpio_claim_output(chip, DIR_PIN, initial=0)

def step(direction_high: bool):
    # 방향 설정 (1=HIGH, 0=LOW)
    lgpio.gpio_write(chip, DIR_PIN, 1 if direction_high else 0)
    
    for _ in range(STEPS):
        lgpio.gpio_write(chip, PUL_PIN, 1)  # HIGH
        time.sleep(STEP_DELAY / 2)
        lgpio.gpio_write(chip, PUL_PIN, 0)  # LOW
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
    # GPIO 정리
    lgpio.gpiochip_close(chip)
    print("GPIO cleaned up")

