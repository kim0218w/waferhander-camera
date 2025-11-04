#!/usr/bin/env python3
import lgpio
import time

# BCM 번호 기준
PUL_PIN = 11  # pul_pin (STEP/CLK) 
DIR_PIN = 13  # dir_pin+

STEP_DELAY = 0.001  # 펄스 간 간격 (초); 모터/드라이버에 맞춰 조정
STEPS = 200         # 한 방향으로 줄 펄스 수

# GPIO 칩 열기 (라즈베리파이의 기본 GPIO 칩은 0번)
# 반환값은 핸들(handle)입니다
chip = lgpio.gpiochip_open(0)

# 출력 핀 설정
# gpio_claim_output(handle, gpio) 또는 gpio_claim_output(handle, gpio, initial_level)
# initial_level이 지원되는 경우: 0=LOW, 1=HIGH
try:
    # 방법 1: initial 파라미터가 지원되는 경우
    lgpio.gpio_claim_output(chip, PUL_PIN, 0)  # LOW로 초기화
    lgpio.gpio_claim_output(chip, DIR_PIN, 0)   # LOW로 초기화
except TypeError:
    # 방법 2: initial 파라미터가 지원되지 않는 경우
    lgpio.gpio_claim_output(chip, PUL_PIN)
    lgpio.gpio_claim_output(chip, DIR_PIN)
    # 초기값을 LOW로 설정
    lgpio.gpio_write(chip, PUL_PIN, 0)
    lgpio.gpio_write(chip, DIR_PIN, 0)

def step(direction_high: bool):
    """
    모터 스텝 실행
    
    Args:
        direction_high: True면 정방향, False면 역방향
    """
    # 방향 설정 (1=HIGH, 0=LOW)
    lgpio.gpio_write(chip, DIR_PIN, 1 if direction_high else 0)
    
    # 스텝 펄스 생성
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

except Exception as e:
    print(f"Error occurred: {e}")
    import traceback
    traceback.print_exc()

finally:
    # GPIO 정리 (반드시 실행되어야 함)
    try:
        # 사용한 핀 해제
        lgpio.gpio_free(chip, PUL_PIN)
        lgpio.gpio_free(chip, DIR_PIN)
        # GPIO 칩 닫기
        lgpio.gpiochip_close(chip)
        print("GPIO cleaned up")
    except Exception as e:
        print(f"Cleanup error: {e}")

