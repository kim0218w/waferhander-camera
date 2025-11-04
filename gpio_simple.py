"""
Jetson Orin NX GPIO 간단한 예제

가장 기본적인 GPIO 제어 예제입니다.
"""

import Jetson.GPIO as GPIO
import time

# 사용할 GPIO 핀 번호 (BCM 모드)
PIN = 18  # 이 번호를 실제 사용할 핀 번호로 변경하세요

try:
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BCM)
    
    # 핀을 출력 모드로 설정
    GPIO.setup(PIN, GPIO.OUT)
    
    print(f"GPIO {PIN}번 핀으로 LED 제어 시작...")
    print("Ctrl+C를 눌러 종료하세요\n")
    
    # LED 깜빡이기 반복
    while True:
        GPIO.output(PIN, GPIO.HIGH)  # LED 켜기 (3.3V)
        print("LED ON")
        time.sleep(1)  # 1초 대기
        
        GPIO.output(PIN, GPIO.LOW)   # LED 끄기 (0V)
        print("LED OFF")
        time.sleep(1)  # 1초 대기

except KeyboardInterrupt:
    print("\n프로그램 종료 중...")
    
except Exception as e:
    print(f"오류: {e}")
    
finally:
    # GPIO 정리 (중요!)
    GPIO.cleanup()
    print("GPIO 정리 완료")

