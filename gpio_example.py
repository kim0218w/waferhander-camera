"""
Jetson Orin NX GPIO 제어 예제 코드

이 파일은 Jetson.GPIO 라이브러리를 사용하여 GPIO 핀을 제어하는 기본 예제입니다.
"""

import Jetson.GPIO as GPIO
import time

# GPIO 핀 번호 설정 (BCM 모드 사용)
# 실제 사용할 핀 번호로 변경하세요
LED_PIN = 18  # BCM 18번 핀 (예시)
BUTTON_PIN = 23  # BCM 23번 핀 (예시)

def setup():
    """GPIO 초기 설정"""
    # 경고 메시지 비활성화 (선택사항)
    GPIO.setwarnings(False)
    
    # 핀 번호 모드 설정
    # GPIO.BCM: Broadcom 핀 번호 사용 (논리적 번호)
    # GPIO.BOARD: 물리적 핀 번호 사용 (40핀 헤더 기준)
    GPIO.setmode(GPIO.BCM)
    
    # LED 핀을 출력 모드로 설정
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    # 버튼 핀을 입력 모드로 설정 (풀다운 저항 사용)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    print(f"GPIO 설정 완료!")
    print(f"LED 핀: BCM {LED_PIN}")
    print(f"버튼 핀: BCM {BUTTON_PIN}")

def blink_led(times=5, delay=0.5):
    """LED 깜빡이기 예제"""
    print(f"\nLED {times}번 깜빡이기 시작...")
    for i in range(times):
        GPIO.output(LED_PIN, GPIO.HIGH)
        print(f"  LED ON ({i+1}/{times})")
        time.sleep(delay)
        GPIO.output(LED_PIN, GPIO.LOW)
        print(f"  LED OFF ({i+1}/{times})")
        time.sleep(delay)
    print("LED 깜빡이기 완료!\n")

def read_button():
    """버튼 상태 읽기 예제"""
    print("\n버튼 상태 읽기 (Ctrl+C로 종료)...")
    try:
        while True:
            button_state = GPIO.input(BUTTON_PIN)
            if button_state == GPIO.HIGH:
                print("버튼이 눌렸습니다!")
                GPIO.output(LED_PIN, GPIO.HIGH)  # 버튼 눌리면 LED 켜기
            else:
                GPIO.output(LED_PIN, GPIO.LOW)  # 버튼 떼면 LED 끄기
            time.sleep(0.1)  # 100ms 대기
    except KeyboardInterrupt:
        print("\n버튼 읽기 종료")

def cleanup():
    """GPIO 정리 및 리소스 해제"""
    GPIO.cleanup()
    print("\nGPIO 정리 완료")

def main():
    """메인 함수"""
    try:
        # GPIO 설정
        setup()
        
        # 예제 1: LED 깜빡이기
        print("=" * 50)
        print("예제 1: LED 깜빡이기")
        print("=" * 50)
        blink_led(times=5, delay=0.5)
        
        # 예제 2: 버튼 상태 읽기 (주석 해제하여 사용)
        # print("=" * 50)
        # print("예제 2: 버튼 상태 읽기")
        # print("=" * 50)
        # read_button()
        
        # 예제 3: 단순한 ON/OFF 제어
        print("=" * 50)
        print("예제 3: LED 단순 제어")
        print("=" * 50)
        print("LED 켜기")
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(2)
        print("LED 끄기")
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(1)
        
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        # 항상 정리 작업 수행
        cleanup()

if __name__ == "__main__":
    main()

