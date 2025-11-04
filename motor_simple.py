"""
Jetson Orin NX 스테퍼 모터 제어 - 간단한 버전

PUL 핀: 펄스 신호 (스텝 신호)
DIR 핀: 방향 제어 (HIGH = 정방향, LOW = 역방향)
"""

import Jetson.GPIO as GPIO
import time

# ===== 핀 번호 설정 (실제 사용할 핀 번호로 변경하세요) =====
PUL_PIN = 18   # 펄스 핀 (스텝 신호)
DIR_PIN = 23   # 방향 핀

# ===== 모터 설정 =====
STEP_DELAY = 0.001  # 스텝 간격 (초) - 작을수록 빠름 (0.0005 ~ 0.01 권장)

def setup():
    """GPIO 초기 설정"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # PUL 핀: 출력 모드, 초기값 LOW
    GPIO.setup(PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    # DIR 핀: 출력 모드, 초기값 LOW
    GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    print(f"모터 제어 설정 완료!")
    print(f"PUL 핀: GPIO {PUL_PIN}")
    print(f"DIR 핀: GPIO {DIR_PIN}")

def rotate_steps(steps, direction=True):
    """
    모터를 지정된 스텝 수만큼 회전
    
    Args:
        steps: 회전할 스텝 수
        direction: True = 정방향(+), False = 역방향(-)
    """
    # 방향 설정
    if direction:
        GPIO.output(DIR_PIN, GPIO.HIGH)  # 정방향 (+)
        print(f"정방향(+) {steps} 스텝 회전 시작...")
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)   # 역방향 (-)
        print(f"역방향(-) {steps} 스텝 회전 시작...")
    
    time.sleep(0.01)  # 방향 변경 후 짧은 대기
    
    # 펄스 신호 생성
    for i in range(steps):
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(STEP_DELAY)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(STEP_DELAY)
    
    print(f"회전 완료! ({steps} 스텝)")

def cleanup():
    """GPIO 정리"""
    GPIO.cleanup()
    print("\nGPIO 정리 완료")

def main():
    """메인 함수"""
    try:
        setup()
        
        print("\n" + "="*50)
        print("모터 제어 테스트")
        print("="*50)
        
        # 예제 1: 정방향(+) 100 스텝 회전
        print("\n[예제 1] 정방향(+) 100 스텝")
        rotate_steps(steps=100, direction=True)
        time.sleep(1)
        
        # 예제 2: 역방향(-) 100 스텝 회전
        print("\n[예제 2] 역방향(-) 100 스텝")
        rotate_steps(steps=100, direction=False)
        time.sleep(1)
        
        # 예제 3: 정방향(+) 500 스텝 (더 많이 회전)
        print("\n[예제 3] 정방향(+) 500 스텝")
        rotate_steps(steps=500, direction=True)
        time.sleep(1)
        
        print("\n테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n\n프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
