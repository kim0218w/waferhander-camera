"""
Jetson Orin NX 스테퍼 모터 제어 - 클래스 기반 상세 버전

PUL 핀: 펄스 신호 (스텝 신호)
DIR 핀: 방향 제어
  - HIGH (3.3V) = 정방향 회전 (+)
  - LOW (0V) = 역방향 회전 (-)
"""

import Jetson.GPIO as GPIO
import time

class StepperMotor:
    """스테퍼 모터 제어 클래스"""
    
    def __init__(self, pul_pin, dir_pin, step_delay=0.001):
        """
        모터 초기화
        
        Args:
            pul_pin: 펄스 핀 번호 (BCM)
            dir_pin: 방향 핀 번호 (BCM)
            step_delay: 스텝 간격 (초) - 기본값 0.001초
        """
        self.pul_pin = pul_pin
        self.dir_pin = dir_pin
        self.step_delay = step_delay
        self.current_direction = True  # True = 정방향, False = 역방향
        
        # GPIO 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pul_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        
        print(f"StepperMotor 초기화 완료")
        print(f"  PUL 핀: GPIO {self.pul_pin}")
        print(f"  DIR 핀: GPIO {self.dir_pin}")
        print(f"  스텝 간격: {self.step_delay}초")
    
    def set_direction(self, direction):
        """
        모터 방향 설정
        
        Args:
            direction: True = 정방향(+), False = 역방향(-)
        """
        if direction:
            GPIO.output(self.dir_pin, GPIO.HIGH)  # 정방향 (+)
            self.current_direction = True
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)    # 역방향 (-)
            self.current_direction = False
        time.sleep(0.01)  # 방향 변경 후 안정화 대기
    
    def step(self, count=1):
        """
        단일 또는 여러 스텝 실행
        
        Args:
            count: 실행할 스텝 수 (기본값: 1)
        """
        for _ in range(count):
            GPIO.output(self.pul_pin, GPIO.HIGH)
            time.sleep(self.step_delay)
            GPIO.output(self.pul_pin, GPIO.LOW)
            time.sleep(self.step_delay)
    
    def rotate_steps(self, steps, direction=None, delay=None):
        """
        지정된 스텝 수만큼 회전
        
        Args:
            steps: 회전할 스텝 수
            direction: True = 정방향(+), False = 역방향(-), None = 현재 방향 유지
            delay: 스텝 간격 (초), None이면 기본값 사용
        """
        if direction is not None:
            self.set_direction(direction)
        
        step_delay = delay if delay is not None else self.step_delay
        
        dir_str = "정방향(+)" if self.current_direction else "역방향(-)"
        print(f"{dir_str} {steps} 스텝 회전 시작...")
        
        for _ in range(steps):
            GPIO.output(self.pul_pin, GPIO.HIGH)
            time.sleep(step_delay)
            GPIO.output(self.pul_pin, GPIO.LOW)
            time.sleep(step_delay)
        
        print(f"회전 완료!")
    
    def rotate_continuous(self, duration, direction=None, delay=None):
        """
        지정된 시간 동안 연속 회전
        
        Args:
            duration: 회전 시간 (초)
            direction: True = 정방향(+), False = 역방향(-), None = 현재 방향 유지
            delay: 스텝 간격 (초), None이면 기본값 사용
        """
        if direction is not None:
            self.set_direction(direction)
        
        step_delay = delay if delay is not None else self.step_delay
        
        dir_str = "정방향(+)" if self.current_direction else "역방향(-)"
        print(f"{dir_str} {duration}초 동안 연속 회전 시작...")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            GPIO.output(self.pul_pin, GPIO.HIGH)
            time.sleep(step_delay)
            GPIO.output(self.pul_pin, GPIO.LOW)
            time.sleep(step_delay)
        
        print(f"연속 회전 완료!")
    
    def set_speed(self, delay):
        """
        모터 속도 설정 (스텝 간격 변경)
        
        Args:
            delay: 스텝 간격 (초) - 작을수록 빠름
        """
        self.step_delay = delay
        print(f"스텝 간격을 {delay}초로 변경했습니다.")
    
    def stop(self):
        """모터 정지 (펄스 신호 중단)"""
        GPIO.output(self.pul_pin, GPIO.LOW)
        print("모터 정지")
    
    def cleanup(self):
        """GPIO 정리 및 리소스 해제"""
        GPIO.output(self.pul_pin, GPIO.LOW)
        GPIO.output(self.dir_pin, GPIO.LOW)
        GPIO.cleanup()
        print("GPIO 정리 완료")


def main():
    """사용 예제"""
    # ===== 핀 번호 설정 (실제 사용할 핀 번호로 변경하세요) =====
    PUL_PIN = 18   # 펄스 핀
    DIR_PIN = 23   # 방향 핀
    
    try:
        # 모터 초기화
        motor = StepperMotor(pul_pin=PUL_PIN, dir_pin=DIR_PIN, step_delay=0.001)
        
        print("\n" + "="*50)
        print("모터 제어 테스트")
        print("="*50)
        
        # 예제 1: 정방향(+) 100 스텝
        print("\n[예제 1] 정방향(+) 100 스텝")
        motor.rotate_steps(steps=100, direction=True)
        time.sleep(1)
        
        # 예제 2: 역방향(-) 100 스텝
        print("\n[예제 2] 역방향(-) 100 스텝")
        motor.rotate_steps(steps=100, direction=False)
        time.sleep(1)
        
        # 예제 3: 빠른 속도로 정방향 회전
        print("\n[예제 3] 빠른 속도로 정방향(+) 200 스텝")
        motor.rotate_steps(steps=200, direction=True, delay=0.0005)
        time.sleep(1)
        
        # 예제 4: 느린 속도로 역방향 회전
        print("\n[예제 4] 느린 속도로 역방향(-) 100 스텝")
        motor.rotate_steps(steps=100, direction=False, delay=0.005)
        time.sleep(1)
        
        # 예제 5: 연속 회전 (주석 해제하여 사용)
        # print("\n[예제 5] 정방향(+) 3초 연속 회전")
        # motor.rotate_continuous(duration=3, direction=True)
        # time.sleep(1)
        
        print("\n테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n\n프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'motor' in locals():
            motor.cleanup()


if __name__ == "__main__":
    main()
