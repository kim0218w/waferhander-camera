import time
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR and SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import lgpio
except Exception:
    lgpio = None

# -------------------- 3개의 NEMA17 모터 핀 설정 --------------------
# Motor 1 (M1)
DIR_PIN_M1  = 24
STEP_PIN_M1 = 23
ENA_PIN_M1  = 25

# Motor 2 (M2)
DIR_PIN_M2  = 20
STEP_PIN_M2 = 21
ENA_PIN_M2  = 16

# Motor 3 (M3) - 새로운 핀 할당 (필요시 수정)
DIR_PIN_M3  = 17
STEP_PIN_M3 = 27
ENA_PIN_M3  = 22

# -------------------- NEMA17 공통 파라미터 --------------------
DEFAULT_DELAY = 0.001  # 기본 딜레이 (초) - 스텝 간격
DEFAULT_STEPS = 1320   # 기본 스텝 수

# -------------------- Stepper helpers --------------------
def enable_motor(h, ena_pin, enable=True):
    """모터 활성화/비활성화"""
    state = 0 if enable else 1
    lgpio.gpio_write(h, ena_pin, state)

def move_motor_simple(h, dir_pin, step_pin, steps, direction, delay):
    """
    모터를 단순하게 이동 (가감속 없음)
    
    Args:
        h: lgpio 핸들
        dir_pin: 방향 핀
        step_pin: 스텝 핀
        steps: 이동할 스텝 수
        direction: 방향 (0=정방향, 1=역방향)
        delay: 스텝 간 딜레이 (초)
    """
    # 방향 설정
    lgpio.gpio_write(h, dir_pin, direction)
    time.sleep(0.001)  # 방향 신호 안정화
    
    # 스텝 펄스 생성
    for _ in range(steps):
        lgpio.gpio_write(h, step_pin, 1)
        time.sleep(delay)
        lgpio.gpio_write(h, step_pin, 0)
        time.sleep(delay)

# -------------------- Main --------------------
def main():
    if lgpio is None:
        print("[ERROR] lgpio is not available in this environment.")
        print("Please run this on a Raspberry Pi with lgpio installed.")
        return

    # 모터 설정 딕셔너리
    motors = {
        'm1': {
            'name': 'Motor 1 (NEMA17)',
            'dir_pin': DIR_PIN_M1,
            'step_pin': STEP_PIN_M1,
            'ena_pin': ENA_PIN_M1,
        },
        'm2': {
            'name': 'Motor 2 (NEMA17)',
            'dir_pin': DIR_PIN_M2,
            'step_pin': STEP_PIN_M2,
            'ena_pin': ENA_PIN_M2,
        },
        'm3': {
            'name': 'Motor 3 (NEMA17)',
            'dir_pin': DIR_PIN_M3,
            'step_pin': STEP_PIN_M3,
            'ena_pin': ENA_PIN_M3,
        }
    }

    # GPIO 초기화
    h = lgpio.gpiochip_open(0)
    
    # 모든 모터 핀 초기화
    for motor_info in motors.values():
        lgpio.gpio_claim_output(h, motor_info['dir_pin'])
        lgpio.gpio_claim_output(h, motor_info['step_pin'])
        lgpio.gpio_claim_output(h, motor_info['ena_pin'])
        # 초기에는 모터 비활성화
        enable_motor(h, motor_info['ena_pin'], False)

    try:
        print("\n" + "="*70)
        print(" NEMA17 3-Motor Simple Control")
        print("="*70)
        print("\nCommands:")
        print("  M1 f/b [steps] [delay]  -> Control Motor 1")
        print("  M2 f/b [steps] [delay]  -> Control Motor 2")
        print("  M3 f/b [steps] [delay]  -> Control Motor 3")
        print("  q                       -> Quit")
        print("\nParameters:")
        print("  f/b     : f=forward, b=backward")
        print("  steps   : Number of steps (default: 1320)")
        print("  delay   : Delay between steps in seconds (default: 0.001)")
        print("\nExamples:")
        print("  M1 f              -> Motor1 forward, default settings")
        print("  M2 b 1000         -> Motor2 backward, 1000 steps")
        print("  M3 f 500 0.002    -> Motor3 forward, 500 steps, 2ms delay")
        print("="*70 + "\n")

        while True:
            try:
                cmd = input(">> ").strip().lower().split()
            except EOFError:
                break
                
            if not cmd:
                continue
                
            if cmd[0] == 'q':
                break

            # 모터 선택 확인
            if cmd[0] not in motors:
                print(f"[ERROR] Unknown command '{cmd[0]}'. Use M1, M2, M3, or q")
                continue

            motor_info = motors[cmd[0]]
            
            if len(cmd) < 2:
                print("[ERROR] Please specify direction: f (forward) or b (backward)")
                continue

            # 방향 파싱
            if cmd[1] not in ['f', 'b']:
                print("[ERROR] Direction must be 'f' (forward) or 'b' (backward)")
                continue
            
            direction = 0 if cmd[1] == 'f' else 1
            dir_str = "forward" if direction == 0 else "backward"

            # 파라미터 파싱
            try:
                steps = int(cmd[2]) if len(cmd) >= 3 else DEFAULT_STEPS
                delay = float(cmd[3]) if len(cmd) >= 4 else DEFAULT_DELAY
            except ValueError:
                print("[ERROR] Invalid steps or delay value")
                continue

            if steps <= 0:
                print("[ERROR] Steps must be positive")
                continue
                
            if delay <= 0:
                print("[ERROR] Delay must be positive")
                continue

            # 실행 시간 예측
            estimated_time = steps * delay * 2  # delay는 HIGH와 LOW 각각 적용
            
            # 정보 출력
            print(f"\n[{motor_info['name']}] {dir_str}, steps={steps}, delay={delay*1000:.3f}ms")
            print(f"[INFO] Estimated time: {estimated_time:.2f}s")

            # 모터 실행
            print("[INFO] Running motor...")
            start_time = time.time()
            
            enable_motor(h, motor_info['ena_pin'], True)
            time.sleep(0.01)  # 모터 활성화 대기
            
            try:
                move_motor_simple(
                    h, 
                    motor_info['dir_pin'], 
                    motor_info['step_pin'], 
                    steps, 
                    direction, 
                    delay
                )
            except KeyboardInterrupt:
                print("\n[WARN] Motor movement interrupted!")
            finally:
                enable_motor(h, motor_info['ena_pin'], False)
            
            elapsed_time = time.time() - start_time
            print(f"[INFO] Completed in {elapsed_time:.2f}s\n")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        # 모든 모터 비활성화 및 GPIO 정리
        try:
            for motor_info in motors.values():
                enable_motor(h, motor_info['ena_pin'], False)
            lgpio.gpiochip_close(h)
            print("[INFO] GPIO released.")
        except Exception as e:
            print(f"[ERROR] Failed to release GPIO: {e}")

if __name__ == "__main__":
    main()

