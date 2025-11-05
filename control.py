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
MIN_DELAY = 0.0005      # 최소 딜레이 (최대 속도) - 0.5ms
MAX_DELAY = 0.002       # 최대 딜레이 (최소 속도) - 2ms
DEFAULT_DELAY = 0.00125 # 기본 딜레이 (중간 속도) - 1.25ms
DEFAULT_STEPS = 1320    # 기본 스텝 수

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
    # 딜레이 범위 제한 (안전성)
    delay = max(MIN_DELAY, min(delay, MAX_DELAY))
    
    # 방향 설정
    lgpio.gpio_write(h, dir_pin, direction)
    time.sleep(0.001)  # 방향 신호 안정화
    
    # 스텝 펄스 생성
    for _ in range(steps):
        lgpio.gpio_write(h, step_pin, 1)
        time.sleep(delay)
        lgpio.gpio_write(h, step_pin, 0)
        time.sleep(delay)

def move_all_motors_sequential(h, motors, steps, direction, delay=DEFAULT_DELAY):
    """
    모든 모터를 순차적으로 작동시킴 (M1 -> M2 -> M3)
    
    Args:
        h: lgpio 핸들
        motors: 모터 정보 딕셔너리
        steps: 스텝 수
        direction: 방향 (0=정방향, 1=역방향)
        delay: 스텝 간 딜레이
    """
    dir_str = "forward" if direction == 0 else "backward"
    print(f"\n[INFO] All motors starting sequentially: {dir_str}, steps={steps}")
    print(f"[INFO] Estimated time per motor: {steps * delay * 2:.2f}s")
    print(f"[INFO] Total estimated time: {steps * delay * 2 * len(motors):.2f}s\n")
    
    start_time = time.time()
    
    # M1, M2, M3 순서대로 순차 실행
    motor_order = ['m1', 'm2', 'm3']
    for motor_key in motor_order:
        if motor_key not in motors:
            continue
        
        motor_info = motors[motor_key]
        motor_start_time = time.time()
        
        print(f"[INFO] Starting {motor_info['name']}...")
        
        try:
            enable_motor(h, motor_info['ena_pin'], True)
            time.sleep(0.01)  # 모터 활성화 대기
            
            move_motor_simple(
                h,
                motor_info['dir_pin'],
                motor_info['step_pin'],
                steps,
                direction,
                delay
            )
        except KeyboardInterrupt:
            print(f"\n[WARN] {motor_info['name']} movement interrupted!")
            enable_motor(h, motor_info['ena_pin'], False)
            break
        except Exception as e:
            print(f"[ERROR] {motor_info['name']} error: {e}")
        finally:
            enable_motor(h, motor_info['ena_pin'], False)
        
        motor_elapsed = time.time() - motor_start_time
        print(f"[INFO] {motor_info['name']} completed in {motor_elapsed:.2f}s")
    
    elapsed_time = time.time() - start_time
    print(f"\n[INFO] All motors completed sequentially in {elapsed_time:.2f}s\n")

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
        print("  M1 f/b [steps]  -> Control Motor 1")
        print("  M2 f/b [steps]  -> Control Motor 2")
        print("  M3 f/b [steps]  -> Control Motor 3")
        print("  ALL f/b [steps] -> All motors sequentially (M1 -> M2 -> M3)")
        print("  q               -> Quit")
        print("\nParameters:")
        print("  f/b     : f=forward, b=backward")
        print(f"  steps   : Number of steps (default: {DEFAULT_STEPS})")
        print("\nSpeed Settings:")
        print(f"  Fixed speed: MEDIUM (delay={DEFAULT_DELAY*1000:.2f}ms)")
        print("\nExamples:")
        print("  M1 f       -> Motor1 forward, default steps")
        print("  M2 b 1000  -> Motor2 backward, 1000 steps")
        print("  M3 f 500   -> Motor3 forward, 500 steps")
        print("  ALL f 180  -> All motors forward 180 steps (M1 -> M2 -> M3 sequentially)")
        print("  ALL b 180  -> All motors backward 180 steps (M1 -> M2 -> M3 sequentially)")
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

            # 모든 모터 동시 작동
            if cmd[0] == 'all':
                if len(cmd) < 2:
                    print("[ERROR] Please specify direction: f (forward) or b (backward)")
                    continue
                
                if cmd[1] not in ['f', 'b']:
                    print("[ERROR] Direction must be 'f' (forward) or 'b' (backward)")
                    continue
                
                direction = 0 if cmd[1] == 'f' else 1
                
                try:
                    steps = int(cmd[2]) if len(cmd) >= 3 else DEFAULT_STEPS
                except ValueError:
                    print("[ERROR] Invalid steps value")
                    continue
                
                if steps <= 0:
                    print("[ERROR] Steps must be positive")
                    continue
                
                delay = DEFAULT_DELAY
                
                try:
                    move_all_motors_sequential(h, motors, steps, direction, delay)
                except KeyboardInterrupt:
                    print("\n[WARN] All motors movement interrupted!")
                
                continue

            # 모터 선택 확인
            if cmd[0] not in motors:
                print(f"[ERROR] Unknown command '{cmd[0]}'. Use M1, M2, M3, ALL, or q")
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
            except ValueError:
                print("[ERROR] Invalid steps value")
                continue

            if steps <= 0:
                print("[ERROR] Steps must be positive")
                continue

            # 고정 딜레이 사용 (중간 속도)
            delay = DEFAULT_DELAY

            # 실행 시간 예측
            estimated_time = steps * delay * 2  # delay는 HIGH와 LOW 각각 적용
            
            # 정보 출력
            print(f"\n[{motor_info['name']}] {dir_str}, steps={steps}, delay={delay*1000:.2f}ms (MEDIUM)")
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

