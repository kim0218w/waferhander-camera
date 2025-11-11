#!/usr/bin/env python3
"""
IMX219 카메라 진단 스크립트
카메라 연결 및 설정 상태를 확인합니다.
"""

import sys
import subprocess

print("="*70)
print(" IMX219 Camera Diagnostic Tool")
print("="*70)
print()

# 1. picamera2 임포트 확인
print("[1/6] picamera2 라이브러리 확인...")
try:
    from picamera2 import Picamera2
    print("  ✓ picamera2 설치됨")
except ImportError as e:
    print(f"  ✗ picamera2 없음: {e}")
    print("  설치: sudo apt install -y python3-picamera2")
    sys.exit(1)

# 2. global_camera_info() 확인
print("\n[2/6] 카메라 감지 확인...")
try:
    cameras = Picamera2.global_camera_info()
    print(f"  감지된 카메라 수: {len(cameras)}")
    if cameras:
        for i, cam in enumerate(cameras):
            print(f"  카메라 {i}: {cam}")
    else:
        print("  ✗ 카메라가 감지되지 않음!")
except Exception as e:
    print(f"  ✗ 오류 발생: {e}")
    cameras = []

# 3. libcamera 또는 raspistill 명령어로 확인
print("\n[3/6] 카메라 명령어로 확인...")

# libcamera-hello 시도 (새 버전)
libcamera_found = False
try:
    result = subprocess.run(
        ['libcamera-hello', '--list-cameras'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if result.returncode == 0:
        print("  ✓ libcamera-hello 사용 가능:")
        print("  " + "\n  ".join(result.stdout.split('\n')[:10]))
        libcamera_found = True
    else:
        print(f"  ✗ libcamera 오류: {result.stderr}")
except FileNotFoundError:
    print("  ⚠ libcamera-hello 없음 (최신 OS 필요)")
except Exception as e:
    print(f"  ✗ 오류: {e}")

# raspistill 시도 (레거시)
if not libcamera_found:
    try:
        result = subprocess.run(
            ['raspistill', '--help'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print("  ✓ raspistill 사용 가능 (레거시 모드)")
        else:
            print("  ⚠ raspistill 오류")
    except FileNotFoundError:
        print("  ✗ raspistill도 없음")
        print("  ⚠ 카메라 명령어가 모두 없습니다.")
        print("  -> picamera2만 사용 가능")
    except Exception as e:
        print(f"  ✗ 오류: {e}")

# 4. vcgencmd로 카메라 상태 확인
print("\n[4/6] Raspberry Pi 카메라 상태 확인...")
try:
    result = subprocess.run(
        ['vcgencmd', 'get_camera'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if result.returncode == 0:
        output = result.stdout.strip()
        print(f"  {output}")
        if "detected=1" in output:
            print("  ✓ 카메라 감지됨")
        else:
            print("  ✗ 카메라가 감지되지 않음!")
    else:
        print(f"  ✗ 오류: {result.stderr}")
except FileNotFoundError:
    print("  ⚠ vcgencmd 명령어가 없음")
except Exception as e:
    print(f"  ✗ 오류: {e}")

# 5. /dev/video* 확인
print("\n[5/6] 비디오 장치 확인...")
try:
    result = subprocess.run(
        ['ls', '-la', '/dev/video*'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if result.returncode == 0:
        print("  ✓ 비디오 장치 발견:")
        for line in result.stdout.strip().split('\n'):
            print(f"    {line}")
    else:
        print("  ✗ 비디오 장치가 없음")
except Exception as e:
    print(f"  ✗ 오류: {e}")

# 6. Picamera2 객체 생성 시도
print("\n[6/6] Picamera2 객체 생성 시도...")
if cameras and len(cameras) > 0:
    try:
        picam = Picamera2(0)
        print("  ✓ Picamera2(0) 객체 생성 성공")
        
        # 센서 모드 확인
        sensor_modes = picam.sensor_modes
        print(f"  센서 모드 수: {len(sensor_modes)}")
        
        # 카메라 속성
        props = picam.camera_properties
        print(f"  카메라 모델: {props.get('Model', 'Unknown')}")
        
        # 설정 및 시작 시도
        config = picam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        picam.configure(config)
        print("  ✓ 카메라 설정 성공")
        
        picam.start()
        print("  ✓ 카메라 시작 성공")
        
        import time
        time.sleep(1)
        
        frame = picam.capture_array()
        if frame is not None and frame.size > 0:
            print(f"  ✓ 프레임 캡처 성공: {frame.shape}")
        else:
            print("  ✗ 프레임 캡처 실패")
        
        picam.stop()
        print("  ✓ 테스트 완료")
        
    except Exception as e:
        print(f"  ✗ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
else:
    print("  ⊗ 감지된 카메라가 없어 테스트 생략")

# 결과 요약
print("\n" + "="*70)
print(" 진단 결과 요약")
print("="*70)

if cameras and len(cameras) > 0:
    print("✓ 카메라가 감지되었습니다!")
    print("  -> camera_position.py를 실행할 수 있어야 합니다.")
else:
    print("✗ 카메라가 감지되지 않았습니다!")
    print("\n해결 방법:")
    print("1. 카메라 케이블 연결 확인:")
    print("   - Raspberry Pi의 Camera 포트에 제대로 삽입되었는지 확인")
    print("   - 케이블의 파란색 면이 이더넷 포트 쪽을 향해야 함")
    print("   - 케이블이 손상되지 않았는지 확인")
    print()
    print("2. 카메라 인터페이스 활성화:")
    print("   sudo raspi-config")
    print("   -> Interface Options -> Legacy Camera -> No")
    print("   -> Interface Options -> Camera -> Yes")
    print("   -> Finish -> Reboot")
    print()
    print("3. 재부팅:")
    print("   sudo reboot")
    print()
    print("4. 재부팅 후 이 스크립트 다시 실행:")
    print("   python camera_check.py")

print("="*70)

