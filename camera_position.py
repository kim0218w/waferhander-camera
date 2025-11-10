import cv2
import numpy as np
import time
import tkinter as tk
from tkinter import simpledialog
from datetime import datetime

# Raspberry Pi 카메라 지원
try:
    from picamera2 import Picamera2
    USE_PICAMERA = True
except ImportError:
    USE_PICAMERA = False
    print("[INFO] picamera2 not found, using standard camera")

# ===== 설정 =====
SOURCE = 0  # 0=웹캠
FONT = cv2.FONT_HERSHEY_SIMPLEX

# ===== 전역 변수 =====
FIXED_Z_DISTANCE = 0.032  # 3.2cm = 0.032m (기본값)
focal_length = 800.0  # 초점거리 (기본값, 자동 조정됨)

selected_points = []  # 선택한 3점의 초기 픽셀 좌표
point_names = ['Point 1', 'Point 2', 'Point 3']
tracked_points = []  # 추적 중인 점들 (실시간 업데이트)

tracking_active = False

# 측정 관련 변수
measurement_active = False
last_measurement_time = 0
measurement_log = []  # 측정 기록 저장

def get_z_distance_input():
    """Z축 거리 입력 받기"""
    root = tk.Tk()
    root.withdraw()
    
    distance_cm = simpledialog.askfloat(
        "Z축 거리 설정",
        "카메라와 물체 사이의 Z축 거리를 입력하세요 (cm 단위):\n\n"
        "예: 3.2 (3.2cm)\n"
        "    5.0 (5cm)\n"
        "    10.0 (10cm)",
        initialvalue=3.2,
        minvalue=0.1,
        maxvalue=1000.0
    )
    
    root.destroy()
    
    if distance_cm is not None:
        return distance_cm / 100.0  # cm를 m로 변환
    return None

def calculate_3d_position_fixed_z(point_px, focal_length, fixed_z, image_width, image_height):
    """
    고정된 Z 거리로 3D 위치 계산
    
    Args:
        point_px: (x, y) 픽셀 좌표
        focal_length: 초점거리
        fixed_z: 고정된 Z축 거리 (미터)
        image_width, image_height: 이미지 크기
    
    Returns:
        (X, Y, Z, distance_3d)
    """
    cx = image_width / 2.0
    cy = image_height / 2.0
    
    # 픽셀 좌표를 정규화
    x_norm = (point_px[0] - cx) / focal_length
    y_norm = (point_px[1] - cy) / focal_length
    
    # 3D 위치 (Z는 고정)
    Z = fixed_z
    X = x_norm * Z
    Y = y_norm * Z
    
    # 카메라로부터의 유클리드 거리
    distance_3d = np.sqrt(X**2 + Y**2 + Z**2)
    
    return X, Y, Z, distance_3d

def mouse_callback(event, x, y, flags, param):
    """마우스 클릭으로 3점 선택"""
    global selected_points, tracked_points, tracking_active
    
    if event == cv2.EVENT_LBUTTONDOWN and not tracking_active:
        if len(selected_points) < 3:
            selected_points.append((x, y))
            tracked_points.append((x, y))
            print(f"[INFO] {point_names[len(selected_points)-1]} 선택: ({x}, {y})")
            
            if len(selected_points) == 3:
                tracking_active = True
                print("[INFO] 3점 선택 완료! 실시간 추적 시작")

def track_points_optical_flow(prev_gray, curr_gray, points):
    """
    Optical Flow로 점 추적
    
    Args:
        prev_gray: 이전 프레임 (grayscale)
        curr_gray: 현재 프레임 (grayscale)
        points: 추적할 점들
    
    Returns:
        추적된 점들 또는 None
    """
    if points is None or len(points) == 0:
        return None
    
    # Lucas-Kanade Optical Flow 파라미터
    lk_params = dict(
        winSize=(15, 15),
        maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    )
    
    # numpy array로 변환
    points_array = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    
    # Optical Flow 계산
    next_points, status, error = cv2.calcOpticalFlowPyrLK(
        prev_gray, curr_gray, points_array, None, **lk_params
    )
    
    if next_points is None:
        return None
    
    # 성공적으로 추적된 점들만 반환
    good_points = []
    for i, (st, pt) in enumerate(zip(status, next_points)):
        if st == 1:  # 추적 성공
            good_points.append(tuple(pt.ravel()))
        else:
            # 추적 실패 시 이전 위치 유지
            good_points.append(points[i])
    
    return good_points

def save_measurement_log():
    """측정 기록을 CSV 파일로 저장"""
    if not measurement_log:
        print("[INFO] 저장할 측정 데이터가 없습니다.")
        return
    
    filename = f"distance_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with open(filename, 'w', encoding='utf-8') as f:
        # 헤더
        f.write("Timestamp,Point1_X(cm),Point1_Y(cm),Point1_Z(cm),Point1_Distance(cm),")
        f.write("Point2_X(cm),Point2_Y(cm),Point2_Z(cm),Point2_Distance(cm),")
        f.write("Point3_X(cm),Point3_Y(cm),Point3_Z(cm),Point3_Distance(cm)\n")
        
        # 데이터
        for entry in measurement_log:
            f.write(f"{entry['timestamp']},")
            for i in range(3):
                pt = entry['points'][i]
                f.write(f"{pt['X']:.2f},{pt['Y']:.2f},{pt['Z']:.2f},{pt['distance']:.2f}")
                if i < 2:
                    f.write(",")
            f.write("\n")
    
    print(f"[INFO] 측정 데이터 저장: {filename} ({len(measurement_log)}개 기록)")

def main():
    global FIXED_Z_DISTANCE, focal_length
    global selected_points, tracked_points, tracking_active
    global measurement_active, last_measurement_time, measurement_log
    
    # Z축 거리 설정
    print("[INFO] Z축 거리 설정...")
    z_input = get_z_distance_input()
    
    if z_input is not None:
        FIXED_Z_DISTANCE = z_input
        print(f"[INFO] Z축 거리 설정: {FIXED_Z_DISTANCE*100:.1f}cm ({FIXED_Z_DISTANCE:.4f}m)")
    else:
        print(f"[INFO] 기본값 사용: {FIXED_Z_DISTANCE*100:.1f}cm")
    
    print("[INFO] 카메라 초기화 중...")
    
    # 카메라 초기화
    picam = None
    cap = None
    
    if USE_PICAMERA:
        try:
            print("[INFO] Raspberry Pi 카메라 모듈 사용")
            picam = Picamera2()
            config = picam.create_preview_configuration(
                main={"size": (1280, 720), "format": "RGB888"}
            )
            picam.configure(config)
            picam.start()
            time.sleep(2)
            
            test_frame = picam.capture_array()
            if test_frame is None:
                print("[ERROR] 카메라에서 프레임을 읽을 수 없습니다!")
                picam.stop()
                return
            
            print(f"[INFO] 카메라 초기화 성공! 해상도: {test_frame.shape[1]}x{test_frame.shape[0]}")
        except Exception as e:
            print(f"[ERROR] Raspberry Pi 카메라 초기화 실패: {e}")
            return
    else:
        print("[INFO] USB 카메라 사용")
        cap = cv2.VideoCapture(SOURCE)
        if not cap.isOpened():
            print("[ERROR] 카메라를 열 수 없습니다!")
            return
        
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        for _ in range(5):
            cap.read()
            time.sleep(0.1)
        
        ret, test_frame = cap.read()
        if not ret or test_frame is None:
            print("[ERROR] 카메라에서 프레임을 읽을 수 없습니다!")
            cap.release()
            return
        
        print(f"[INFO] 카메라 초기화 성공! 해상도: {test_frame.shape[1]}x{test_frame.shape[0]}")
    
    # 창 생성
    window_name = "Fixed Z-Axis Distance Tracker"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    print("\n" + "="*70)
    print(" Fixed Z-Axis Distance Tracker")
    print("="*70)
    print(f"\n설정: Z축 고정 거리 = {FIXED_Z_DISTANCE*100:.1f}cm")
    print("\n사용법:")
    print("  1. 마우스로 3점을 클릭하여 선택")
    print("  2. 자동으로 실시간 추적 시작")
    print("  3. 카메라를 좌우로 움직이면 각 점과의 거리가 실시간으로 표시됩니다")
    print("  4. 'm' 키를 누르면 1초 단위로 거리 측정 시작 (다시 'm' 키로 중지)")
    print("\n단축키:")
    print("  'm' - 1초 단위 측정 시작/중지")
    print("  'r' - 점 선택 초기화 (다시 선택)")
    print("  'z' - Z축 거리 재설정")
    print("  's' - 측정 데이터를 CSV 파일로 저장")
    print("  'q' - 종료")
    print("="*70 + "\n")
    
    # EMA 필터 (부드러운 출력)
    alpha = 0.3
    ema_distances = [None, None, None]
    
    # Optical Flow용 이전 프레임
    prev_gray = None
    
    h_img, w_img = test_frame.shape[:2]
    
    while True:
        # 프레임 읽기
        if picam is not None:
            frame = picam.capture_array()
            if frame is None:
                time.sleep(0.1)
                continue
        else:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.1)
                continue
        
        h_img, w_img = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 점 추적 (Optical Flow)
        if tracking_active and prev_gray is not None:
            new_points = track_points_optical_flow(prev_gray, gray, tracked_points)
            if new_points is not None:
                tracked_points = new_points
        
        prev_gray = gray.copy()
        
        # 현재 시간
        current_time = time.time()
        
        # 상태 표시
        if not tracking_active:
            remaining = 3 - len(selected_points)
            cv2.putText(frame, f"Click to select points ({remaining} remaining)", 
                       (20, 30), FONT, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, f"Z-axis distance: {FIXED_Z_DISTANCE*100:.1f}cm (fixed)", 
                       (20, 60), FONT, 0.6, (255, 255, 255), 2)
        else:
            cv2.putText(frame, "Tracking active - Move camera left/right", 
                       (20, 30), FONT, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Z-axis: {FIXED_Z_DISTANCE*100:.1f}cm (fixed)", 
                       (20, 60), FONT, 0.6, (255, 255, 255), 2)
            
            # 측정 상태 표시
            if measurement_active:
                cv2.putText(frame, f"[MEASURING] Recording every 1 sec ({len(measurement_log)} records)", 
                           (20, 90), FONT, 0.7, (0, 0, 255), 2)
                # 깜박이는 효과
                if int(current_time * 2) % 2 == 0:
                    cv2.circle(frame, (w_img - 30, 30), 15, (0, 0, 255), -1)
            else:
                cv2.putText(frame, "[Press 'M' to start measuring]", 
                           (20, 90), FONT, 0.6, (200, 200, 200), 1)
        
        # 선택된/추적 중인 점들 표시 및 거리 계산
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        
        # 현재 프레임의 측정 데이터 저장용
        current_measurements = []
        
        for i in range(len(tracked_points)):
            point = tracked_points[i]
            color = colors[i]
            
            # 점 표시
            cv2.circle(frame, (int(point[0]), int(point[1])), 10, color, -1)
            cv2.circle(frame, (int(point[0]), int(point[1])), 15, color, 2)
            cv2.putText(frame, point_names[i], 
                       (int(point[0]) + 20, int(point[1]) - 15), 
                       FONT, 0.6, color, 2)
            
            # 거리 계산
            result = calculate_3d_position_fixed_z(
                point, focal_length, FIXED_Z_DISTANCE, w_img, h_img
            )
            
            if result is not None:
                X, Y, Z, distance_3d = result
                
                # EMA 필터 적용
                if ema_distances[i] is None:
                    ema_distances[i] = distance_3d
                else:
                    ema_distances[i] = alpha * distance_3d + (1 - alpha) * ema_distances[i]
                
                # 측정 데이터 저장
                current_measurements.append({
                    'X': X * 100,  # m → cm
                    'Y': Y * 100,
                    'Z': Z * 100,
                    'distance': ema_distances[i] * 100
                })
                
                # 화면에 표시
                y_offset = 120 + i * 130
                cv2.putText(frame, f"=== {point_names[i]} ===", 
                           (20, y_offset), FONT, 0.7, color, 2)
                cv2.putText(frame, f"X: {X*100:+.2f}cm  Y: {Y*100:+.2f}cm  Z: {Z*100:.2f}cm", 
                           (20, y_offset + 30), FONT, 0.6, color, 1)
                cv2.putText(frame, f"Distance: {ema_distances[i]*100:.2f}cm ({ema_distances[i]*1000:.1f}mm)", 
                           (20, y_offset + 60), FONT, 0.7, color, 2)
                
                # 초기 위치로부터의 변화량
                if i < len(selected_points):
                    initial_point = selected_points[i]
                    dx = (point[0] - initial_point[0])
                    dy = (point[1] - initial_point[1])
                    cv2.putText(frame, f"Pixel shift: X{dx:+.0f}px Y{dy:+.0f}px", 
                               (20, y_offset + 90), FONT, 0.5, color, 1)
        
        # 1초마다 측정 기록
        if measurement_active and tracking_active and len(current_measurements) == 3:
            if current_time - last_measurement_time >= 1.0:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                measurement_log.append({
                    'timestamp': timestamp,
                    'points': current_measurements
                })
                last_measurement_time = current_time
                print(f"[MEASUREMENT] {timestamp} - "
                      f"P1: {current_measurements[0]['distance']:.2f}cm, "
                      f"P2: {current_measurements[1]['distance']:.2f}cm, "
                      f"P3: {current_measurements[2]['distance']:.2f}cm")
        
        # 화면 표시
        cv2.imshow(window_name, frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            # 종료 시 측정 중이었다면 자동 저장
            if measurement_log:
                print(f"\n[INFO] 종료 전 측정 데이터 자동 저장 중...")
                save_measurement_log()
            break
        elif key == ord('m') or key == ord('M'):
            # 측정 시작/중지
            if tracking_active:
                measurement_active = not measurement_active
                if measurement_active:
                    last_measurement_time = current_time
                    print("[INFO] 1초 단위 측정 시작")
                else:
                    print(f"[INFO] 측정 중지 (총 {len(measurement_log)}개 기록)")
            else:
                print("[WARNING] 먼저 3점을 선택해주세요!")
        elif key == ord('s') or key == ord('S'):
            # 측정 데이터 저장
            save_measurement_log()
        elif key == ord('r'):
            # 초기화
            selected_points = []
            tracked_points = []
            tracking_active = False
            measurement_active = False
            ema_distances = [None, None, None]
            prev_gray = None
            print("[INFO] 점 선택 초기화")
        elif key == ord('z'):
            # Z축 거리 재설정
            z_input = get_z_distance_input()
            if z_input is not None:
                FIXED_Z_DISTANCE = z_input
                print(f"[INFO] Z축 거리 변경: {FIXED_Z_DISTANCE*100:.1f}cm")
                # 거리 재계산을 위해 EMA 초기화
                ema_distances = [None, None, None]
    
    # 리소스 정리
    if picam is not None:
        picam.stop()
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] 프로그램이 사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n[ERROR] 예상치 못한 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        print("[INFO] 프로그램 종료")
