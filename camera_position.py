import cv2
import numpy as np
import time
import tkinter as tk
from tkinter import simpledialog
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

# Raspberry Pi 카메라 지원 (picamera2)
try:
    from picamera2 import Picamera2
    USE_PICAMERA2 = True
except ImportError:
    USE_PICAMERA2 = False
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

# 그래프 관련 변수
graph_enabled = False
graph_data_time = deque(maxlen=100)  # 최근 100개 데이터만 유지
graph_data_p1 = deque(maxlen=100)
graph_data_p2 = deque(maxlen=100)
graph_data_p3 = deque(maxlen=100)
graph_start_time = None
graph_lock = threading.Lock()

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

def calculate_alignment_metrics(points_3d):
    """
    3개 점의 정렬 상태를 측정
    
    Args:
        points_3d: [(X1, Y1, Z1), (X2, Y2, Z2), (X3, Y3, Z3)]
    
    Returns:
        dict: {
            'z_std': Z축 표준편차,
            'z_range': Z축 최대-최소 차이,
            'collinearity': 공선성 정도 (0에 가까울수록 일직선),
            'is_aligned': 정렬 여부 (bool)
        }
    """
    if len(points_3d) != 3:
        return None
    
    # Z축 분석
    z_values = [p[2] for p in points_3d]
    z_std = np.std(z_values)
    z_range = max(z_values) - min(z_values)
    
    # 공선성 측정 (3점이 일직선상에 있는지)
    # 벡터 AB와 AC를 구하고 외적(cross product)의 크기를 계산
    p1, p2, p3 = points_3d
    vec_AB = np.array([p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]])
    vec_AC = np.array([p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]])
    
    # 외적의 크기 (0에 가까울수록 일직선)
    cross_product = np.cross(vec_AB, vec_AC)
    collinearity = np.linalg.norm(cross_product)
    
    # 정규화 (거리 기준)
    distance_AB = np.linalg.norm(vec_AB)
    distance_AC = np.linalg.norm(vec_AC)
    if distance_AB > 0 and distance_AC > 0:
        collinearity_normalized = collinearity / (distance_AB * distance_AC)
    else:
        collinearity_normalized = 0
    
    # 정렬 판정 기준
    # Z축 범위가 1mm 이내이고, 공선성이 낮으면 정렬된 것으로 판정
    is_aligned = (z_range < 0.001) and (collinearity_normalized < 0.05)
    
    return {
        'z_std': z_std,
        'z_range': z_range,
        'collinearity': collinearity,
        'collinearity_normalized': collinearity_normalized,
        'is_aligned': is_aligned
    }

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

def update_graph_data(distances_cm):
    """
    그래프 데이터 업데이트
    
    Args:
        distances_cm: [dist1, dist2, dist3] (cm 단위)
    """
    global graph_data_time, graph_data_p1, graph_data_p2, graph_data_p3
    global graph_start_time, graph_lock
    
    if graph_start_time is None:
        graph_start_time = time.time()
    
    elapsed_time = time.time() - graph_start_time
    
    with graph_lock:
        graph_data_time.append(elapsed_time)
        graph_data_p1.append(distances_cm[0])
        graph_data_p2.append(distances_cm[1])
        graph_data_p3.append(distances_cm[2])

def create_realtime_graph():
    """실시간 그래프 창 생성 및 업데이트"""
    global graph_enabled
    
    # matplotlib 설정
    try:
        plt.style.use('seaborn-v0_8-darkgrid')
    except:
        try:
            plt.style.use('seaborn-darkgrid')
        except:
            pass  # 기본 스타일 사용
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    line1, = ax.plot([], [], 'b-', linewidth=2, label='Point 1', marker='o', markersize=3)
    line2, = ax.plot([], [], 'g-', linewidth=2, label='Point 2', marker='s', markersize=3)
    line3, = ax.plot([], [], 'r-', linewidth=2, label='Point 3', marker='^', markersize=3)
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Distance (cm)', fontsize=12)
    ax.set_title('Real-time Distance Tracking (3 Points)', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    def init():
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        return line1, line2, line3
    
    def update(frame):
        global graph_data_time, graph_data_p1, graph_data_p2, graph_data_p3
        
        with graph_lock:
            if len(graph_data_time) > 0:
                times = list(graph_data_time)
                p1_data = list(graph_data_p1)
                p2_data = list(graph_data_p2)
                p3_data = list(graph_data_p3)
                
                line1.set_data(times, p1_data)
                line2.set_data(times, p2_data)
                line3.set_data(times, p3_data)
                
                # 축 범위 자동 조정
                if len(times) > 0:
                    ax.set_xlim(max(0, times[-1] - 30), times[-1] + 2)  # 최근 30초
                    
                    all_distances = p1_data + p2_data + p3_data
                    if all_distances:
                        min_dist = min(all_distances)
                        max_dist = max(all_distances)
                        margin = (max_dist - min_dist) * 0.1 or 1
                        ax.set_ylim(min_dist - margin, max_dist + margin)
        
        return line1, line2, line3
    
    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=100, cache_frame_data=False)
    
    plt.tight_layout()
    plt.show()
    
    graph_enabled = False
    print("[INFO] 그래프 창이 닫혔습니다.")

def start_graph_thread():
    """그래프를 별도 스레드에서 실행"""
    global graph_enabled, graph_start_time
    global graph_data_time, graph_data_p1, graph_data_p2, graph_data_p3
    
    if graph_enabled:
        print("[WARNING] 그래프가 이미 실행 중입니다.")
        return
    
    # 그래프 데이터 초기화
    graph_start_time = time.time()
    with graph_lock:
        graph_data_time.clear()
        graph_data_p1.clear()
        graph_data_p2.clear()
        graph_data_p3.clear()
    
    graph_enabled = True
    graph_thread = threading.Thread(target=create_realtime_graph, daemon=True)
    graph_thread.start()
    print("[INFO] 실시간 그래프 시작 (별도 창)")

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
        f.write("Point3_X(cm),Point3_Y(cm),Point3_Z(cm),Point3_Distance(cm),")
        f.write("Z_Range(mm),Collinearity,Aligned\n")
        
        # 데이터
        for entry in measurement_log:
            f.write(f"{entry['timestamp']},")
            for i in range(3):
                pt = entry['points'][i]
                f.write(f"{pt['X']:.2f},{pt['Y']:.2f},{pt['Z']:.2f},{pt['distance']:.2f},")
            
            # 정렬 정보 추가
            if 'alignment' in entry and entry['alignment'] is not None:
                align = entry['alignment']
                f.write(f"{align['z_range']*1000:.2f},")  # mm로 변환
                f.write(f"{align['collinearity_normalized']:.4f},")
                f.write(f"{'YES' if align['is_aligned'] else 'NO'}")
            else:
                f.write("N/A,N/A,N/A")
            
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
    
    # Raspberry Pi 카메라 시도 (picamera2)
    if USE_PICAMERA2:
        try:
            print("[INFO] Raspberry Pi 카메라(picamera2) 초기화 중...")
            
            picam = Picamera2()
            
            # 비디오 설정 생성 (BGR888 포맷으로 OpenCV와 호환)
            config = picam.create_video_configuration(
                main={"size": (1280, 720), "format": "BGR888"},
                controls={"FrameRate": 30}
            )
            picam.configure(config)
            
            # 카메라 시작
            picam.start()
            
            print("[INFO] 카메라 워밍업 중... (2초)")
            time.sleep(2)  # 카메라 워밍업
            
            # 테스트 프레임 캡처
            test_frame = picam.capture_array()
            
            if test_frame is not None and test_frame.size > 0:
                h, w = test_frame.shape[:2]
                print(f"[SUCCESS] Raspberry Pi 카메라 초기화 완료!")
                print(f"  해상도: {w}x{h}")
                print(f"  포맷: BGR888")
                print(f"  라이브러리: picamera2")
            else:
                print("[ERROR] 테스트 프레임 캡처 실패")
                picam.stop()
                picam.close()
                picam = None
                
        except Exception as e:
            print(f"[ERROR] Raspberry Pi 카메라 초기화 실패: {e}")
            print(f"  오류 타입: {type(e).__name__}")
            print("\n해결 방법:")
            print("  1. 카메라 케이블 연결 확인")
            print("  2. libcamera가 설치되어 있는지 확인:")
            print("     sudo apt install -y python3-picamera2")
            print("  3. 카메라가 감지되는지 확인:")
            print("     libcamera-hello --list-cameras")
            print("  4. 재부팅: sudo reboot")
            
            if picam is not None:
                try:
                    picam.stop()
                    picam.close()
                except:
                    pass
            picam = None
            print("[INFO] USB 카메라로 자동 전환합니다...\n")
    
    # USB 카메라 사용 (picamera가 없거나 실패한 경우)
    if picam is None:
        print("[INFO] USB 카메라 사용")
        cap = cv2.VideoCapture(SOURCE)
        if not cap.isOpened():
            print("[ERROR] USB 카메라를 열 수 없습니다!")
            print("[INFO] 다른 카메라 인덱스 시도 중...")
            # 다른 카메라 인덱스 시도
            for i in range(1, 4):
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    print(f"[INFO] 카메라 인덱스 {i}에서 카메라 발견!")
                    break
            
            if not cap.isOpened():
                print("[ERROR] 사용 가능한 카메라를 찾을 수 없습니다!")
                print("\n해결 방법:")
                print("1. Raspberry Pi 카메라를 사용하려면:")
                print("   sudo raspi-config -> Interface Options -> Camera -> Enable")
                print("2. USB 카메라가 제대로 연결되었는지 확인하세요:")
                print("   ls /dev/video*")
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
        
        print(f"[INFO] USB 카메라 초기화 성공! 해상도: {test_frame.shape[1]}x{test_frame.shape[0]}")
    
    # 창 생성
    window_name = "Fixed Z-Axis Distance Tracker"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    print("\n" + "="*70)
    print(" Fixed Z-Axis Distance Tracker with Alignment Detection")
    print("="*70)
    print(f"\n설정: Z축 고정 거리 = {FIXED_Z_DISTANCE*100:.1f}cm")
    print("\n사용법:")
    print("  1. 마우스로 3점을 클릭하여 선택")
    print("  2. 자동으로 실시간 추적 시작")
    print("  3. 카메라를 좌우로 움직이면 각 점과의 거리가 실시간으로 표시됩니다")
    print("  4. 'm' 키를 누르면 1초 단위로 거리 측정 시작 (다시 'm' 키로 중지)")
    print("  5. 오른쪽 상단에서 3점의 정렬 상태를 실시간으로 확인할 수 있습니다")
    print("\n정렬 측정 항목:")
    print("  - Z-axis range: 3점의 Z축 편차 (1mm 이하면 녹색)")
    print("  - Collinearity: 3점의 일직선 정도 (0.05 이하면 녹색)")
    print("  - Status: ALIGNED (녹색 체크) 또는 NOT ALIGNED (빨간색 X)")
    print("\n단축키:")
    print("  'g' - 실시간 그래프 표시 (3점의 거리 vs 시간)")
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
    
    # picamera2는 스트리밍 설정이 필요 없음 (이미 start()로 시작됨)
    if picam is not None:
        print("[INFO] Raspberry Pi 카메라 스트리밍 준비 완료")
    
    while True:
        # 프레임 읽기
        if picam is not None:
            try:
                # picamera2로 프레임 캡처 (BGR888 포맷)
                frame = picam.capture_array()
                
                if frame is None or frame.size == 0:
                    time.sleep(0.01)
                    continue
            except Exception as e:
                print(f"[ERROR] 프레임 읽기 오류: {e}")
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
            
            # 그래프 상태 표시
            if graph_enabled:
                cv2.putText(frame, "[Graph: ON] Press 'G' to restart", 
                           (20, 120), FONT, 0.6, (0, 255, 255), 1)
            else:
                cv2.putText(frame, "[Press 'G' for real-time graph]", 
                           (20, 120), FONT, 0.6, (200, 200, 200), 1)
        
        # 선택된/추적 중인 점들 표시 및 거리 계산
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        
        # 현재 프레임의 측정 데이터 저장용
        current_measurements = []
        points_3d = []  # 정렬 측정용 3D 좌표 리스트
        
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
                
                # 3D 좌표 저장 (정렬 측정용)
                points_3d.append((X, Y, Z))
                
                # 측정 데이터 저장
                current_measurements.append({
                    'X': X * 100,  # m → cm
                    'Y': Y * 100,
                    'Z': Z * 100,
                    'distance': ema_distances[i] * 100
                })
                
                # 화면에 표시
                y_offset = 150 + i * 130
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
        
        # 그래프 데이터 업데이트 (그래프가 활성화된 경우)
        if graph_enabled and len(tracked_points) == 3 and len(ema_distances) == 3:
            if all(d is not None for d in ema_distances):
                distances_cm = [d * 100 for d in ema_distances]  # m → cm
                update_graph_data(distances_cm)
        
        # 정렬 상태 측정 및 표시
        if len(points_3d) == 3:
            alignment = calculate_alignment_metrics(points_3d)
            if alignment is not None:
                # 정렬 상태 표시 영역 (오른쪽 상단)
                align_x = w_img - 480
                align_y = 120
                
                # 배경 박스
                cv2.rectangle(frame, (align_x - 10, align_y - 30), 
                             (w_img - 10, align_y + 160), (0, 0, 0), -1)
                cv2.rectangle(frame, (align_x - 10, align_y - 30), 
                             (w_img - 10, align_y + 160), (255, 255, 255), 2)
                
                # 제목
                cv2.putText(frame, "=== ALIGNMENT STATUS ===", 
                           (align_x, align_y), FONT, 0.7, (255, 255, 255), 2)
                
                # Z축 편차
                z_range_mm = alignment['z_range'] * 1000  # m → mm
                z_color = (0, 255, 0) if z_range_mm < 1.0 else (0, 165, 255) if z_range_mm < 2.0 else (0, 0, 255)
                cv2.putText(frame, f"Z-axis range: {z_range_mm:.2f}mm", 
                           (align_x, align_y + 35), FONT, 0.6, z_color, 2)
                
                # 공선성
                col_norm = alignment['collinearity_normalized']
                col_color = (0, 255, 0) if col_norm < 0.05 else (0, 165, 255) if col_norm < 0.1 else (0, 0, 255)
                cv2.putText(frame, f"Collinearity: {col_norm:.4f}", 
                           (align_x, align_y + 70), FONT, 0.6, col_color, 2)
                
                # 정렬 상태
                if alignment['is_aligned']:
                    status_text = "ALIGNED!"
                    status_color = (0, 255, 0)
                    # 체크 마크
                    cv2.circle(frame, (w_img - 50, align_y + 115), 20, (0, 255, 0), 3)
                    cv2.line(frame, (w_img - 58, align_y + 115), (w_img - 50, align_y + 123), (0, 255, 0), 3)
                    cv2.line(frame, (w_img - 50, align_y + 123), (w_img - 38, align_y + 105), (0, 255, 0), 3)
                else:
                    status_text = "NOT ALIGNED"
                    status_color = (0, 0, 255)
                    # X 마크
                    cv2.line(frame, (w_img - 65, align_y + 100), (w_img - 35, align_y + 130), (0, 0, 255), 3)
                    cv2.line(frame, (w_img - 35, align_y + 100), (w_img - 65, align_y + 130), (0, 0, 255), 3)
                
                cv2.putText(frame, status_text, 
                           (align_x, align_y + 120), FONT, 0.8, status_color, 2)
                
                # 가이드 메시지
                if not alignment['is_aligned']:
                    cv2.putText(frame, "Adjust motor positions", 
                               (align_x, align_y + 150), FONT, 0.5, (255, 200, 0), 1)
        
        # 1초마다 측정 기록
        if measurement_active and tracking_active and len(current_measurements) == 3:
            if current_time - last_measurement_time >= 1.0:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                
                # 정렬 정보도 함께 저장
                alignment_info = calculate_alignment_metrics(points_3d) if len(points_3d) == 3 else None
                
                measurement_log.append({
                    'timestamp': timestamp,
                    'points': current_measurements,
                    'alignment': alignment_info
                })
                last_measurement_time = current_time
                
                # 콘솔 출력
                align_status = "ALIGNED" if (alignment_info and alignment_info['is_aligned']) else "NOT ALIGNED"
                print(f"[MEASUREMENT] {timestamp} - "
                      f"P1: {current_measurements[0]['distance']:.2f}cm, "
                      f"P2: {current_measurements[1]['distance']:.2f}cm, "
                      f"P3: {current_measurements[2]['distance']:.2f}cm - "
                      f"{align_status}")
        
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
        elif key == ord('g') or key == ord('G'):
            # 실시간 그래프 시작
            if tracking_active:
                start_graph_thread()
            else:
                print("[WARNING] 먼저 3점을 선택해주세요!")
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
        try:
            picam.stop()
            picam.close()
            print("[INFO] Raspberry Pi 카메라 종료")
        except:
            pass
    if cap is not None:
        cap.release()
        print("[INFO] USB 카메라 종료")
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
