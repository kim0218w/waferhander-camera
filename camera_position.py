import cv2
import numpy as np
import time
from collections import deque

# Raspberry Pi 카메라 지원
try:
    from picamera2 import Picamera2
    USE_PICAMERA = True
except ImportError:
    USE_PICAMERA = False
    print("[INFO] picamera2 not found, using standard camera")

# ===== 사용자 설정 =====
SOURCE = 0                 # 0=웹캠 (USB 카메라용)
A4_LONG_M = 0.297          # A4 긴변 (세로) 297mm = 0.297m
A4_SHORT_M = 0.210         # A4 짧은변 (가로) 210mm = 0.210m
KNOWN_DISTANCE_M = 1.50    # 's' 로 캘리브레이트할 때 실제 거리(미터)
FOCAL_PX = None            # 한 번 추정되면 여기 저장됨

# 3점 평면 좌표계 캘리브레이션 (실제 물리적 거리, 미터)
# A4 용지 기준: 왼쪽 아래를 원점으로
REFERENCE_POINTS_M = {
    'origin': (0.0, 0.0),           # 원점 (왼쪽 아래)
    'x_axis': (A4_SHORT_M, 0.0),    # X축 정의점 (오른쪽 아래, 210mm)
    'y_axis': (0.0, A4_LONG_M),     # Y축 정의점 (왼쪽 위, 297mm)
}

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ===== 전역 변수 =====
reference_points_px = {}  # 화면에서 선택한 3점의 픽셀 좌표
calibration_mode = False
current_point_name = None
homography_matrix = None

# ===== 칼만 필터 클래스 =====
class KalmanFilter1D:
    """1D 칼만 필터 (거리, X, Y, 회전각 등에 사용)"""
    def __init__(self, process_variance=1e-5, measurement_variance=1e-1):
        self.process_variance = process_variance  # 프로세스 노이즈 (작을수록 예측 신뢰)
        self.measurement_variance = measurement_variance  # 측정 노이즈 (작을수록 측정 신뢰)
        self.estimate = None
        self.error_estimate = 1.0
        
    def update(self, measurement):
        """새로운 측정값으로 필터 업데이트"""
        if self.estimate is None:
            self.estimate = measurement
            return self.estimate
        
        # Prediction
        error_prediction = self.error_estimate + self.process_variance
        
        # Update
        kalman_gain = error_prediction / (error_prediction + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error_estimate = (1 - kalman_gain) * error_prediction
        
        return self.estimate
    
    def reset(self):
        """필터 리셋"""
        self.estimate = None
        self.error_estimate = 1.0

class MovingAverageFilter:
    """이동 평균 필터"""
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
    
    def update(self, value):
        """새로운 값 추가 및 평균 계산"""
        self.values.append(value)
        return np.mean(self.values)
    
    def reset(self):
        """필터 리셋"""
        self.values.clear()

class MedianFilter:
    """중앙값 필터 (이상치 제거에 효과적)"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
    
    def update(self, value):
        """새로운 값 추가 및 중앙값 계산"""
        self.values.append(value)
        return np.median(self.values)
    
    def reset(self):
        """필터 리셋"""
        self.values.clear()

def largest_quad(img):
    """가장 큰 사각형(4점)을 찾고 꼭짓점들을 정렬해서 반환"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 60, 160)
    edges = cv2.dilate(edges, None, iterations=1)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_area = 0
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4:
            area = cv2.contourArea(approx)
            if area > best_area:
                best_area = area
                best = approx

    if best is None:
        return None

    # 시계/반시계 정렬
    pts = best.reshape(-1, 2).astype(np.float32)
    # 중심 기준 정렬
    c = pts.mean(axis=0)
    angles = np.arctan2(pts[:,1]-c[1], pts[:,0]-c[0])
    order = np.argsort(angles)
    pts = pts[order]
    return pts

def edge_lengths(quad):
    """사각형 4점에서 인접 변 길이(px) 4개와 그 중 긴변, 짧은변 길이를 반환"""
    if quad is None: 
        return None
    lengths = []
    for i in range(4):
        p1 = quad[i]
        p2 = quad[(i+1)%4]
        lengths.append(np.linalg.norm(p2 - p1))
    lengths = np.array(lengths)
    long_px = lengths.max()
    short_px = lengths.min()
    return lengths, long_px, short_px

def estimate_focal_px(h_px, H_m, D_m):
    """초점거리 추정 f_px = (h_px * D_m) / H_m"""
    return (h_px * D_m) / H_m

def distance_from_px(h_px, H_m, f_px):
    """거리 계산 D = (H * f) / h"""
    if h_px <= 0: 
        return None
    return (H_m * f_px) / h_px

def get_x_position(center_x_px, image_width, focal_px, distance_m):
    """
    카메라 중심을 기준으로 한 X축 위치 계산 (미터)
    음수=왼쪽, 양수=오른쪽
    """
    offset_px = center_x_px - (image_width / 2.0)
    x_m = (offset_px * distance_m) / focal_px
    return x_m

def get_y_position(center_y_px, image_height, focal_px, distance_m):
    """
    카메라 중심을 기준으로 한 Y축 위치 계산 (미터)
    음수=위, 양수=아래
    """
    offset_px = center_y_px - (image_height / 2.0)
    y_m = (offset_px * distance_m) / focal_px
    return y_m

def get_rotation_angle(quad):
    """
    사각형의 회전 각도 계산 (도)
    첫 번째 변의 각도를 기준으로 계산
    """
    if quad is None or len(quad) < 2:
        return None
    
    p1, p2 = quad[0], quad[1]
    angle_rad = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
    angle_deg = np.degrees(angle_rad)
    
    # 0~360 범위로 정규화
    if angle_deg < 0:
        angle_deg += 360
    
    return angle_deg

def compute_homography():
    """3점 기준으로 Homography 행렬 계산"""
    global homography_matrix
    
    if len(reference_points_px) != 3:
        return False
    
    # 픽셀 좌표 (소스)
    src_pts = []
    dst_pts = []
    
    for name in ['origin', 'x_axis', 'y_axis']:
        if name not in reference_points_px:
            return False
        src_pts.append(reference_points_px[name])
        dst_pts.append(REFERENCE_POINTS_M[name])
    
    # numpy 배열로 변환
    src_pts = np.array(src_pts, dtype=np.float32)
    dst_pts = np.array(dst_pts, dtype=np.float32)
    
    # Affine 변환 계산 (3점)
    homography_matrix = cv2.getAffineTransform(src_pts, dst_pts)
    
    print(f"[INFO] Homography matrix computed successfully!")
    print(f"[INFO] Reference points set:")
    for name, pt_px in reference_points_px.items():
        pt_m = REFERENCE_POINTS_M[name]
        print(f"  {name}: pixel{pt_px} -> real{pt_m}")
    
    return True

def transform_to_plane_coords(point_px):
    """
    픽셀 좌표를 평면 좌표계(미터)로 변환
    
    Args:
        point_px: (x, y) 픽셀 좌표
    
    Returns:
        (x_m, y_m) 평면 좌표 (미터)
    """
    if homography_matrix is None:
        return None
    
    # Homogeneous 좌표로 변환
    pt = np.array([point_px[0], point_px[1], 1.0], dtype=np.float32)
    
    # Affine 변환 적용
    transformed = homography_matrix @ pt
    
    return (transformed[0], transformed[1])

def mouse_callback(event, x, y, flags, param):
    """마우스 클릭으로 기준점 설정"""
    global reference_points_px, calibration_mode, current_point_name, homography_matrix
    
    if event == cv2.EVENT_LBUTTONDOWN and calibration_mode:
        if current_point_name is not None:
            reference_points_px[current_point_name] = (x, y)
            print(f"[INFO] {current_point_name} set at pixel ({x}, {y})")
            
            # 다음 포인트로 이동
            if current_point_name == 'origin':
                current_point_name = 'x_axis'
                print("[INFO] Click on X-axis point (right corner)")
            elif current_point_name == 'x_axis':
                current_point_name = 'y_axis'
                print("[INFO] Click on Y-axis point (top corner)")
            elif current_point_name == 'y_axis':
                current_point_name = None
                calibration_mode = False
                # Homography 계산
                if compute_homography():
                    print("[INFO] Plane calibration completed! Press 'p' to toggle plane coordinate display")

def main():
    global FOCAL_PX, calibration_mode, current_point_name, reference_points_px, homography_matrix
    
    print("[INFO] 카메라 초기화 중...")
    
    # Raspberry Pi 카메라 또는 USB 카메라 초기화
    picam = None
    cap = None
    
    if USE_PICAMERA:
        try:
            print("[INFO] Raspberry Pi 카메라 모듈 사용")
            picam = Picamera2()
            # 카메라 설정
            config = picam.create_preview_configuration(
                main={"size": (1280, 720), "format": "RGB888"}
            )
            picam.configure(config)
            picam.start()
            
            # 카메라 워밍업
            print("[INFO] 카메라 워밍업 중...")
            time.sleep(2)
            
            # 테스트 프레임
            test_frame = picam.capture_array()
            if test_frame is None:
                print("[ERROR] 카메라에서 프레임을 읽을 수 없습니다!")
                picam.stop()
                return
            
            print(f"[INFO] 카메라 초기화 성공! 해상도: {test_frame.shape[1]}x{test_frame.shape[0]}")
            
        except Exception as e:
            print(f"[ERROR] Raspberry Pi 카메라 초기화 실패: {e}")
            print("[ERROR] 'libcamera-hello' 명령어로 카메라가 작동하는지 확인하세요.")
            return
    else:
        # USB 카메라 사용
        print("[INFO] USB 카메라 사용")
        cap = cv2.VideoCapture(SOURCE)
        if not cap.isOpened():
            print("[ERROR] 카메라를 열 수 없습니다!")
            print("[ERROR] 카메라가 연결되어 있는지 확인하세요.")
            return
        
        # 카메라 설정
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # 카메라 워밍업
        print("[INFO] 카메라 워밍업 중...")
        for _ in range(5):
            cap.read()
            time.sleep(0.1)
        
        # 테스트 프레임
        ret, test_frame = cap.read()
        if not ret or test_frame is None:
            print("[ERROR] 카메라에서 프레임을 읽을 수 없습니다!")
            cap.release()
            return
        
        print(f"[INFO] 카메라 초기화 성공! 해상도: {test_frame.shape[1]}x{test_frame.shape[0]}")

    # ===== 필터링 설정 =====
    FILTER_MODE = 'kalman'  # 'none', 'ema', 'kalman', 'moving_avg', 'median'
    EMA_ALPHA = 0.2  # EMA alpha (낮을수록 부드럽지만 느림)
    
    # 필터 초기화
    if FILTER_MODE == 'kalman':
        filter_d = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-2)
        filter_x = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-2)
        filter_y = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-2)
        filter_rot = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-1)
    elif FILTER_MODE == 'moving_avg':
        filter_d = MovingAverageFilter(window_size=10)
        filter_x = MovingAverageFilter(window_size=10)
        filter_y = MovingAverageFilter(window_size=10)
        filter_rot = MovingAverageFilter(window_size=10)
    elif FILTER_MODE == 'median':
        filter_d = MedianFilter(window_size=7)
        filter_x = MedianFilter(window_size=7)
        filter_y = MedianFilter(window_size=7)
        filter_rot = MedianFilter(window_size=7)
    else:
        filter_d = filter_x = filter_y = filter_rot = None
    
    # EMA 변수들 (EMA 모드용)
    ema_d = None
    ema_x = None
    ema_y = None
    ema_rot = None
    
    # 스냅샷 모드
    snapshot_mode = False
    snapshot_values = {'d': None, 'x': None, 'y': None, 'rot': None}
    
    # 현재 필터링된 값들
    filtered_d = None
    filtered_x = None
    filtered_y = None
    filtered_rot = None

    # 평면 좌표 표시 모드
    show_plane_coords = False

    # 창 생성 및 마우스 콜백 설정
    window_name = "A4 Position Tracker"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        cv2.setMouseCallback(window_name, mouse_callback)
        print("[INFO] 디스플레이 창 생성 성공")
    except Exception as e:
        print(f"[ERROR] 디스플레이 창 생성 실패: {e}")
        print("[ERROR] X11 디스플레이가 활성화되어 있는지 확인하세요.")
        cap.release()
        return

    print("\n" + "="*70)
    print(" A4 3D Position and Plane Coordinate Tracker")
    print("="*70)
    print(f"\n[FILTER MODE: {FILTER_MODE.upper()}]")
    print("\nControls:")
    print("  's' - Calibrate focal length (camera distance)")
    print("  'c' - Start plane calibration (3-point setup)")
    print("  'p' - Toggle plane coordinate display")
    print("  'r' - Reset plane calibration")
    print("  'f' - Freeze/Unfreeze values (snapshot mode)")
    print("  '1-5' - Change filter mode (1=None, 2=EMA, 3=Kalman, 4=MovingAvg, 5=Median)")
    print("  'q' - Quit")
    print("\nPlane Calibration Steps:")
    print("  1. Press 'c'")
    print("  2. Click on Origin point (bottom-left corner of A4)")
    print("  3. Click on X-axis point (bottom-right corner)")
    print("  4. Click on Y-axis point (top-left corner)")
    print("\nStabilization Tips:")
    print("  - Press 'f' to freeze current values")
    print("  - Kalman filter (default) provides best stability")
    print("  - Adjust lighting and camera position for better tracking")
    print("="*70 + "\n")
    print("[INFO] 프로그램 실행 중... 종료하려면 'q'를 누르세요.")

    while True:
        # 프레임 읽기 (picamera2 또는 USB 카메라)
        if picam is not None:
            frame = picam.capture_array()
            if frame is None:
                print("[WARN] 프레임 읽기 실패, 재시도 중...")
                time.sleep(0.1)
                continue
        else:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[WARN] 프레임 읽기 실패, 재시도 중...")
                time.sleep(0.1)
                continue

        h_img, w_img = frame.shape[:2]
        quad = largest_quad(frame)
        
        # 캘리브레이션 모드 표시
        if calibration_mode:
            cv2.putText(frame, f"CALIBRATION: Click {current_point_name}", 
                       (20, 30), FONT, 0.8, (0, 255, 255), 2)
        
        # 기준점 표시
        for name, pt in reference_points_px.items():
            color = (255, 0, 0) if name == 'origin' else (0, 255, 255) if name == 'x_axis' else (255, 0, 255)
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 8, color, -1)
            cv2.putText(frame, name, (int(pt[0])+10, int(pt[1])-10), 
                       FONT, 0.5, color, 2)
        
        if quad is not None:
            # 변 길이 측정
            edge_result = edge_lengths(quad)
            if edge_result is None:
                continue
            
            _, long_px, short_px = edge_result

            # 화면에 외곽선 그리기
            cv2.polylines(frame, [quad.astype(int)], True, (0,255,0), 2)

            # 중심점 계산
            center = quad.mean(axis=0)
            center_x_px, center_y_px = center[0], center[1]
            
            # 중심점 표시
            cv2.circle(frame, (int(center_x_px), int(center_y_px)), 5, (0,0,255), -1)

            # A4 긴변/짧은변 판별
            H_real_m = A4_LONG_M if long_px >= short_px else A4_SHORT_M
            h_px = max(long_px, short_px)

            # === 카메라 좌표계 기반 3D 위치 ===
            if FOCAL_PX is not None:
                # 거리 계산 (Z축)
                D = distance_from_px(h_px, H_real_m, FOCAL_PX)
                
                if D is not None:
                    # X, Y 위치 계산
                    x_m = get_x_position(center_x_px, w_img, FOCAL_PX, D)
                    y_m = get_y_position(center_y_px, h_img, FOCAL_PX, D)
                    
                    # 회전각 계산
                    rotation = get_rotation_angle(quad)
                    
                    # === 필터링 적용 ===
                    if snapshot_mode:
                        # 스냅샷 모드: 고정된 값 사용
                        filtered_d = snapshot_values['d']
                        filtered_x = snapshot_values['x']
                        filtered_y = snapshot_values['y']
                        filtered_rot = snapshot_values['rot']
                    elif FILTER_MODE == 'none':
                        # 필터 없음: 원본 값 사용
                        filtered_d = D
                        filtered_x = x_m
                        filtered_y = y_m
                        filtered_rot = rotation
                    elif FILTER_MODE == 'ema':
                        # EMA 필터 적용
                        ema_d = D if ema_d is None else (EMA_ALPHA*D + (1-EMA_ALPHA)*ema_d)
                        ema_x = x_m if ema_x is None else (EMA_ALPHA*x_m + (1-EMA_ALPHA)*ema_x)
                        ema_y = y_m if ema_y is None else (EMA_ALPHA*y_m + (1-EMA_ALPHA)*ema_y)
                        if rotation is not None:
                            ema_rot = rotation if ema_rot is None else (EMA_ALPHA*rotation + (1-EMA_ALPHA)*ema_rot)
                        filtered_d = ema_d
                        filtered_x = ema_x
                        filtered_y = ema_y
                        filtered_rot = ema_rot
                    else:
                        # Kalman, Moving Average, Median 필터 적용
                        filtered_d = filter_d.update(D)
                        filtered_x = filter_x.update(x_m)
                        filtered_y = filter_y.update(y_m)
                        if rotation is not None:
                            filtered_rot = filter_rot.update(rotation)
                    
                    # === 카메라 좌표계 표시 ===
                    y_offset = 60 if calibration_mode else 30
                    
                    # 필터 모드 표시
                    mode_text = f"[{FILTER_MODE.upper()}]" + (" [FROZEN]" if snapshot_mode else "")
                    cv2.putText(frame, mode_text, (20, y_offset-25), FONT, 0.5, (255,255,0), 2)
                    
                    cv2.putText(frame, "=== Camera Coordinates ===", 
                               (20, y_offset), FONT, 0.6, (255,255,255), 2)
                    
                    if filtered_d is not None:
                        cv2.putText(frame, f"Z(Distance): {filtered_d:.3f} m", 
                                   (20, y_offset+30), FONT, 0.7, (0,255,0), 2)
                    if filtered_x is not None:
                        cv2.putText(frame, f"X(Horizontal): {filtered_x:+.3f} m", 
                                   (20, y_offset+60), FONT, 0.7, (0,255,0), 2)
                    if filtered_y is not None:
                        cv2.putText(frame, f"Y(Vertical): {filtered_y:+.3f} m", 
                                   (20, y_offset+90), FONT, 0.7, (0,255,0), 2)
                    if filtered_rot is not None:
                        cv2.putText(frame, f"Rotation: {filtered_rot:.1f} deg", 
                                   (20, y_offset+120), FONT, 0.7, (0,255,0), 2)
                    
                    # === 평면 좌표계 표시 ===
                    if show_plane_coords and homography_matrix is not None:
                        plane_coords = transform_to_plane_coords((center_x_px, center_y_px))
                        
                        if plane_coords is not None:
                            plane_x, plane_y = plane_coords
                            cv2.putText(frame, "=== Plane Coordinates ===", 
                                       (20, y_offset+160), FONT, 0.6, (255,255,0), 2)
                            cv2.putText(frame, f"X(on plane): {plane_x:.4f} m ({plane_x*1000:.1f} mm)", 
                                       (20, y_offset+190), FONT, 0.6, (255,255,0), 2)
                            cv2.putText(frame, f"Y(on plane): {plane_y:.4f} m ({plane_y*1000:.1f} mm)", 
                                       (20, y_offset+220), FONT, 0.6, (255,255,0), 2)
                            
                            # A4 용지 범위 체크
                            in_range = (0 <= plane_x <= A4_SHORT_M) and (0 <= plane_y <= A4_LONG_M)
                            status = "IN RANGE" if in_range else "OUT OF RANGE"
                            color = (0,255,0) if in_range else (0,0,255)
                            cv2.putText(frame, f"Status: {status}", 
                                       (20, y_offset+250), FONT, 0.6, color, 2)
                    
            else:
                cv2.putText(frame, "Press 's' to calibrate focal length", 
                           (20,40), FONT, 0.8, (0,255,255), 2)
                cv2.putText(frame, f"h_px(longest edge): {h_px:.1f}", 
                           (20,70), FONT, 0.7, (0,255,0), 2)

        else:
            y_offset = 60 if calibration_mode else 30
            cv2.putText(frame, "A4 not found", (20, y_offset), FONT, 0.8, (0,0,255), 2)

        try:
            cv2.imshow(window_name, frame)
        except Exception as e:
            print(f"[ERROR] 프레임 표시 실패: {e}")
            break
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            # 초점거리 캘리브레이션
            if quad is None:
                print("[WARN] A4가 화면에 보일 때 눌러주세요.")
            else:
                edge_result = edge_lengths(quad)
                if edge_result is None:
                    print("[WARN] A4 변 길이를 측정할 수 없습니다.")
                    continue
                
                _, long_px, short_px = edge_result
                H_real_m = A4_LONG_M if long_px >= short_px else A4_SHORT_M
                h_px = max(long_px, short_px)
                FOCAL_PX = estimate_focal_px(h_px, H_real_m, KNOWN_DISTANCE_M)
                
                # 필터 리셋
                ema_d = ema_x = ema_y = ema_rot = None
                if FILTER_MODE in ['kalman', 'moving_avg', 'median']:
                    if hasattr(filter_d, 'reset'):
                        filter_d.reset()
                        filter_x.reset()
                        filter_y.reset()
                        filter_rot.reset()
                
                print(f"[INFO] Focal length calibrated: f_px = {FOCAL_PX:.2f}")
                print(f"       (distance={KNOWN_DISTANCE_M}m, h_px={h_px:.1f}px, H={H_real_m}m)")
        
        elif key == ord('c'):
            # 평면 좌표계 캘리브레이션 시작
            if quad is None:
                print("[WARN] A4가 화면에 보일 때 시작해주세요.")
            else:
                calibration_mode = True
                current_point_name = 'origin'
                reference_points_px = {}
                homography_matrix = None
                print("\n[INFO] Plane calibration started")
                print("[INFO] Click on Origin point (bottom-left corner of A4)")
        
        elif key == ord('p'):
            # 평면 좌표 표시 토글
            show_plane_coords = not show_plane_coords
            status = "ON" if show_plane_coords else "OFF"
            print(f"[INFO] Plane coordinate display: {status}")
        
        elif key == ord('r'):
            # 평면 캘리브레이션 리셋
            reference_points_px = {}
            homography_matrix = None
            calibration_mode = False
            current_point_name = None
            show_plane_coords = False
            print("[INFO] Plane calibration reset")
        
        elif key == ord('f'):
            # 스냅샷 모드 토글 (값 고정/해제)
            snapshot_mode = not snapshot_mode
            if snapshot_mode:
                # 현재 값을 스냅샷으로 저장
                snapshot_values['d'] = filtered_d
                snapshot_values['x'] = filtered_x
                snapshot_values['y'] = filtered_y
                snapshot_values['rot'] = filtered_rot
                print(f"[INFO] Values FROZEN at: D={filtered_d:.3f}m, X={filtered_x:+.3f}m, Y={filtered_y:+.3f}m")
            else:
                print("[INFO] Values UNFROZEN - tracking resumed")
        
        elif key == ord('1'):
            # 필터 없음
            FILTER_MODE = 'none'
            print("[INFO] Filter mode: NONE (no filtering)")
        
        elif key == ord('2'):
            # EMA 필터
            FILTER_MODE = 'ema'
            ema_d = ema_x = ema_y = ema_rot = None
            print(f"[INFO] Filter mode: EMA (alpha={EMA_ALPHA})")
        
        elif key == ord('3'):
            # Kalman 필터
            FILTER_MODE = 'kalman'
            filter_d = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-2)
            filter_x = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-2)
            filter_y = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-2)
            filter_rot = KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-1)
            print("[INFO] Filter mode: KALMAN (best stability)")
        
        elif key == ord('4'):
            # Moving Average 필터
            FILTER_MODE = 'moving_avg'
            filter_d = MovingAverageFilter(window_size=10)
            filter_x = MovingAverageFilter(window_size=10)
            filter_y = MovingAverageFilter(window_size=10)
            filter_rot = MovingAverageFilter(window_size=10)
            print("[INFO] Filter mode: MOVING AVERAGE (10 frames)")
        
        elif key == ord('5'):
            # Median 필터
            FILTER_MODE = 'median'
            filter_d = MedianFilter(window_size=7)
            filter_x = MedianFilter(window_size=7)
            filter_y = MedianFilter(window_size=7)
            filter_rot = MedianFilter(window_size=7)
            print("[INFO] Filter mode: MEDIAN (noise reduction)")

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

