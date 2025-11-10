import cv2
import numpy as np
import time
import csv
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
KNOWN_DISTANCE_M = 1.0     # 캘리브레이션 시 실제 거리(미터)
REFERENCE_SIZE_PX = None   # 캘리브레이션된 기준 크기 (픽셀)
FOCAL_PX = None            # 추정된 초점거리

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ===== 전역 변수 =====
tracking_points = []  # 추적할 3개의 포인트 [(x, y), ...]
point_distances = []  # 각 포인트의 거리
calibration_mode = False
calibration_point_idx = 0

# 측정 관련 변수
measurement_interval = 1.0  # 측정 간격 (초)
last_measurement_time = 0
measurement_count = 0
measurement_log = []  # 측정 기록 저장

# ===== 칼만 필터 클래스 =====
class KalmanFilter1D:
    """1D 칼만 필터 (거리 측정에 사용)"""
    def __init__(self, process_variance=1e-5, measurement_variance=1e-1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
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

def detect_feature_points(frame, method='sift', max_points=100):
    """
    프레임에서 특징점 검출
    
    Args:
        frame: 입력 이미지
        method: 'sift', 'orb', 'good_features' 중 선택
        max_points: 최대 검출 포인트 수
    
    Returns:
        keypoints: 검출된 특징점 리스트
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    if method == 'sift':
        try:
            detector = cv2.SIFT_create(nfeatures=max_points)
            keypoints = detector.detect(gray, None)
        except:
            print("[WARN] SIFT not available, using ORB")
            method = 'orb'
    
    if method == 'orb':
        detector = cv2.ORB_create(nfeatures=max_points)
        keypoints = detector.detect(gray, None)
    
    elif method == 'good_features':
        corners = cv2.goodFeaturesToTrack(gray, max_points, 0.01, 10)
        if corners is not None:
            keypoints = [cv2.KeyPoint(x=c[0][0], y=c[0][1], size=1) for c in corners]
        else:
            keypoints = []
    
    return keypoints

def estimate_distance_from_reference(current_size_px, reference_size_px, reference_distance_m):
    """
    기준 크기와 현재 크기를 비교하여 거리 추정
    거리 = 기준거리 * (기준크기 / 현재크기)
    """
    if current_size_px <= 0:
        return None
    return reference_distance_m * (reference_size_px / current_size_px)

def calculate_point_distances(points, center_x, center_y):
    """
    각 포인트와 중심점 간의 픽셀 거리 계산
    
    Returns:
        distances: 각 포인트까지의 픽셀 거리
        avg_distance: 평균 거리 (거리 추정에 사용)
    """
    if len(points) == 0:
        return [], 0
    
    distances = []
    for px, py in points:
        dist = np.sqrt((px - center_x)**2 + (py - center_y)**2)
        distances.append(dist)
    
    avg_distance = np.mean(distances) if distances else 0
    return distances, avg_distance

def mouse_callback(event, x, y, flags, param):
    """마우스 클릭으로 3개의 추적 포인트 설정"""
    global tracking_points, calibration_mode, calibration_point_idx, REFERENCE_SIZE_PX
    
    if event == cv2.EVENT_LBUTTONDOWN:
        if calibration_mode and len(tracking_points) < 3:
            tracking_points.append((x, y))
            print(f"[INFO] Point {len(tracking_points)} set at pixel ({x}, {y})")
            
            if len(tracking_points) == 3:
                calibration_mode = False
                # 3점의 중심점 계산
                center_x = np.mean([p[0] for p in tracking_points])
                center_y = np.mean([p[1] for p in tracking_points])
                
                # 3점 간의 평균 거리를 기준 크기로 사용
                _, avg_dist = calculate_point_distances(tracking_points, center_x, center_y)
                REFERENCE_SIZE_PX = avg_dist
                
                print(f"[INFO] 3 points selected. Center: ({center_x:.1f}, {center_y:.1f})")
                print(f"[INFO] Reference size: {REFERENCE_SIZE_PX:.2f} pixels")
                print("[INFO] Now press 's' to calibrate distance")

def save_measurement_log(filename='measurement_log.csv'):
    """측정 기록을 CSV 파일로 저장"""
    global measurement_log
    
    if len(measurement_log) == 0:
        print("[WARN] No measurements to save")
        return
    
    try:
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Count', 'Distance(m)', 'Distance(mm)', 
                           'P1_x', 'P1_y', 'P2_x', 'P2_y', 'P3_x', 'P3_y',
                           'P1_dist_px', 'P2_dist_px', 'P3_dist_px'])
            writer.writerows(measurement_log)
        print(f"[INFO] Measurement log saved to {filename}")
    except Exception as e:
        print(f"[ERROR] Failed to save log: {e}")

def main():
    global FOCAL_PX, REFERENCE_SIZE_PX, KNOWN_DISTANCE_M, calibration_mode, tracking_points, point_distances
    global measurement_interval, last_measurement_time, measurement_count, measurement_log
    
    print("[INFO] 카메라 초기화 중...")
    
    # Raspberry Pi 카메라 또는 USB 카메라 초기화
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
            
            print("[INFO] 카메라 워밍업 중...")
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
        
        print("[INFO] 카메라 워밍업 중...")
        for _ in range(5):
            cap.read()
            time.sleep(0.1)
        
        ret, test_frame = cap.read()
        if not ret or test_frame is None:
            print("[ERROR] 카메라에서 프레임을 읽을 수 없습니다!")
            cap.release()
            return
        
        print(f"[INFO] 카메라 초기화 성공! 해상도: {test_frame.shape[1]}x{test_frame.shape[0]}")

    # 필터 초기화 (각 포인트마다)
    filters = [KalmanFilter1D(process_variance=1e-5, measurement_variance=1e-2) for _ in range(3)]
    
    # 창 생성
    window_name = "3-Point Distance Tracker"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        cv2.setMouseCallback(window_name, mouse_callback)
        print("[INFO] 디스플레이 창 생성 성공")
    except Exception as e:
        print(f"[ERROR] 디스플레이 창 생성 실패: {e}")
        if cap:
            cap.release()
        return

    print("\n" + "="*70)
    print(" 3-Point Distance Tracker")
    print("="*70)
    print("\nControls:")
    print("  'c' - Start point selection (select 3 points on object)")
    print("  's' - Calibrate distance (set known distance)")
    print("  'r' - Reset points")
    print("  'd' - Toggle feature point detection")
    print("  'm' - Start/Stop 1-second interval measurement")
    print("  'l' - Save measurement log to CSV")
    print("  'q' - Quit")
    print("\nCalibration Steps:")
    print("  1. Press 'c' and click 3 points on the object")
    print("  2. Position object at known distance (default: 1.0m)")
    print("  3. Press 's' to calibrate")
    print("  4. Press 'm' to start automatic measurements every 1 second")
    print("="*70 + "\n")
    print("[INFO] 프로그램 실행 중... 종료하려면 'q'를 누르세요.")

    show_features = False
    measurement_mode = False  # 1초 간격 측정 모드
    
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
        display = frame.copy()
        
        # 특징점 검출 표시
        if show_features:
            keypoints = detect_feature_points(frame, method='good_features', max_points=50)
            for kp in keypoints:
                x, y = int(kp.pt[0]), int(kp.pt[1])
                cv2.circle(display, (x, y), 3, (0, 255, 255), -1)
        
        # 캘리브레이션 모드 표시
        if calibration_mode:
            cv2.putText(display, f"Click point {len(tracking_points) + 1}/3 on the object", 
                       (20, 30), FONT, 0.8, (0, 255, 255), 2)
        
        # 추적 포인트가 설정되었을 때
        if len(tracking_points) == 3:
            # 3점의 중심 계산
            center_x = np.mean([p[0] for p in tracking_points])
            center_y = np.mean([p[1] for p in tracking_points])
            
            # 중심점 표시
            cv2.circle(display, (int(center_x), int(center_y)), 8, (255, 0, 255), -1)
            cv2.putText(display, "Center", (int(center_x)+10, int(center_y)-10), 
                       FONT, 0.5, (255, 0, 255), 2)
            
            # 각 포인트 표시 및 선 연결
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
            point_names = ["P1", "P2", "P3"]
            
            for i, (px, py) in enumerate(tracking_points):
                cv2.circle(display, (int(px), int(py)), 8, colors[i], -1)
                cv2.putText(display, point_names[i], (int(px)+10, int(py)-10), 
                           FONT, 0.6, colors[i], 2)
                # 중심점과 연결
                cv2.line(display, (int(center_x), int(center_y)), (int(px), int(py)), 
                        colors[i], 2)
            
            # 3점을 연결하는 삼각형 그리기
            pts = np.array(tracking_points, dtype=np.int32)
            cv2.polylines(display, [pts], True, (255, 255, 0), 2)
            
            # 각 포인트와 중심 간의 픽셀 거리 계산
            pixel_distances, avg_pixel_dist = calculate_point_distances(
                tracking_points, center_x, center_y
            )
            
            # 거리 추정 (캘리브레이션이 완료된 경우)
            if REFERENCE_SIZE_PX is not None and REFERENCE_SIZE_PX > 0:
                estimated_distance_m = estimate_distance_from_reference(
                    avg_pixel_dist, REFERENCE_SIZE_PX, KNOWN_DISTANCE_M
                )
                
                if estimated_distance_m is not None:
                    # 필터 적용 (평균 거리만 필터링)
                    filtered_distance = filters[0].update(estimated_distance_m)
                    
                    # 1초 간격 측정
                    current_time = time.time()
                    if measurement_mode and (current_time - last_measurement_time) >= measurement_interval:
                        measurement_count += 1
                        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                        
                        # 측정 데이터 기록
                        measurement_data = [
                            timestamp,
                            measurement_count,
                            f"{filtered_distance:.4f}",
                            f"{filtered_distance*1000:.1f}",
                            int(tracking_points[0][0]), int(tracking_points[0][1]),
                            int(tracking_points[1][0]), int(tracking_points[1][1]),
                            int(tracking_points[2][0]), int(tracking_points[2][1]),
                            f"{pixel_distances[0]:.2f}",
                            f"{pixel_distances[1]:.2f}",
                            f"{pixel_distances[2]:.2f}"
                        ]
                        measurement_log.append(measurement_data)
                        
                        # 콘솔에 출력
                        print(f"\n[MEASUREMENT #{measurement_count}] {timestamp}")
                        print(f"  Distance: {filtered_distance:.4f}m ({filtered_distance*1000:.1f}mm)")
                        print(f"  P1: ({int(tracking_points[0][0])}, {int(tracking_points[0][1])}) - {pixel_distances[0]:.2f}px from center")
                        print(f"  P2: ({int(tracking_points[1][0])}, {int(tracking_points[1][1])}) - {pixel_distances[1]:.2f}px from center")
                        print(f"  P3: ({int(tracking_points[2][0])}, {int(tracking_points[2][1])}) - {pixel_distances[2]:.2f}px from center")
                        
                        last_measurement_time = current_time
                    
                    # 정보 표시
                    y_offset = 60
                    
                    # 측정 모드 표시
                    if measurement_mode:
                        time_until_next = max(0, measurement_interval - (current_time - last_measurement_time))
                        mode_text = f"[MEASURING] Count: {measurement_count} | Next in: {time_until_next:.1f}s"
                        cv2.putText(display, mode_text, (20, 30), FONT, 0.6, (0, 255, 0), 2)
                    else:
                        cv2.putText(display, "[IDLE] Press 'm' to start measurements", 
                                   (20, 30), FONT, 0.6, (200, 200, 200), 2)
                    
                    cv2.putText(display, "=== Distance Measurements ===", 
                               (20, y_offset), FONT, 0.7, (255, 255, 255), 2)
                    
                    cv2.putText(display, f"Estimated Distance: {filtered_distance:.4f} m ({filtered_distance*1000:.1f} mm)", 
                               (20, y_offset+35), FONT, 0.7, (0, 255, 0), 2)
                    
                    cv2.putText(display, "=== Point Details ===", 
                               (20, y_offset+70), FONT, 0.6, (255, 255, 255), 2)
                    
                    # 각 포인트의 정보 표시
                    for i, (px, py) in enumerate(tracking_points):
                        pixel_dist = pixel_distances[i]
                        
                        # 픽셀 좌표와 중심으로부터의 픽셀 거리
                        info_text = f"{point_names[i]}: ({int(px)}, {int(py)}) | Dist from center: {pixel_dist:.2f}px"
                        cv2.putText(display, info_text, 
                                   (20, y_offset+100+i*30), FONT, 0.5, colors[i], 2)
                        
                        # 각 포인트의 실제 거리 추정 (중심 거리 기준)
                        point_distance_m = estimated_distance_m
                        cv2.putText(display, f"   Estimated: {point_distance_m:.4f}m ({point_distance_m*1000:.1f}mm)", 
                                   (20, y_offset+120+i*30), FONT, 0.5, colors[i], 2)
                    
                    # 3점 간의 거리 (픽셀)
                    dist_01 = np.sqrt((tracking_points[0][0]-tracking_points[1][0])**2 + 
                                     (tracking_points[0][1]-tracking_points[1][1])**2)
                    dist_12 = np.sqrt((tracking_points[1][0]-tracking_points[2][0])**2 + 
                                     (tracking_points[1][1]-tracking_points[2][1])**2)
                    dist_20 = np.sqrt((tracking_points[2][0]-tracking_points[0][0])**2 + 
                                     (tracking_points[2][1]-tracking_points[0][1])**2)
                    
                    cv2.putText(display, "=== Inter-Point Distances (pixels) ===", 
                               (20, y_offset+220), FONT, 0.6, (255, 255, 255), 2)
                    cv2.putText(display, f"P1-P2: {dist_01:.2f}px | P2-P3: {dist_12:.2f}px | P3-P1: {dist_20:.2f}px", 
                               (20, y_offset+250), FONT, 0.5, (255, 255, 0), 2)
                    
                    # 평균 픽셀 거리
                    cv2.putText(display, f"Avg distance from center: {avg_pixel_dist:.2f}px", 
                               (20, y_offset+280), FONT, 0.5, (255, 255, 0), 2)
                    
                    # 캘리브레이션 정보
                    cv2.putText(display, f"Calibrated at: {KNOWN_DISTANCE_M:.2f}m with {REFERENCE_SIZE_PX:.2f}px", 
                               (20, h_img-20), FONT, 0.5, (200, 200, 200), 1)
                    
            else:
                # 캘리브레이션 필요
                cv2.putText(display, "Press 's' to calibrate distance", 
                           (20, 60), FONT, 0.8, (0, 255, 255), 2)
                cv2.putText(display, f"Position object at {KNOWN_DISTANCE_M:.2f}m", 
                           (20, 95), FONT, 0.7, (0, 255, 255), 2)
                
                # 포인트의 픽셀 좌표만 표시
                y_offset = 130
                for i, (px, py) in enumerate(tracking_points):
                    cv2.putText(display, f"{point_names[i]}: ({int(px)}, {int(py)})", 
                               (20, y_offset+i*30), FONT, 0.6, colors[i], 2)
        
        else:
            # 포인트 선택 필요
            cv2.putText(display, "Press 'c' to select 3 points on the object", 
                       (20, 60), FONT, 0.8, (0, 255, 255), 2)
        
        # 화면에 표시
        try:
            cv2.imshow(window_name, display)
        except Exception as e:
            print(f"[ERROR] 프레임 표시 실패: {e}")
            break
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        
        elif key == ord('c'):
            # 포인트 선택 시작
            calibration_mode = True
            tracking_points = []
            REFERENCE_SIZE_PX = None
            print("\n[INFO] Point selection started")
            print("[INFO] Click 3 points on the object")
        
        elif key == ord('s'):
            # 거리 캘리브레이션
            if len(tracking_points) != 3:
                print("[WARN] Please select 3 points first (press 'c')")
            elif REFERENCE_SIZE_PX is None:
                print("[WARN] Reference size not calculated")
            else:
                # 현재 거리 입력받기 (콘솔)
                try:
                    distance_input = input(f"Enter actual distance in meters (default: {KNOWN_DISTANCE_M}): ").strip()
                    if distance_input:
                        KNOWN_DISTANCE_M = float(distance_input)
                    
                    print(f"[INFO] Distance calibrated at {KNOWN_DISTANCE_M:.2f}m")
                    print(f"[INFO] Reference size: {REFERENCE_SIZE_PX:.2f} pixels")
                    
                    # 필터 리셋
                    for f in filters:
                        f.reset()
                    
                except ValueError:
                    print("[ERROR] Invalid distance value")
        
        elif key == ord('r'):
            # 리셋
            tracking_points = []
            REFERENCE_SIZE_PX = None
            calibration_mode = False
            for f in filters:
                f.reset()
            print("[INFO] Points reset")
        
        elif key == ord('d'):
            # 특징점 검출 토글
            show_features = not show_features
            status = "ON" if show_features else "OFF"
            print(f"[INFO] Feature detection: {status}")
        
        elif key == ord('m'):
            # 1초 간격 측정 모드 토글
            if len(tracking_points) != 3:
                print("[WARN] Please select 3 points first (press 'c')")
            elif REFERENCE_SIZE_PX is None:
                print("[WARN] Please calibrate distance first (press 's')")
            else:
                measurement_mode = not measurement_mode
                if measurement_mode:
                    last_measurement_time = time.time()
                    measurement_count = 0
                    measurement_log = []
                    print("\n[INFO] ===== MEASUREMENT MODE STARTED =====")
                    print("[INFO] Recording measurements every 1 second")
                    print("[INFO] Press 'm' again to stop, 'l' to save log")
                else:
                    print(f"\n[INFO] ===== MEASUREMENT MODE STOPPED =====")
                    print(f"[INFO] Total measurements: {measurement_count}")
        
        elif key == ord('l'):
            # 측정 로그 저장
            if len(measurement_log) == 0:
                print("[WARN] No measurements to save. Start measurement mode first (press 'm')")
            else:
                save_measurement_log()
                print(f"[INFO] Saved {len(measurement_log)} measurements")

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
