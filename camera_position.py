import cv2
import numpy as np
import time

# Raspberry Pi 카메라 지원
try:
    from picamera2 import Picamera2
    USE_PICAMERA = True
except ImportError:
    USE_PICAMERA = False
    print("[INFO] picamera2 not found, using standard camera")

# ===== 설정 =====
SOURCE = 0  # 0=웹캠
A4_LONG_M = 0.297   # A4 긴변 (m)
A4_SHORT_M = 0.210  # A4 짧은변 (m)
KNOWN_DISTANCE_M = 1.0  # 초기 캘리브레이션 거리 (m)
FOCAL_PX = None  # 자동 계산됨

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ===== 전역 변수 =====
selected_points = []  # 사용자가 선택한 3점 (픽셀 좌표)
point_names = ['Point 1', 'Point 2', 'Point 3']
current_point_idx = 0
selection_mode = False

def largest_quad(img):
    """가장 큰 사각형 찾기"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
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

    pts = best.reshape(-1, 2).astype(np.float32)
    c = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    order = np.argsort(angles)
    pts = pts[order]
    return pts

def edge_lengths(quad):
    """사각형 변 길이 계산"""
    if quad is None:
        return None
    lengths = []
    for i in range(4):
        p1 = quad[i]
        p2 = quad[(i + 1) % 4]
        lengths.append(np.linalg.norm(p2 - p1))
    lengths = np.array(lengths)
    return lengths, lengths.max(), lengths.min()

def estimate_focal_px(h_px, H_m, D_m):
    """초점거리 추정"""
    return (h_px * D_m) / H_m

def distance_from_px(h_px, H_m, f_px):
    """거리 계산 (Z축)"""
    if h_px <= 0:
        return None
    return (H_m * f_px) / h_px

def get_3d_position(point_px, image_width, image_height, focal_px, distance_m):
    """
    픽셀 좌표를 3D 위치로 변환
    
    Returns:
        (x, y, z) 카메라 좌표계 기준 (미터)
    """
    center_x = image_width / 2.0
    center_y = image_height / 2.0
    
    offset_x_px = point_px[0] - center_x
    offset_y_px = point_px[1] - center_y
    
    x_m = (offset_x_px * distance_m) / focal_px
    y_m = (offset_y_px * distance_m) / focal_px
    z_m = distance_m
    
    return x_m, y_m, z_m

def calculate_distance_from_camera(point_px, quad, focal_px, image_width, image_height):
    """
    선택한 점과 카메라 사이의 거리 계산
    
    Returns:
        (x, y, z, distance_3d): 3D 위치와 유클리드 거리
    """
    if quad is None or focal_px is None:
        return None
    
    # A4 용지 크기로 거리 추정
    edge_result = edge_lengths(quad)
    if edge_result is None:
        return None
    
    _, long_px, short_px = edge_result
    H_real_m = A4_LONG_M if long_px >= short_px else A4_SHORT_M
    h_px = max(long_px, short_px)
    
    # 평면 중심과의 거리 (대략적)
    D = distance_from_px(h_px, H_real_m, focal_px)
    
    if D is None:
        return None
    
    # 3D 위치 계산
    x, y, z = get_3d_position(point_px, image_width, image_height, focal_px, D)
    
    # 카메라로부터의 유클리드 거리
    distance_3d = np.sqrt(x**2 + y**2 + z**2)
    
    return x, y, z, distance_3d

def mouse_callback(event, x, y, flags, param):
    """마우스 클릭으로 점 선택"""
    global selected_points, current_point_idx, selection_mode
    
    if event == cv2.EVENT_LBUTTONDOWN and selection_mode:
        if current_point_idx < 3:
            selected_points.append((x, y))
            print(f"[INFO] {point_names[current_point_idx]} selected at ({x}, {y})")
            current_point_idx += 1
            
            if current_point_idx >= 3:
                selection_mode = False
                print("[INFO] All 3 points selected! Press 'p' to reselect")

def main():
    global FOCAL_PX, selected_points, current_point_idx, selection_mode
    
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
    
    # 창 생성
    window_name = "3-Point Distance Tracker"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        cv2.setMouseCallback(window_name, mouse_callback)
        print("[INFO] 디스플레이 창 생성 성공")
    except Exception as e:
        print(f"[ERROR] 디스플레이 창 생성 실패: {e}")
        return
    
    print("\n" + "="*70)
    print(" 3-Point Distance Tracker (Camera Coordinate System)")
    print("="*70)
    print("\n사용법:")
    print("  1. A4 용지를 화면에 보이게 하세요")
    print("  2. 's' 키를 눌러 초점거리 캘리브레이션 (1m 거리에서)")
    print("  3. 'p' 키를 눌러 3점 선택 모드 시작")
    print("  4. 마우스로 3점을 클릭하여 선택")
    print("  5. 각 점과 카메라 사이의 실시간 거리가 표시됩니다")
    print("\n단축키:")
    print("  's' - 초점거리 캘리브레이션")
    print("  'p' - 3점 선택 모드")
    print("  'r' - 선택한 점 초기화")
    print("  'q' - 종료")
    print("="*70 + "\n")
    
    # EMA 필터 (부드러운 출력)
    alpha = 0.3
    ema_distances = [None, None, None]
    
    while True:
        # 프레임 읽기
        if picam is not None:
            frame = picam.capture_array()
            if frame is None:
                print("[WARN] 프레임 읽기 실패")
                time.sleep(0.1)
                continue
        else:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[WARN] 프레임 읽기 실패")
                time.sleep(0.1)
                continue
        
        h_img, w_img = frame.shape[:2]
        quad = largest_quad(frame)
        
        # A4 용지 외곽선 표시
        if quad is not None:
            cv2.polylines(frame, [quad.astype(int)], True, (0, 255, 0), 2)
            
            # 코너 표시
            for i, corner in enumerate(quad):
                cv2.circle(frame, (int(corner[0]), int(corner[1])), 5, (255, 0, 0), -1)
        
        # 선택 모드 표시
        if selection_mode:
            cv2.putText(frame, f"Click to select {point_names[current_point_idx]}", 
                       (20, 30), FONT, 0.8, (0, 255, 255), 2)
        
        # 선택한 점 표시 및 거리 계산
        if not selection_mode and len(selected_points) == 3:
            for i, point in enumerate(selected_points):
                # 점 표시
                color = [(255, 0, 0), (0, 255, 0), (0, 0, 255)][i]
                cv2.circle(frame, point, 8, color, -1)
                cv2.putText(frame, point_names[i], 
                           (point[0] + 15, point[1] - 10), 
                           FONT, 0.5, color, 2)
                
                # 거리 계산
                if FOCAL_PX is not None and quad is not None:
                    result = calculate_distance_from_camera(point, quad, FOCAL_PX, w_img, h_img)
                    
                    if result is not None:
                        x, y, z, distance_3d = result
                        
                        # EMA 필터 적용
                        if ema_distances[i] is None:
                            ema_distances[i] = distance_3d
                        else:
                            ema_distances[i] = alpha * distance_3d + (1 - alpha) * ema_distances[i]
                        
                        # 화면에 표시
                        y_offset = 60 + i * 120
                        cv2.putText(frame, f"=== {point_names[i]} ===", 
                                   (20, y_offset), FONT, 0.6, color, 2)
                        cv2.putText(frame, f"X: {x:+.3f} m", 
                                   (20, y_offset + 25), FONT, 0.5, color, 1)
                        cv2.putText(frame, f"Y: {y:+.3f} m", 
                                   (20, y_offset + 50), FONT, 0.5, color, 1)
                        cv2.putText(frame, f"Z: {z:.3f} m", 
                                   (20, y_offset + 75), FONT, 0.5, color, 1)
                        cv2.putText(frame, f"Distance: {ema_distances[i]:.3f} m ({ema_distances[i]*1000:.1f} mm)", 
                                   (20, y_offset + 100), FONT, 0.6, color, 2)
        
        # 초점거리 캘리브레이션 안내
        if FOCAL_PX is None:
            cv2.putText(frame, "Press 's' to calibrate focal length", 
                       (20, 30), FONT, 0.8, (0, 255, 255), 2)
        elif len(selected_points) < 3:
            cv2.putText(frame, "Press 'p' to select 3 points", 
                       (20, 30), FONT, 0.8, (0, 255, 255), 2)
        
        # 화면 표시
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
                print("[WARN] A4 용지가 화면에 보일 때 눌러주세요.")
            else:
                edge_result = edge_lengths(quad)
                if edge_result is None:
                    print("[WARN] A4 변 길이를 측정할 수 없습니다.")
                    continue
                
                _, long_px, short_px = edge_result
                H_real_m = A4_LONG_M if long_px >= short_px else A4_SHORT_M
                h_px = max(long_px, short_px)
                FOCAL_PX = estimate_focal_px(h_px, H_real_m, KNOWN_DISTANCE_M)
                print(f"[INFO] Focal length calibrated: f_px = {FOCAL_PX:.2f}")
                print(f"       (distance={KNOWN_DISTANCE_M}m, h_px={h_px:.1f}px, H={H_real_m}m)")
        
        elif key == ord('p'):
            # 3점 선택 모드
            if FOCAL_PX is None:
                print("[WARN] 먼저 's' 키를 눌러 초점거리를 캘리브레이션하세요.")
            else:
                selection_mode = True
                selected_points = []
                current_point_idx = 0
                ema_distances = [None, None, None]
                print("\n[INFO] 3점 선택 모드 시작")
                print("[INFO] 마우스로 3점을 클릭하세요")
        
        elif key == ord('r'):
            # 초기화
            selected_points = []
            current_point_idx = 0
            selection_mode = False
            ema_distances = [None, None, None]
            print("[INFO] 선택한 점 초기화")
    
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
