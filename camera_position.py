import cv2
import numpy as np
import time

# ===== 사용자 설정 =====
SOURCE = 0                 # 0=웹캠
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
    
    cap = cv2.VideoCapture(SOURCE)
    if not cap.isOpened():
        raise RuntimeError("카메라 열기 실패")

    # EMA 변수들
    ema_d = None
    ema_x = None
    ema_y = None
    ema_rot = None
    alpha = 0.2  # 지터 완화용 EMA

    # 평면 좌표 표시 모드
    show_plane_coords = False

    cv2.namedWindow("A4 Position Tracker")
    cv2.setMouseCallback("A4 Position Tracker", mouse_callback)

    print("\n" + "="*70)
    print(" A4 3D Position and Plane Coordinate Tracker")
    print("="*70)
    print("\nControls:")
    print("  's' - Calibrate focal length (camera distance)")
    print("  'c' - Start plane calibration (3-point setup)")
    print("  'p' - Toggle plane coordinate display")
    print("  'r' - Reset plane calibration")
    print("  'q' - Quit")
    print("\nPlane Calibration Steps:")
    print("  1. Press 'c'")
    print("  2. Click on Origin point (bottom-left corner of A4)")
    print("  3. Click on X-axis point (bottom-right corner)")
    print("  4. Click on Y-axis point (top-left corner)")
    print("="*70 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

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
            _, long_px, short_px = edge_lengths(quad)

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
                    
                    # EMA 적용 (지터 완화)
                    ema_d = D if ema_d is None else (alpha*D + (1-alpha)*ema_d)
                    ema_x = x_m if ema_x is None else (alpha*x_m + (1-alpha)*ema_x)
                    ema_y = y_m if ema_y is None else (alpha*y_m + (1-alpha)*ema_y)
                    if rotation is not None:
                        ema_rot = rotation if ema_rot is None else (alpha*rotation + (1-alpha)*ema_rot)
                    
                    # === 카메라 좌표계 표시 ===
                    y_offset = 60 if calibration_mode else 30
                    cv2.putText(frame, "=== Camera Coordinates ===", 
                               (20, y_offset), FONT, 0.6, (255,255,255), 2)
                    cv2.putText(frame, f"Z(Distance): {ema_d:.3f} m", 
                               (20, y_offset+30), FONT, 0.7, (0,255,0), 2)
                    cv2.putText(frame, f"X(Horizontal): {ema_x:+.3f} m", 
                               (20, y_offset+60), FONT, 0.7, (0,255,0), 2)
                    cv2.putText(frame, f"Y(Vertical): {ema_y:+.3f} m", 
                               (20, y_offset+90), FONT, 0.7, (0,255,0), 2)
                    if ema_rot is not None:
                        cv2.putText(frame, f"Rotation: {ema_rot:.1f} deg", 
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

        cv2.imshow("A4 Position Tracker", frame)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            # 초점거리 캘리브레이션
            if quad is None:
                print("[WARN] A4가 화면에 보일 때 눌러주세요.")
            else:
                _, long_px, short_px = edge_lengths(quad)
                H_real_m = A4_LONG_M if long_px >= short_px else A4_SHORT_M
                h_px = max(long_px, short_px)
                FOCAL_PX = estimate_focal_px(h_px, H_real_m, KNOWN_DISTANCE_M)
                ema_d = None
                ema_x = None
                ema_y = None
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

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

