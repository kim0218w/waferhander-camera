import cv2
import numpy as np

# ===== 사용자 설정 =====
SOURCE = 0                 # 0=웹캠
A4_LONG_M = 0.297          # A4 긴변 (세로) 297mm = 0.297m
A4_SHORT_M = 0.210         # A4 짧은변 (가로) 210mm = 0.210m
KNOWN_DISTANCE_M = 1.50    # 's' 로 캘리브레이트할 때 실제 거리(미터)
FOCAL_PX = None            # 한 번 추정되면 여기 저장됨(프로그램 켤 때 직접 숫자 넣어도 됨)

FONT = cv2.FONT_HERSHEY_SIMPLEX

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
    if quad is None: return None
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
    # f_px = (h_px * D_m) / H_m
    return (h_px * D_m) / H_m

def distance_from_px(h_px, H_m, f_px):
    # D = (H * f) / h
    if h_px <= 0: return None
    return (H_m * f_px) / h_px

cap = cv2.VideoCapture(SOURCE)
if not cap.isOpened():
    raise RuntimeError("카메라 열기 실패")

ema_d = None
alpha = 0.2  # 지터 완화용 EMA

print("[INFO] 's' 를 눌러 현재 프레임과 KNOWN_DISTANCE_M를 이용해 f_px(초점거리)를 추정할 수 있습니다.")
print("[INFO] 종이 테두리에 두껍게 선을 그으면(마커 느낌) 검출이 안정적입니다.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    quad = largest_quad(frame)
    if quad is not None:
        # 변 길이 측정
        _, long_px, short_px = edge_lengths(quad)

        # 화면에 외곽선 그리기
        cv2.polylines(frame, [quad.astype(int)], True, (0,255,0), 2)

        # A4는 긴변: 0.297m, 짧은변: 0.210m
        # 카메라와 거의 평행하면 긴변/짧은변 중 픽셀에서도 더 긴 쪽이 실제 긴변에 해당
        # (기울어지면 약간 섞이지만 긴변 기준이 대체로 더 안정적)
        H_real_m = A4_LONG_M if long_px >= short_px else A4_SHORT_M
        h_px = max(long_px, short_px)

        if FOCAL_PX is not None:
            D = distance_from_px(h_px, H_real_m, FOCAL_PX)
            if D is not None:
                ema_d = D if ema_d is None else (alpha*D + (1-alpha)*ema_d)
                cv2.putText(frame, f"Distance: {ema_d:.2f} m", (20,40), FONT, 1.0, (0,255,0), 2)
        else:
            cv2.putText(frame, "Press 's' to calibrate f(px)", (20,40), FONT, 0.8, (0,255,255), 2)

        # 디버그용 길이 표시
        cv2.putText(frame, f"h_px(longest edge): {h_px:.1f}", (20,70), FONT, 0.7, (0,255,0), 2)

    else:
        cv2.putText(frame, "A4 not found", (20,40), FONT, 0.8, (0,0,255), 2)

    cv2.imshow("A4 Distance", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        # 현재 프레임에서 h_px 가져와 f_px 추정
        if quad is None:
            print("[WARN] A4가 화면에 보일 때 눌러주세요.")
        else:
            _, long_px, short_px = edge_lengths(quad)
            H_real_m = A4_LONG_M if long_px >= short_px else A4_SHORT_M
            h_px = max(long_px, short_px)
            FOCAL_PX = estimate_focal_px(h_px, H_real_m, KNOWN_DISTANCE_M)
            ema_d = None
            print(f"[INFO] f_px 추정 = {FOCAL_PX:.2f}  (KNOWN_DISTANCE_M={KNOWN_DISTANCE_M} m, h_px={h_px:.1f} px, H={H_real_m} m)")

cap.release()
cv2.destroyAllWindows()
