import cv2
import numpy as np
import json
from picamera2 import Picamera2

# --- 1. PHYSICAL MEASUREMENTS OF CALIBRATION MAT ---
REAL_MAT_W_CM = 33.3       # Width of calibration mat (cm)
REAL_MAT_H_CM = 63.0        # Height of calibration mat (cm)
MAT_DIST_FROM_BUMPER = 33.0 # Distance from front bumper to bottom edge of mat (cm)

# Desired resolution scale (e.g., 10 pixels per cm -> 1 pixel = 1 mm on the ground)
PIXELS_PER_CM = 5.0

# Capture initial frame to get camera resolution
# (no checkerboard calibration: camera_calib.npz not used)
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
frame = picam2.capture_array()
picam2.stop()



RAW_H, RAW_W = frame.shape[:2]
img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
display_img = img.copy()
src_pts = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(src_pts) < 4:
        src_pts.append([x, y])
        cv2.circle(display_img, (x, y), 5, (0, 0, 255), -1)
        labels = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
        cv2.putText(display_img, labels[len(src_pts)-1], (x + 10, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow("Calibrate Full Uncropped BEV", display_img)

print("INSTRUCTIONS: Click the 4 corners of your 40x40 cm mat in order:")
print("1. Top-Left  2. Top-Right  3. Bottom-Right  4. Bottom-Left")

cv2.imshow("Calibrate Full Uncropped BEV", display_img)
cv2.setMouseCallback("Calibrate Full Uncropped BEV", mouse_callback)
cv2.waitKey(0)
cv2.destroyAllWindows()

if len(src_pts) == 4:
    # 2. Initial Temporary Mapping based purely on the Mat's known Metric dimensions
    src = np.float32(src_pts)
    
    # Place mat temporarily at an arbitrary metric ground location
    # X_mat: [-20cm to +20cm], Y_mat: [10cm to 50cm] relative to bumper
    dst_temp_metric = np.float32([
        [-REAL_MAT_W_CM / 2.0, MAT_DIST_FROM_BUMPER + REAL_MAT_H_CM], # Top-Left
        [ REAL_MAT_W_CM / 2.0, MAT_DIST_FROM_BUMPER + REAL_MAT_H_CM], # Top-Right
        [ REAL_MAT_W_CM / 2.0, MAT_DIST_FROM_BUMPER],                # Bottom-Right
        [-REAL_MAT_W_CM / 2.0, MAT_DIST_FROM_BUMPER]                 # Bottom-Left
    ])
    
    # Initial Perspective Matrix from Raw Pixels -> Metric Ground Coordinates (cm)
    M_metric = cv2.getPerspectiveTransform(src, dst_temp_metric)

    # 3. Project the 4 Extreme Outer Corners of the RAW CAMERA FRAME into Metric Ground Space
    raw_corners = np.float32([
        [[0, 0]], 
        [[RAW_W, 0]], 
        [[RAW_W, RAW_H]], 
        [[0, RAW_H]]
    ])
    
    # Warp raw corners to find the ground plane bounding box of the FULL camera view
    warped_corners = cv2.perspectiveTransform(raw_corners, M_metric)

    x_coords = warped_corners[:, 0, 0]
    y_coords = warped_corners[:, 0, 1]

    # Find maximum metric extent visible by the raw camera
    min_x_cm, max_x_cm = np.min(x_coords), np.max(x_coords)
    min_y_cm, max_y_cm = np.min(y_coords), np.max(y_coords)

    # 4. Compute Final Canvas Pixel Dimensions (Uncropped Canvas)
    BEV_W = int((max_x_cm - min_x_cm) * PIXELS_PER_CM)
    BEV_H = int((max_y_cm - min_y_cm) * PIXELS_PER_CM)

    # 5. Build Offset Matrix to Shift and Scale Ground Coordinates into Pixel Canvas [0, BEV_W] x [0, BEV_H]
    # In BEV pixel space: X increases right, Y increases down (so max_y_cm corresponds to pixel row 0)
    S = np.array([
        [PIXELS_PER_CM, 0,              -min_x_cm * PIXELS_PER_CM],
        [0,             -PIXELS_PER_CM,  max_y_cm * PIXELS_PER_CM],
        [0,             0,               1]
    ], dtype=np.float32)

    # Final Master Perspective Matrix (Raw Camera Pixels -> Full Uncropped BEV Canvas Pixels)
    M_final = S @ M_metric
    M_final_inv = np.linalg.inv(M_final)

    # Save complete calibration data
    calib_data = {
        "M": M_final.tolist(),
        "M_inv": M_final_inv.tolist(),
        "bev_width": BEV_W,
        "bev_height": BEV_H,
        "cm_per_pixel": 1.0 / PIXELS_PER_CM,
        "min_x_cm": float(min_x_cm),
        "max_x_cm": float(max_x_cm),
        "min_y_cm": float(min_y_cm),
        "max_y_cm": float(max_y_cm),
        "target_offset_from_bumper_cm": MAT_DIST_FROM_BUMPER
    }

    with open("camera_calib.json", "w") as f:
        json.dump(calib_data, f, indent=4)

    print(f"\n[SUCCESS] Saved Full Uncropped BEV Calibration!")
    print(f"BEV Canvas Size: {BEV_W} x {BEV_H} pixels")
    print(f"Ground Field of View Covered: X [{min_x_cm:.1f} to {max_x_cm:.1f}] cm, Y [{min_y_cm:.1f} to {max_y_cm:.1f}] cm")