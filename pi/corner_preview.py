# corner_preview.py - PA03, simple version
# Built from the provided references:
#   - preview loop + BEV: CameraCalibration README
#   - orange mask, polygon solver, pixel->cm: course Colab (parking.ipynb)
#   - Bluetooth client: EV3-RFCOMM bluetooth_client.py
# No checkerboard / lens undistortion used (camera_calib.npz not needed).
# EV3 side: run BTServer.java + BTConnection.java from EV3-RFCOMM unchanged
# (BTServer prints each received line on the LCD).
#
# Keys: q = quit, b = connect Bluetooth, s = send corners to EV3

import json
import socket
import time
from itertools import combinations

import cv2
import numpy as np
from picamera2 import Picamera2

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
EV3_MAC_ADDRESS = "00:16:53:5F:11:CE"  # CHANGE THIS to your EV3's MAC address
EV3_PORT = 1
AXLE_TO_BUMPER_CM = 0.0  # distance from wheel axle (leJOS origin) to bumper line; 0 = measure from bumper

with open("camera_calib.json", "r") as f:  # made by calibrate_bev.py (same folder)
    calib = json.load(f)
M = np.array(calib["M"], dtype=np.float32)
BEV_W = calib["bev_width"]
BEV_H = calib["bev_height"]


# ---------------- Bluetooth (from bluetooth_client.py) ----------------
class BT:
    def __init__(self):
        self.bt_socket = None
        self.connected = False

    def connect_bluetooth(self):
        try:
            print(f"Connecting to EV3 at {EV3_MAC_ADDRESS}...")
            self.bt_socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self.bt_socket.settimeout(10)
            self.bt_socket.connect((EV3_MAC_ADDRESS, EV3_PORT))
            self.bt_socket.settimeout(None)
            self.connected = True
            print("Connected to EV3 successfully!")
        except OSError as e:
            print(f"Bluetooth connection failed: {e}")
            self.connected = False

    def send_message(self, message):
        if not self.connected:
            print("Not connected, press 'b' first")
            return
        try:
            self.bt_socket.sendall((message + "\r\n").encode())
        except OSError as e:
            print(f"Send failed: {e}")
            self.connected = False

    def disconnect_bluetooth(self):
        if self.bt_socket:
            self.bt_socket.close()
        self.connected = False


# ---------------- Colab section 3: orange mask ----------------
def detect_orange(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


# ---------------- Colab section 4.5: polygon solver ----------------
def calculate_quad_area(quad_pts):
    cx, cy = np.mean(quad_pts, axis=0)
    angles = np.arctan2(quad_pts[:, 1] - cy, quad_pts[:, 0] - cx)
    q = quad_pts[np.argsort(angles)]
    x, y = q[:, 0], q[:, 1]
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def find_four_corners(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    u_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(u_contour) < 300:  # ignore tiny orange specks
        return None
    perimeter = cv2.arcLength(u_contour, False)
    approx = cv2.approxPolyDP(u_contour, 0.03 * perimeter, closed=False)
    vertices = approx.reshape(-1, 2)

    # best 4-combination of hull points = 4 outermost corners
    candidate_points = cv2.convexHull(vertices).reshape(-1, 2)
    if len(candidate_points) < 4:
        return None
    max_area, final_4_corners = 0, None
    for quad in combinations(candidate_points, 4):
        quad_array = np.array(quad)
        area = calculate_quad_area(quad_array)
        if area > max_area:
            max_area, final_4_corners = area, quad_array
    cx, cy = np.mean(final_4_corners, axis=0)
    angles = np.arctan2(final_4_corners[:, 1] - cy, final_4_corners[:, 0] - cx)
    return final_4_corners[np.argsort(angles)]


# ---------------- Colab section 5: BEV pixel -> cm ----------------
def bev_pixel_to_robot_cm(bev_x, bev_y, calib_data):
    cm_per_pix = calib_data["cm_per_pixel"]
    x_cm = calib_data["min_x_cm"] + (bev_x * cm_per_pix)  # right is positive
    y_cm = calib_data["max_y_cm"] - (bev_y * cm_per_pix)  # forward is positive
    return x_cm, y_cm


def to_lejos(x_cm, y_cm):
    # leJOS: X forward, Y left (Figure 1 of the handout)
    X = y_cm + AXLE_TO_BUMPER_CM
    Y = -x_cm
    return round(X, 1), round(Y, 1)


# ---------------- Preview loop (CameraCalibration README) ----------------
bt = BT()
camera = Picamera2()
try:
    config = camera.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    camera.configure(config)
    camera.start()
    time.sleep(2)  # Camera warm-up
    while True:
        frame = camera.capture_array()
        bev_frame = cv2.warpPerspective(frame, M, (BEV_W, BEV_H))

        mask = detect_orange(bev_frame)
        four_corners = find_four_corners(mask)

        lejos_corners = []
        if four_corners is not None:
            for idx, (x, y) in enumerate(four_corners):
                X, Y = to_lejos(*bev_pixel_to_robot_cm(x, y, calib))
                lejos_corners.append((X, Y))
                cv2.circle(bev_frame, (int(x), int(y)), 8, (0, 255, 0), -1)
                cv2.putText(bev_frame, f"P{idx+1} X={X} Y={Y}", (int(x) + 10, int(y) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow("Camera Preview", frame)
        cv2.imshow("BEV", bev_frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b'):
            bt.connect_bluetooth()
        elif key == ord('s'):
            if not lejos_corners:
                print("No corners detected")
            for idx, (X, Y) in enumerate(lejos_corners):
                msg = f"P{idx+1} {X},{Y}"  # short: EV3 LCD is 18 chars wide
                bt.send_message(msg)
                print("Sent:", msg)
finally:
    bt.disconnect_bluetooth()
    camera.stop()
    cv2.destroyAllWindows()
