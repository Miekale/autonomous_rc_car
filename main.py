import cv2
import numpy as np
import struct
import time
from time import sleep
import argparse

from pandas.core.computation.ops import d

from perception.line_detection import Perception
from rpi.ArduinoConnection import ArduinoConnection

CMD_PURE_PURSUIT = 0x00

def _render_xy_plot(
    points_3d: np.ndarray,
    xlim=(-500.0, 500.0),
    ylim=(0.0, 1000.0),
    size=(600, 600),
    x_index: int = 0,
    y_index: int = 1,
) -> np.ndarray:
    h, w = size
    img = np.zeros((h, w, 3), dtype=np.uint8)

    if points_3d is None:
        return img

    pts = np.asarray(points_3d, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return img

    if x_index < 0 or x_index > 2 or y_index < 0 or y_index > 2:
        return img

    x = pts[:, x_index]
    y = pts[:, y_index]

    xmin, xmax = float(xlim[0]), float(xlim[1])
    ymin, ymax = float(ylim[0]), float(ylim[1])
    if xmax == xmin or ymax == ymin:
        return img

    x_norm = (x - xmin) / (xmax - xmin)
    y_norm = (y - ymin) / (ymax - ymin)

    in_bounds = (x_norm >= 0.0) & (x_norm <= 1.0) & (y_norm >= 0.0) & (y_norm <= 1.0)
    if not np.any(in_bounds):
        return img

    x_px = (x_norm[in_bounds] * (w - 1)).astype(np.int32)
    y_px = ((1.0 - y_norm[in_bounds]) * (h - 1)).astype(np.int32)

    for xi, yi in zip(x_px, y_px):
        cv2.circle(img, (int(xi), int(yi)), 1, (0, 255, 0), -1)

    cv2.line(img, (0, h - 1), (w - 1, h - 1), (60, 60, 60), 1)
    cv2.line(img, (0, 0), (0, h - 1), (60, 60, 60), 1)

    return img

def _send_pure_pursuit(conn: ArduinoConnection, v: float, w: float, ack_timeout_s: float = 0.1) -> bool:
    conn.write_data(CMD_PURE_PURSUIT, [v, w])
    deadline = time.time() + ack_timeout_s
    while time.time() < deadline:
        if conn.serial.in_waiting > 0:
            resp = conn.serial.read(1)
            if resp == b'\x06':
                return True
            if resp == b'\x15':
                return False
        time.sleep(0.001)
    return False


def decide_turn_from_points(points: np.ndarray, x_deadband: float, frame_width) -> str:    
    if points is None or len(points) == 0:
        return "straight"

    pts = points.reshape(-1, 2)
    mean_x = np.mean(pts[:, 0]) - (frame_width / 2)

    # pts = np.asarray(points, dtype=np.float64)
    # mean_x = np.mean(pts[:,0]) - frame_width / 2
    if mean_x > x_deadband:
        print(f"{mean_x}: right")
        return "right"
    if mean_x < -x_deadband:
        print(f"{mean_x}: left")
        return "left"
    print(f"{mean_x}: straight")
    return "straight"

def shit(ser):
    try:
        while True:
            line = ser.readline()  # read one line (until '\n')
            if line:
                # Decode bytes to string, strip newline characters
                print(line.decode('utf-8', errors='replace').strip())
            # Optional: tiny delay to avoid CPU hog
            time.sleep(0.01)
    except KeyboardInterrupt:
        ser.close()
        print("Serial monitor stopped.")


def walk(arduino):
    v = 1.0
    w = 1.0
    print("milk")
    while True:
        print("new")
        _send_pure_pursuit(arduino, v, w)
        sleep(5)
        _send_pure_pursuit(arduino, v, -w)
        sleep(5)
        _send_pure_pursuit(arduino, 0, 0)
    
        sleep(2)

def _make_debug_view(frame, mask, ridge, points_3d, w, h):
    frame = cv2.resize(frame, (w//2, h//2), interpolation=cv2.INTER_NEAREST)
    w, h = frame.shape[1], frame.shape[0]

    # Normalize single-channel images to BGR for stacking
    mask_bgr  = cv2.cvtColor(mask,  cv2.COLOR_GRAY2BGR)
    ridge_bgr = cv2.cvtColor(ridge, cv2.COLOR_GRAY2BGR)
    plot_bgr  = _render_xy_plot(points_3d, xlim=(-1000, 1000), ylim=(0, 2000), x_index=0, y_index=2)

    # Resize all panels to match frame size
    def fit(img):
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)

    top    = np.hstack([frame,         fit(mask_bgr)])
    bottom = np.hstack([fit(ridge_bgr), fit(plot_bgr)])
    grid   = np.vstack([top, bottom])

    # Labels
    labels = ["Frame", "Mask", "Ridge", "3D Points (XZ)"]
    positions = [(10, 30), (w + 10, 30), (10, h + 30), (w + 10, h + 30)]
    for label, pos in zip(labels, positions):
        cv2.putText(grid, label, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2, cv2.LINE_AA)

    return grid


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', default='', help='Video file path. If omitted, uses camera index 0')
    parser.add_argument('--serial', default='/dev/ttyACM0', help='Serial device')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--intrinsics', default='config/intrinsics.yaml')
    parser.add_argument('--v', type=float, default=0.2, help='Forward velocity command')
    parser.add_argument('--w', type=float, default=0.8, help='Turn rate magnitude')
    parser.add_argument('--x-deadband', type=float, default=75, help='Deadband on mean X (pixels) for straight')
    parser.add_argument('--show', action='store_true', help='Show debug windows')
    parser.add_argument('--demo_open_claw', default = False, help="Just for demo")
    parser.add_argument('--demo_close_claw', default = False, help="Just for demo")

    args = parser.parse_args()

    if args.demo_close_claw:
        arduino = ArduinoConnection(args.serial, baud=args.baud)
        arduino.write_data(0x02, [])
        arduino.write_data(0x00, [0.0, 0.0])
        #shit(arduino.serial)
        return
    if args.demo_open_claw:
        arduino = ArduinoConnection(args.serial, baud=args.baud)
        arduino.write_data(0x01, [])
        arduino.write_data(0x00, [0.0, 0.0])
        #shit(arduino.serial)
        # walk(arduino)
        return

    perception = Perception(args.intrinsics, debug=True)
    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError('Failed to open video source')

    # arduino = ArduinoConnection(args.serial, baud=args.baud)

    try:
        while cap.isOpened():

            ret, frame = cap.read()
            if not ret or frame is None:
                break

            # cutting off the top 60%
            cut_off = int(frame.shape[0] * .6)

            mask, ridge, points, points_3d = perception.detect_line(frame, cut_off) 

            decision = decide_turn_from_points(points, args.x_deadband, frame.shape[1])

            if decision == 'left':
                v, w = args.v, +abs(args.w)
            elif decision == 'right':
                v, w = args.v, -abs(args.w)
            else:
                v, w = args.v, 0.0

            # _send_pure_pursuit(arduino, v, w)

            if args.show:
                # In your loop:
                debug_view = _make_debug_view(frame, mask, ridge, points_3d, int(frame.shape[1]/ 1.5), int(frame.shape[0] / 1.5))
                cv2.imshow('Debug', debug_view)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break

    finally:
        cap.release()
        
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())