import cv2
import numpy as np
import struct
import time
import argparse

from perception.line_detection import Perception
from rpi.ArduinoConnection import ArduinoConnection

CMD_PURE_PURSUIT = 0x00


def _xor_checksum(header: int, length: int, payload: bytes) -> int:
    x = header ^ length
    for b in payload:
        x ^= b
    return x & 0xFF


def _send_pure_pursuit(conn: ArduinoConnection, v: float, w: float, ack_timeout_s: float = 0.1) -> bool:
    header = 0xAA
    payload = bytes([CMD_PURE_PURSUIT]) + struct.pack('<ff', float(v), float(w))
    length = len(payload)
    checksum = _xor_checksum(header, length, payload)
    packet = bytes([header, length]) + payload + bytes([checksum])

    conn.serial.write(packet)

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


def decide_turn_from_points(points: np.ndarray, x_deadband: float) -> str:
    if points is None or len(points) == 0:
        return "straight"

    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    mean_x = float(np.mean(pts[:, 0]))
    if mean_x > x_deadband:
        return "right"
    if mean_x < -x_deadband:
        return "left"
    return "straight"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', default='', help='Video file path. If omitted, uses camera index 0')
    parser.add_argument('--serial', default='/dev/ttyACM0', help='Serial device')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--intrinsics', default='config/intrinsics.yaml')
    parser.add_argument('--v', type=float, default=0.2, help='Forward velocity command')
    parser.add_argument('--w', type=float, default=0.8, help='Turn rate magnitude')
    parser.add_argument('--x-deadband', type=float, default=0.02, help='Deadband on mean X (meters) for straight')
    parser.add_argument('--show', action='store_true', help='Show debug windows')
    args = parser.parse_args()

    perception = Perception(args.intrinsics, debug=False)

    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError('Failed to open video source')

    arduino = ArduinoConnection(args.serial, baud=args.baud)

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret or frame is None:
                break

            height = 600
            width = int(frame.shape[1] * (height / frame.shape[0]))
            frame = cv2.resize(frame, (width, height))

            mask, ridge, points = perception.line_detection_ridge(frame, 0)
            decision = decide_turn_from_points(points, args.x_deadband)

            if decision == 'left':
                v, w = args.v, +abs(args.w)
            elif decision == 'right':
                v, w = args.v, -abs(args.w)
            else:
                v, w = args.v, 0.0

            _send_pure_pursuit(arduino, v, w)

            if args.show:
                cv2.imshow('Frame', frame)
                cv2.imshow('Mask', mask)
                cv2.imshow('Ridge', ridge)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break

    finally:
        cap.release()
        arduino.closeConnection()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())