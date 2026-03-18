import cv2
import numpy as np
import struct
import time
from time import sleep
import argparse

from perception.line_detection import Perception
from rpi.ArduinoConnection import ArduinoConnection

CMD_PURE_PURSUIT = 0x00

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

    perception = Perception(args.intrinsics, debug=False)
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

            height = 600
            width = int(frame.shape[1] * (height / frame.shape[0]))
    
            frame = cv2.resize(frame, (width, height))
            
            # cutting off the top 60%
            cut_off = int(height * .6)

            mask, ridge, points = perception.line_detection_ridge(frame, cut_off) 

            decision = decide_turn_from_points(points, args.x_deadband, width)

            if decision == 'left':
                v, w = args.v, +abs(args.w)
            elif decision == 'right':
                v, w = args.v, -abs(args.w)
            else:
                v, w = args.v, 0.0

            # _send_pure_pursuit(arduino, v, w)

            if args.show:
                cv2.imshow('Frame', frame)
                cv2.imshow('Mask', mask)
                cv2.imshow('Ridge', ridge)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break

    finally:
        cap.release()
        
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())