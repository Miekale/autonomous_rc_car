import cv2
import numpy as np


def main(file_name : str = "video.mp4"):
    cap = None
    if file_name:
        cap = cv2.VideoCapture(file_name)
    else:
        cap = cv2.VideoCapture(0)

    ret, frame = cap.read()

    while cap.isOpened():
        cv2.imshow("Frame", frame)
        ret, frame = cap.read()
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()


if __name__ == "__main__":
    main("20260109_163058.mp4")