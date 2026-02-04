import cv2
import numpy as np
from perception.line_detection import  line_detection_ridge

def main(file_name : str = "video.mp4"):
    # Reading video
    cap = None

    if file_name:
        cap = cv2.VideoCapture(file_name)
    else:
        cap = cv2.VideoCapture(0)

    ret, frame = cap.read()

    while cap.isOpened():
        height = 600
        width = int(frame.shape[1] * (height / frame.shape[0]))
        
        cv2.imshow("Frame", frame)
        ret, frame = cap.read()
        frame = cv2.resize(frame, (width, height))

        # mask, contours = line_detection(frame)
        mask, ridge, coeffs = line_detection_ridge(frame)
        
        mask = cv2.resize(mask, (width, height))

        mask[ridge > 0] = 0

        if coeffs is not None:
            y_vals = np.arange(height)
            x_vals = np.polyval(coeffs, y_vals).astype(int)
            
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            for i in range(len(y_vals) - 1):
                if 0 <= x_vals[i] < width and 0 <= x_vals[i+1] < width:
                    cv2.line(mask_color, (x_vals[i], y_vals[i]), (x_vals[i+1], y_vals[i+1]), (0, 255, 0), 2)
            
            cv2.imshow("Mask with Lines", mask_color)
        else:
            cv2.imshow("Mask with Lines", mask)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()

if __name__ == "__main__":
    main("20260109_163058.mp4")