import cv2
import numpy as np
import time
import cv2.ximgproc as xip
from config.constants import RED_LOWER, RED_UPPER

def line_detection_contours(image : np):

    start = time.perf_counter()
    # Color segmentation
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(hsv, RED_LOWER, RED_UPPER)

    # Morphological cleanup (CRITICAL)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Thinning (on FILLED regions)
    skeleton = xip.thinning(mask)

    # Extract centerline contours
    contours, _ = cv2.findContours(
        skeleton,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )
    end = time.perf_counter()
    print(f"Line detection took {end - start} seconds")
    return mask, contours 
    

def line_detection_ridge(image : np): 

    start = time.perf_counter()

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(hsv, RED_LOWER, RED_UPPER)

    # 2. Distance transform
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

    # 3. Find ridge (local maxima)
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(dist, kernel)

    ridge = (dist == dilated) & (dist > 0)
    ridge = ridge.astype(np.uint8) * 255
    
    # Remove points in the lower half of the image
    ridge[0:int(ridge.shape[0] / 1.8), :] = 0
    
    # Get ridge points for polynomial fitting
    y_coords, x_coords = np.where(ridge > 0)
    if len(x_coords) > 2:
        coeff = np.polyfit(y_coords, x_coords, 10)
    else:
        coeff = None
    
    end = time.perf_counter()
    print(f"Line detection took {end - start} seconds")
    return mask, ridge, coeff