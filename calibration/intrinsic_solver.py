import numpy as np
import cv2 as cv
import yaml
 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

config = None
with open("calibration_config.yaml", "r") as f:
    config = yaml.safe_load(f)
 
points_x = config["grid_size"]["x"]-1
points_y = config["grid_size"]["y"]-1
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros(((points_x)* points_y, 3), np.float32)

top_left = np.array([
    config["bottom_left_offset"]["x"], 
    config["bottom_left_offset"]["y"] + 9 * config["square_size"], 
    config["bottom_left_offset"]["z"]
    ], np.float32)

# additional offset of x and y to accout for not using outermost squares
for j in range(points_y):
    for i in range(points_x):
        objp[j*points_x+i] = np.array([
            top_left[0] + i*config["square_size"] + config["square_size"], 
            top_left[1] - j*config["square_size"] + config["square_size"], 
            top_left[2]
            ], np.float32)

objp = objp * 25.4 # convert to mm
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
images = [config["image_file"]]
# images = ["/home/miekale/Downloads/test.png"]
 
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    cv.imshow("Gray", gray)
    cv.waitKey()
 
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (config["grid_size"]["x"] - 1, config["grid_size"]["y"] - 1), None)
    print(ret)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (config["grid_size"]["x"], config["grid_size"]["y"]), corners2, ret)
        for i, corner in enumerate(corners2):
            x, y = int(corner[0][0]), int(corner[0][1])
            cv.putText(img, f"{i}", 
                      (x+5, y-5), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

        cv.imshow('img', img)
        cv.waitKey(0)
 
cv.destroyAllWindows()

mtx = np.array([[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints,
    imgpoints,
    (gray.shape[1], gray.shape[0]),
    mtx,
    None,
    flags=cv.CALIB_USE_INTRINSIC_GUESS,
)

newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (gray.shape[1], gray.shape[0]), 1.0, (gray.shape[1], gray.shape[0]))
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

x, y, w, h = roi
cv.imshow("Undistorted", dst)
cv.waitKey(0)
cv.destroyAllWindows()

# undistort using remap with original image dimensions
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (gray.shape[1], gray.shape[0]), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

cv.imshow('calibresult', dst)
cv.waitKey(0)
cv.destroyAllWindows()

if input("Write parmas to file? (y / n)") == "y":
    with open("../config/intrinsics.yaml", "w") as f:
        mtx = mtx.tolist()
        dist = dist.flatten()[:5].tolist()

        print(mtx)
        print(dist)
        
        yaml.dump({
            "INTRINSIC_MATRIX": mtx,
            "DISTORTION_COEFFICIENTS": dist
        }, f, default_flow_style=False)

    print("Parameters written to config/intrinsics.yaml")