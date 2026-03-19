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

newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (gray.shape[1], gray.shape[0]), 1, (gray.shape[1], gray.shape[0]))
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]
cv.imshow("Undistorted", dst)
cv.waitKey(0)
cv.destroyAllWindows()


def points2d_to_3d(points_2d: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, mounting_height: float) -> np.ndarray:
    undistorted = cv.undistortPoints(
        points_2d.reshape(-1, 1, 2),
        camera_matrix,
        dist_coeffs,
        P=camera_matrix,
    ).reshape(-1, 2)

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    depth = float(mounting_height)

    u = undistorted[:, 0]
    v = undistorted[:, 1]

    Z = np.full_like(u, depth)

    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy

    return np.stack((X, Y, Z), axis=1)

mtx = np.array([[996.49526547,   0.        , 960.0], [  0, 796.39222603,  540], [0., 0., 1.]])
if len(objpoints) > 0 and len(imgpoints) > 0:
    real_pts_3d = np.asarray(objpoints[0], dtype=np.float64).reshape(-1, 3)
    img_pts_2d = np.asarray(imgpoints[0], dtype=np.float64).reshape(-1, 2)

    assumed_depth = float(np.mean(real_pts_3d[:, 2]))
    est_pts_3d = points2d_to_3d(img_pts_2d, mtx, dist, assumed_depth)

    print("\nIndex | img(u,v) -> real(X,Y,Z) vs est(X,Y,Z) | err_norm")
    for i in range(min(len(real_pts_3d), len(est_pts_3d))):
        real_p = real_pts_3d[i]
        est_p = est_pts_3d[i]
        err = float(np.linalg.norm(est_p - real_p))
        u, v = img_pts_2d[i]
        print(
            f"{i:4d} | ({u:8.2f},{v:8.2f}) -> "
            f"real({real_p[0]:9.2f},{real_p[1]:9.2f},{real_p[2]:9.2f})  "
            f"est({est_p[0]:9.2f},{est_p[1]:9.2f},{est_p[2]:9.2f}) | {err:9.2f}"
        )

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