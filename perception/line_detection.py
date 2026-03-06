import cv2
import numpy as np
import time
import cv2.ximgproc as xip
import yaml
from config.constants import RED_LOWER, RED_UPPER

class Perception:
    def __init__(self, intrinsics_yaml_path: str, debug: bool = False):
        with open(intrinsics_yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        self.camera_matrix = np.asarray(data["INTRINSIC_MATRIX"], dtype=np.float64)
        self.dist_coeffs = np.asarray(data["DISTORTION_COEFFICIENTS"], dtype=np.float64).reshape(-1)
        self.mounting_height = float(data.get("MOUNTING_HEIGHT", 0.0))

        self.debug = debug

        self.undistortion_map = None
        self._undistort_size = None
        self._new_camera_matrix = None

    def _ensure_undistortion_map(self, image: np.ndarray) -> None:
        h, w = image.shape[:2]
        size = (w, h)
        if self.undistortion_map is not None and self._undistort_size == size:
            return

        self._new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.dist_coeffs,
            size,
            1,
            size,
        )

        map1, map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            R=None,
            newCameraMatrix=self._new_camera_matrix,
            size=size,
            m1type=cv2.CV_32FC1,
        )

        self.undistortion_map = (map1, map2)
        self._undistort_size = size

    def undistort(self, image: np.ndarray) -> np.ndarray:
        self._ensure_undistortion_map(image)
        map1, map2 = self.undistortion_map
        return cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)

    def line_detection_ridge(self, image: np.ndarray, height_filter: int):

        start = time.perf_counter()

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        mask = cv2.inRange(hsv, RED_LOWER, RED_UPPER)

        dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(dist, kernel)

        ridge = (dist == dilated) & (dist > 0)
        ridge = ridge.astype(np.uint8) * 255
        
        points = cv2.findNonZero(ridge)
        points = self.points2d_to_3d(points.astype(np.float32))

        end = time.perf_counter()
        if self.debug:
            print(f"Line detection took {end - start} seconds")
        return mask, ridge, points

    def points2d_to_3d(self, points_2d: np.ndarray) -> np.ndarray:
        undistorted = cv2.undistortPoints(
            points_2d.reshape(-1, 1, 2),
            self.camera_matrix,
            self.dist_coeffs,
            P=self.camera_matrix
        ).reshape(-1, 2)

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        depth = self.mounting_height

        u = undistorted[:, 0]
        v = undistorted[:, 1]

        X = depth * (u - cx) / fx
        Y = depth * (v - cy) / fy
        Z = np.full_like(X, depth)

        return np.stack((X, Y, Z), axis=1)