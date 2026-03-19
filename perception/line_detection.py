import cv2
import numpy as np
import time
import yaml

class Perception:
    def __init__(self, intrinsics_yaml_path: str, debug: bool = False):
        with open(intrinsics_yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        self.camera_matrix = np.asarray(data["INTRINSIC_MATRIX"], dtype=np.float64)
        self.dist_coeffs = np.asarray(data["DISTORTION_COEFFICIENTS"], dtype=np.float64).reshape(-1)
        self.mounting_height = float(data.get("MOUNTING_HEIGHT", 0.0))

        self.lower_A = np.array([174, 100, 100])
        self.upper_A = np.array([179, 255, 255])
        self.lower_B = np.array([0, 100, 175])
        self.upper_B = np.array([10, 255, 255])

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
            self.camera_matrix, self.dist_coeffs, size, 1, size
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


    def get_red_mask(self, image: np.ndarray) -> np.ndarray:
        image = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask_A = cv2.inRange(hsv, self.lower_A, self.upper_A)
        mask_B = cv2.inRange(hsv, self.lower_B, self.upper_B)

        return cv2.bitwise_or(mask_A, mask_B)


    def clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask


    def extract_ridge(self, mask: np.ndarray, height_filter: int) -> np.ndarray:
        dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

        local_max = (
            (dist == cv2.dilate(dist, np.ones((3, 3), np.uint8)))
            & (dist > 5)
        )

        ridge = local_max.astype(np.uint8) * 255

        # remove top region
        ridge[0:height_filter, :] = 0

        return ridge


    def extract_points(self, ridge: np.ndarray) -> np.ndarray:
        ys, xs = np.where(ridge > 0)

        if len(xs) == 0:
            return np.array([]).reshape(0, 1, 2)

        points = np.stack((xs, ys), axis=-1).astype(np.float32)
        return points.reshape(-1, 1, 2)


    def points2d_to_3d(self, points_2d: np.ndarray) -> np.ndarray:
        if len(points_2d) == 0:
            return np.array([]).reshape(0, 3)

        undistorted = cv2.undistortPoints(
            points_2d,
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

        Y = np.full_like(u, depth)
        Z = Y * fy / (v - cy)
        X = Z * (u - cx) / fx

        return np.stack((X, Y, Z), axis=1)

    # Pipeline
    def detect_line(self, image: np.ndarray, height_filter: int):
        start = time.perf_counter()

        image = self.undistort(image)
        t_undistort = time.perf_counter() - start
        total = t_undistort

        mask = self.get_red_mask(image)
        t_red = time.perf_counter() - start - total
        total += t_red

        mask = self.clean_mask(mask)
        t_clean = time.perf_counter() - start - total
        total += t_clean

        ridge = self.extract_ridge(mask, height_filter)
        t_ridge = time.perf_counter() - start - total
        total += t_ridge

        points_2d = self.extract_points(ridge)
        t_points = time.perf_counter() - start - total
        total += t_points

        points_3d = self.points2d_to_3d(points_2d)
        t_3d = time.perf_counter() - start - total
        total += t_3d

        if self.debug:
            print("=======================")
            print(f"Detection took {time.perf_counter() - start:.4f}s")
            print(f"Undistort took {t_undistort:.4f}s")
            print(f"Red mask took {t_red:.4f}s")
            print(f"Clean mask took {t_clean:.4f}s")
            print(f"Extract ridge took {t_ridge:.4f}s")
            print(f"Extract points took {t_points:.4f}s")
            print(f"Points to 3D took {t_3d:.4f}s")
            print("=======================")

        return mask, ridge, points_2d, points_3d