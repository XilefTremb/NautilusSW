import cv2
import depthai as dai
import numpy as np


class HostStereoDepth:
    """
    Host-side replacement for DepthAI StereoDepth + depth alignment.

    Output convention:
      - input RGB/left/right frames are NOT rotated.
      - depth is computed in the left mono camera frame.
      - depth is projected into the RGB frame.
      - caller can rotate RGB + aligned depth together afterward.
    """

    def __init__(
        self,
        calib,
        rgb_size=(1280, 960),
        mono_size=(640, 400),
        rgb_socket=dai.CameraBoardSocket.CAM_A,
        left_socket=dai.CameraBoardSocket.CAM_B,
        right_socket=dai.CameraBoardSocket.CAM_C,
        max_depth_mm=10000,
        logger=None,
    ):
        self.calib = calib
        self.rgb_size = tuple(rgb_size)
        self.mono_size = tuple(mono_size)
        self.rgb_socket = rgb_socket
        self.left_socket = left_socket
        self.right_socket = right_socket
        self.max_depth_mm = float(max_depth_mm)
        self.logger = logger

        self._load_calibration()
        self._create_rectification_maps()
        self._create_stereo_matcher()

    # =====================================================
    # CALIBRATION
    # =====================================================
    def _log(self, msg):
        if self.logger is not None:
            self.logger.info(str(msg))

    def _warn(self, msg):
        if self.logger is not None:
            self.logger.warn(str(msg))

    @staticmethod
    def _distortion_as_opencv(dist):
        """DepthAI can return up to 14 distortion coeffs. OpenCV accepts 4,5,8,12,14."""
        d = np.asarray(dist, dtype=np.float64).reshape(-1)
        if d.size >= 14:
            return d[:14]
        if d.size >= 8:
            return d[:8]
        if d.size >= 5:
            return d[:5]
        if d.size >= 4:
            return d[:4]
        return np.zeros(5, dtype=np.float64)

    @staticmethod
    def _extrinsic_rt(extrinsic_4x4):
        E = np.asarray(extrinsic_4x4, dtype=np.float64)
        if E.shape != (4, 4):
            raise ValueError(f"Expected 4x4 extrinsic matrix, got {E.shape}")
        R = E[:3, :3]
        t = E[:3, 3].reshape(3, 1)
        return R, t

    @staticmethod
    def _translation_to_mm(t):
        """
        DepthAI EEPROM translations are commonly in centimeters.
        This heuristic keeps the code usable across calibration exports.
        """
        t = np.asarray(t, dtype=np.float64).reshape(3, 1)
        n = float(np.linalg.norm(t))

        if n <= 0:
            return t

        # OAK-D mono baseline is usually around 7.5 cm.
        # If the norm looks like centimeters, convert cm -> mm.
        if 1.0 <= n <= 30.0:
            return t * 10.0

        # If it looks like meters, convert m -> mm.
        if 0.01 <= n < 1.0:
            return t * 1000.0

        # Otherwise assume mm already.
        return t

    def _load_calibration(self):
        rgb_w, rgb_h = self.rgb_size
        mono_w, mono_h = self.mono_size

        self.K_rgb = np.asarray(
            self.calib.getCameraIntrinsics(self.rgb_socket, rgb_w, rgb_h),
            dtype=np.float64,
        )
        self.K_left = np.asarray(
            self.calib.getCameraIntrinsics(self.left_socket, mono_w, mono_h),
            dtype=np.float64,
        )
        self.K_right = np.asarray(
            self.calib.getCameraIntrinsics(self.right_socket, mono_w, mono_h),
            dtype=np.float64,
        )

        self.D_rgb = self._distortion_as_opencv(
            self.calib.getDistortionCoefficients(self.rgb_socket)
        )
        self.D_left = self._distortion_as_opencv(
            self.calib.getDistortionCoefficients(self.left_socket)
        )
        self.D_right = self._distortion_as_opencv(
            self.calib.getDistortionCoefficients(self.right_socket)
        )

        # Left -> Right transform, used for stereoRectify and baseline.
        E_left_to_right = self.calib.getCameraExtrinsics(self.left_socket, self.right_socket)
        self.R_left_to_right, t_lr_raw = self._extrinsic_rt(E_left_to_right)
        self.t_left_to_right_mm = self._translation_to_mm(t_lr_raw)
        self.baseline_mm = float(np.linalg.norm(self.t_left_to_right_mm))

        # Left -> RGB transform, used to align depth into RGB frame.
        E_left_to_rgb = self.calib.getCameraExtrinsics(self.left_socket, self.rgb_socket)
        self.R_left_to_rgb, t_lrgb_raw = self._extrinsic_rt(E_left_to_rgb)
        self.t_left_to_rgb_mm = self._translation_to_mm(t_lrgb_raw)

        self._log(f"HostStereoDepth baseline_mm={self.baseline_mm:.2f}")
        self._log(f"HostStereoDepth fx_left={self.K_left[0, 0]:.2f}, fx_rgb={self.K_rgb[0, 0]:.2f}")

    def _create_rectification_maps(self):
        mono_w, mono_h = self.mono_size

        # Use OpenCV stereoRectify on host to replace StereoDepth rectification.
        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            cameraMatrix1=self.K_left,
            distCoeffs1=self.D_left,
            cameraMatrix2=self.K_right,
            distCoeffs2=self.D_right,
            imageSize=(mono_w, mono_h),
            R=self.R_left_to_right,
            T=self.t_left_to_right_mm,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )

        self.left_map_x, self.left_map_y = cv2.initUndistortRectifyMap(
            self.K_left,
            self.D_left,
            self.R1,
            self.P1,
            (mono_w, mono_h),
            cv2.CV_32FC1,
        )

        self.right_map_x, self.right_map_y = cv2.initUndistortRectifyMap(
            self.K_right,
            self.D_right,
            self.R2,
            self.P2,
            (mono_w, mono_h),
            cv2.CV_32FC1,
        )

        self.fx_rect = float(self.P1[0, 0])
        self.fy_rect = float(self.P1[1, 1])
        self.cx_rect = float(self.P1[0, 2])
        self.cy_rect = float(self.P1[1, 2])

    def _create_stereo_matcher(self):
        block_size = 5
        num_disparities = 128  # Must be multiple of 16.

        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=50,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    # =====================================================
    # PUBLIC PIPELINE
    # =====================================================
    def rectify(self, left_raw, right_raw):
        left_rect = cv2.remap(
            left_raw,
            self.left_map_x,
            self.left_map_y,
            cv2.INTER_LINEAR,
        )
        right_rect = cv2.remap(
            right_raw,
            self.right_map_x,
            self.right_map_y,
            cv2.INTER_LINEAR,
        )
        return left_rect, right_rect

    def compute_disparity(self, left_rect, right_rect):
        if left_rect.ndim == 3:
            left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_rect

        if right_rect.ndim == 3:
            right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
        else:
            right_gray = right_rect

        return self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

    def disparity_to_depth_left(self, disparity):
        depth_mm = np.zeros_like(disparity, dtype=np.float32)
        valid = disparity > 0.5
        depth_mm[valid] = (self.fx_rect * self.baseline_mm) / disparity[valid]
        depth_mm = np.nan_to_num(depth_mm, nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm[(depth_mm < 0.0) | (depth_mm > self.max_depth_mm)] = 0.0
        return depth_mm

    def align_depth_to_rgb(self, depth_left_mm):
        """
        Project left-rectified depth into RGB image.
        Uses a simple z-buffer: if multiple depth pixels hit the same RGB pixel,
        keep the closest one.
        """
        rgb_w, rgb_h = self.rgb_size
        h, w = depth_left_mm.shape[:2]

        valid_mask = depth_left_mm > 0
        if not np.any(valid_mask):
            return np.zeros((rgb_h, rgb_w), dtype=np.uint16)

        v, u = np.nonzero(valid_mask)
        z = depth_left_mm[v, u].astype(np.float64)

        # 3D points in rectified-left camera coordinates.
        x_rect = (u.astype(np.float64) - self.cx_rect) * z / self.fx_rect
        y_rect = (v.astype(np.float64) - self.cy_rect) * z / self.fy_rect
        pts_rect = np.vstack((x_rect, y_rect, z))

        # Convert rectified-left coordinates back to original-left coordinates.
        pts_left = self.R1.T @ pts_rect

        # Transform original-left coordinates to RGB coordinates.
        pts_rgb = (self.R_left_to_rgb @ pts_left) + self.t_left_to_rgb_mm

        x = pts_rgb[0]
        y = pts_rgb[1]
        z_rgb = pts_rgb[2]

        in_front = z_rgb > 1.0
        if not np.any(in_front):
            return np.zeros((rgb_h, rgb_w), dtype=np.uint16)

        x = x[in_front]
        y = y[in_front]
        z_rgb = z_rgb[in_front]

        u_rgb = np.round(self.K_rgb[0, 0] * x / z_rgb + self.K_rgb[0, 2]).astype(np.int32)
        v_rgb = np.round(self.K_rgb[1, 1] * y / z_rgb + self.K_rgb[1, 2]).astype(np.int32)

        inside = (u_rgb >= 0) & (u_rgb < rgb_w) & (v_rgb >= 0) & (v_rgb < rgb_h)
        if not np.any(inside):
            return np.zeros((rgb_h, rgb_w), dtype=np.uint16)

        u_rgb = u_rgb[inside]
        v_rgb = v_rgb[inside]
        z_rgb = z_rgb[inside]

        depth_rgb = np.zeros((rgb_h, rgb_w), dtype=np.float32)

        # Z-buffer by sorting far -> near so near overwrites far.
        order = np.argsort(z_rgb)[::-1]
        depth_rgb[v_rgb[order], u_rgb[order]] = z_rgb[order]

        # Optional small hole filling. Helps YOLO median depth inside bboxes.
        depth_rgb = self._fill_small_holes(depth_rgb)

        depth_rgb = np.clip(depth_rgb, 0, self.max_depth_mm).astype(np.uint16)
        return depth_rgb

    @staticmethod
    def _fill_small_holes(depth):
        """Very light hole filling. Keeps runtime low."""
        valid = depth > 0
        if not np.any(valid):
            return depth

        # Dilate depth slightly to fill projection holes, but only where empty.
        dilated = cv2.dilate(depth, np.ones((3, 3), np.uint8), iterations=1)
        out = depth.copy()
        out[(out == 0) & (dilated > 0)] = dilated[(out == 0) & (dilated > 0)]
        return out

    def compute_depth_aligned_to_rgb(self, left_raw, right_raw):
        left_rect, right_rect = self.rectify(left_raw, right_raw)
        disparity = self.compute_disparity(left_rect, right_rect)
        depth_left = self.disparity_to_depth_left(disparity)
        depth_rgb = self.align_depth_to_rgb(depth_left)
        return depth_rgb, disparity, depth_left
