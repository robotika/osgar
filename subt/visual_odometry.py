import cv2
import functools
import logging
import numpy as np
import sys

import osgar.bus
import osgar.node
import osgar.lib.quaternion

g_logger = logging.getLogger(sys.modules[__name__].__spec__.name)

def no_change_rigid_transform():
    """Create a 4x4 matrix representing no change in orientation or position."""
    return np.eye(4)


def as_rigid_transform(xyz=None, rpy=None, rot=None):
    """Create a rigid transformation matrix representing motion.

    xyz -- X, y and z translation.
    rpy -- Roll, pitch and yaw.
    rot -- 3x3 rotation matrix.

    Only one of `rot` and `rpy` can be set.
    """
    assert (rpy is None or rot is None)
    rt = no_change_rigid_transform()
    if xyz is not None:
        rt[:3, 3] = np.asarray(xyz)
    if rpy is not None:
        rt[:3, :3] = euler_angles_to_rotation_matrix(*rpy)
    if rot is not None:
        rt[:3, :3] = np.asarray(rot)
    return rt


def inverse_rotation(r):
    """Invert a 3x3 rotation matrix.

    https://en.wikipedia.org/wiki/Rotation_matrix#Multiplication
    """
    assert (r.shape == (3, 3))
    return r.T


def inverse_rigid_transform(transform):
    """Invert a 4x4 rigid transformation matrix.

    A rigid transformation matrix consists of a 3x3 rotation submatrix (top
    left) 3x1 translation submatrix and a padding 'to make it work':
        |R t|
        |0 1]

    http://negativeprobability.blogspot.com/2011/11/affine-transformations-and-their.html
    """
    transform = np.asarray(transform)
    assert (transform.shape == (4, 4))
    rot = transform[:3, :3]
    rot_inv = inverse_rotation(rot)
    translation = transform[:3, 3]
    translation_inv = -rot_inv @ translation
    inverse = as_rigid_transform(xyz=translation_inv, rot=rot_inv)
    return inverse


def rotation_matrix_to_euler_angles(rot):
    """Invert a 3x3 rotation matrix to roll, pitch and yaw.

    https://stackoverflow.com/questions/43364900/rotation-matrix-to-euler-angles-with-opencv
    """
    rot = np.asarray(rot)
    assert (rot.shape == (3, 3))
    cosine_for_pitch = np.sqrt(rot[0][0]**2 + rot[1][0]**2)
    is_singular = cosine_for_pitch < 10**-6
    if not is_singular:
        yaw = np.arctan2(rot[1][0], rot[0][0])
        pitch = np.arctan2(-rot[2][0], cosine_for_pitch)
        roll = np.arctan2(rot[2][1], rot[2][2])
    else:
        yaw = np.arctan2(-rot[1][2], rot[1][1])
        pitch = np.arctan2(-rot[2][0], cosine_for_pitch)
        roll = 0
    return roll, pitch, yaw


def euler_angles_to_rotation_matrix(roll, pitch, yaw):
    """Convert roll, pitch and yaw into a 3x3 rotation matrix.

    http://planning.cs.uiuc.edu/node102.html
    """
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    roll_m = np.array([[1., 0., 0.], [0., cos_roll, -sin_roll],
                       [0., sin_roll, cos_roll]])
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    pitch_m = np.array([[cos_pitch, 0, sin_pitch], [0, 1, 0],
                        [-sin_pitch, 0, cos_pitch]])
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    yaw_m = np.array([[cos_yaw, -sin_yaw, 0], [sin_yaw, cos_yaw, 0],
                      [0, 0, 1]])

    return yaw_m @ pitch_m @ roll_m


def near_rotation_matrix(m):
    """Find a rotation matrix as close as possible to the input matrix.

    For some definition of "as close as".
    http://users.cecs.anu.edu.au/~hongdong/rotationaveraging.pdf

    This is useful to get rid of errors accumulated in a rotation matrix
    by repeatedly applying matrix multiplication.
    """
    u, s, v = np.linalg.svd(m)
    return u @ v


class VisualOdometryParams:
    def __init__(self,
                 camera_focal_length,
                 camera_center,
                 min_valid_depth=0.01,
                 max_valid_depth=10.,
                 max_corners=1000,
                 corner_quality_level=0.005,
                 corner_distance=5,
                 subpix_win=1,
                 subpix_iters=8,
                 corner_matching_window_size=25,
                 ransac_reprojection_error=0.4,
                 ransac_confidence=0.9999,
                 ransac_pnp_method=cv2.SOLVEPNP_UPNP,
                 ransac_attempts=1,
                 ransac_attempt_reprojection_increase_factor=2.,
                 min_inliers=128,
                 max_rotation_since_key_frame=np.radians(60),
                 max_translation_since_key_frame=2.,
                 max_avg_static_scene_intensity_change=5):
        """Encapsulate parameters needed for visual odometry.

        camera_focal_length -- Focal length of the camera, in pixels.
                               Assuming fx == fy.
        camera_center -- Principal point of the camera, in pixels.
        min_valid_depth -- Minimum depth still considered to be valid, in
                           meters.
        max_valid_depth -- Maximum depth still considered to be valid, in
                           meters.
        max_corners -- Maximum number of feature points detected in the image.
                       Higher number leads to slower and more precise odometry.
        corner_quality_level -- Quality threshold for detected feature points.
        corner_distance -- Minimum required distance between two feature points.
        subpix_win -- Window size for subpixel feature point search.
        subpix_iters -- Number of iterations when searching for subpixel feature
                        points.
        corner_matching_window_size -- Size of the window when matching feature
                                       points between different images.
        ransac_reprojection_error -- Maximum acceptable projection error when
                                     estimating camera movement. Higher value
                                     leads to higher number of less precise
                                     inliers. It is better to keep the value low
                                     while high enough to get sufficiently many
                                     intliers.
        ransac_confidence -- Probability that the estimation of movement
                             produced a useful result.
        ransac_pnp_method -- Which PnP method to use. SOLVE_UPNP and SOLVE_EPNP
                             appear to work well, SOLVE_ITERATIVE doesn't.
        ransac_attempts -- How many movement estimates to perform when there
                           are not enough inliers while gradually increasing
                           acceptable reprojection error.
        ransac_attempt_reprojection_increase_factor -- Multiplicative factor
             gradually increasing acceptable reprojection error while looking
             for sufficiently many inliers.
        min_inliers -- Minimum number of inliers for a movement estimate to be
                       considered valid. Lower number of inliers leads to less
                       frequent creation of key frames and thus to lower error
                       accumulation over key frames. However, it also leads to
                       less precise estimates of the movements since and between
                       key frames. That in turn leads to higher total error. The
                       right value of this parameter needs to hit the sweet spot
                       in this trade-off.
        max_rotation_since_key_frame -- Maximum rotation since key frame that
             we consider OK to still trust the estimate. In radians.
        max_translation_since_key_frame -- Maximum translation since key frame
            that we consider OK to still trust the estimate. In meters.
        max_avg_static_scene_intensity_change
        """
        self.camera_matrix = np.zeros((3, 3))
        self.camera_matrix[0, 0] = camera_focal_length
        self.camera_matrix[1, 1] = camera_focal_length
        self.camera_matrix[0, 2] = camera_center[0]
        self.camera_matrix[1, 2] = camera_center[1]

        self.min_valid_depth = min_valid_depth
        self.max_valid_depth = max_valid_depth

        self.max_corners = max_corners
        self.corner_quality_level = corner_quality_level
        self.corner_distance = corner_distance

        self.subpix_win = subpix_win
        self.subpix_iters = subpix_iters

        self.corner_matching_window_size = corner_matching_window_size
        self.ransac_reprojection_error = ransac_reprojection_error
        self.ransac_confidence = ransac_confidence
        self.ransac_pnp_method = ransac_pnp_method
        self.ransac_attempts = ransac_attempts
        self.ransac_attempt_reprojection_increase_factor = (
            ransac_attempt_reprojection_increase_factor)
        self.min_inliers = min_inliers
        self.max_rotation_since_key_frame = max_rotation_since_key_frame
        self.max_translation_since_key_frame = max_translation_since_key_frame
        self.max_avg_static_scene_intensity_change = (
                max_avg_static_scene_intensity_change)


class CameraFrame:
    """Encapsulates and pre-processes image and depth data."""
    def __init__(self, img, depth, params):
        """Initialize CameraFrame.

        img -- Greyscale image.
        depth -- np.float32 depth image in meters.
        params -- VisualOdometryParams.
        """
        self.img = img.copy()
        self.depth = depth.copy()
        self.depth_is_valid = np.logical_and(
            self.depth >= params.min_valid_depth,
            self.depth <= params.max_valid_depth)

        # In the Virtual Track of Subterranean Challenge, the world is not rich
        # enough for more advanced features, such as SURF, to work well. Corner
        # detectors seem to be faster and work better.
        self.corners = cv2.goodFeaturesToTrack(
                self.img, params.max_corners, params.corner_quality_level,
                params.corner_distance, None,
                self.depth_is_valid.astype(np.uint8))

        if self.corners is not None:
            self.corners = self.corners[:,0,:]

        if (self.corners is not None and self.corners.shape[0] > 0
                and params.subpix_iters > 0):
            term_crit = (cv2.TermCriteria_COUNT, params.subpix_iters, -1)
            self.corners = cv2.cornerSubPix(
                    img, self.corners, (params.subpix_win, params.subpix_win),
                    (-1, -1), term_crit)

            # Subpixel corner detection sometimes yields points outside of the
            # input image, which causes troubles with their processing. Avoid
            # them.
            in_picture = np.logical_and.reduce([
                self.corners[:, 0] >= 0, self.corners[:, 0] < img.shape[1],
                self.corners[:, 1] >= 0, self.corners[:, 1] <= img.shape[0]
            ])
            self.corners = self.corners[in_picture]
        else:
            self.corners = np.zeros((0, 2), dtype=np.float32)


# Conversion transform from OpenCV's coordinates to Osgar's.
# In Osgar, X points forward, Y left and Z up.
# In OpenCV, X points right, Y down and Z forward.
CV2OSGAR = np.array([[0, 0, 1, 0],
                     [-1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])
# Conversion transform from Osgar's coordinate frame to OpenCV's.
OSGAR2CV = inverse_rigid_transform(CV2OSGAR)


def osgar_xyz_to_opencv(pts):
    """Transform (X, Y, Z, 1) points from OpenCV's coordinate system to Osgar's.

    pts -- Row-major list of point coordinates.
    """
    pts = np.asarray(pts)
    assert(pts.shape[-1] == 4)
    return (OSGAR2CV @ pts.T).T


class KeyFrame:
    """Represents a preprocessed key frame for visual odometry.

    Preprocessing consists of computing XYZ coordinates of feature points 
    observed in the frame (= image + depth) both in Osgar's coordinate system
    (X forward, Y left, Z up) and in OpenCV's (X right, Y down, Z forward).

    Useful object properties: `corners_xyz` and `corners_cv_xyz`.
    """
    def __init__(self, frame, params):
        """Initialize and preprocess the key frame.

        frame -- CameraFrame with image & depth information and detected feature
                 points.
        params -- VisualOdometryParams.
        """
        self.frame = frame

        assert (frame.corners is not None)
        # For best results, we should do a bilinear (?) interpolation between
        # depths in the surrounding four pixels around the subpixel feature
        # coordinate. The current solution is simpler and seems to work well
        # enough.
        corner_idxs = np.rint(self.frame.corners).astype(np.int)
        corner_depths = frame.depth[corner_idxs[:, 1], corner_idxs[:, 0]]
        camera_f = params.camera_matrix[0, 0]
        assert(camera_f == params.camera_matrix[1, 1])
        assert (camera_f > 0)
        camera_cx = params.camera_matrix[0, 2]
        camera_cy = params.camera_matrix[1, 2]
        # Coordinates in Osgar's system, i.e. "x forward".
        self.corners_xyz = (np.vstack([
            np.ones((corner_idxs.shape[0], )),
            (camera_cx - corner_idxs[:, 0]) / camera_f,
            (camera_cy - corner_idxs[:, 1]) / camera_f,
        ]).T * corner_depths.reshape((-1, 1))).astype(np.float32)
        if self.corners_xyz.shape[0] > 0:
            self.corners_xyz = cv2.convertPointsToHomogeneous(
                    self.corners_xyz)[:,0,:]
        else:
            # Something to keep us from crashing later when there is nothing
            # to work with.
            self.corners_xyz = np.zeros((0, 4), dtype=np.float32)

        # To OpenCV's coordinate system, i.e. "z forward".
        self.corners_cv_xyz = osgar_xyz_to_opencv(self.corners_xyz)


class CameraVisualOdometry:
    """RGBD visual odometry in a camera-centric coordinate system.

    It reports pose relative to the starting position of the camera. Most
    likely, you should use `RobotVisualOdometry` or `GlobalVisualOdometry`
    wrappers instead of using `CameraVisualOdometry` directly.

    The odometry algorithm utilizes information from a depth camera. Optionally,
    it can incorporate trusted information about heading, for example from an
    IMU + accelerometer + compass sensor, and from an altimeter.

    Terminology: Frame = image + depth

    The algorithm starts with creating a key frame. The following frames are
    localized relative to the key frame by minimizing projection errors of
    feature points from the key frame in the new image. If the algorithm fails
    to find satisfactory position estimate of the new frame relative to the key
    frame, it replaces the key frame with a new key frame created from the last
    preceding frame (i.e. a frame closer to the current frame) and tries again.
    With such new key frame in place, the cycle repeats.

    The algorithm reverts to a constant motion assumption if all attempts to
    estimate what is happening fail.

    The optional supplemental information about orientation and altitude are
    fully trusted and replace corresponding estimates from odometry.

    This is pure odometry and does not close loops and does not try to
    re-localize itself after getting lost.

    There is no guarantee on smoothness of the produced estimates.
    """
    def __init__(self, odometry_params):
        """Initialize CameraVisualOdometry

        odometry_params -- VisualOdometryParams.
        """
        self.params = odometry_params

        self.prev_frame = None
        self.key_frame = None
        self.key_frame_pose = None

        self.trajectory = [no_change_rigid_transform()]

    def can_advance_keyframe(self):
        """Return `True` iff `self.key_frame` can be updated to a new frame."""
        return (self.key_frame is not None
                and self.key_frame.frame is not self.prev_frame)

    def advance_keyframe(self):
        """Turn the last observed frame into a new key frame."""
        self.key_frame = KeyFrame(self.prev_frame, self.params)
        self.key_frame_pose = self.pose

    @property
    def xyz(self):
        "XYZ coordinates relative to initial position of the camera in meters."
        return self.pose[:3, 3]

    @property
    def rpy(self):
        """Roll, pitch and yaw relative to initial orientation of the camera.

        In radians.
        """
        return rotation_matrix_to_euler_angles(self.pose[:3, :3])

    @property
    def pose(self):
        """ Rotation & translation matrix representing current pose.

        4x4 rigid transformation matrix capturing current rotation and
        translation of the camera relative to its initial position.

        I.e., this matrix transforms from coordinate system centered at the
        *current* camera location to a coordinate frame centered at the initial
        camera location.

        |R t|
        |0 1|
        """
        return self.trajectory[-1]

    def update(self, img, depth, camera_orientation=None, altitude=None,
               visualize=False):
        """Update the odometry estimate, extend `self.trajectory`.

        Depending on circumstances, key frame may get advanced.

        img -- Greyscale image.
        depth -- np.float32 depth map with the same resolution as `img`.
        camera_orientation -- Optional 3x3 rotation matrix representing
                              orientation *of the camera* that we want to get
                              *after* the update finishes.
        altitude -- Optional altitude *of the camera* that we want to get
                    *after* the update finishes.
        visualize -- If set to `True`, display an extra visualization with
                     detected image features and their tracking.
        """
        # Warning for developers:
        #  This function can recursively call itself. Be careful about infinite
        #  recursion. The recursive call happens when the estimation fails, but
        #  the used key frame is old-ish and can be replaced with the previous
        #  frame. After replacing the old-ish key frame, the function calls
        #  itself in what is essentially "GOTO Line 0 & try again". The idea is
        #  that should happen at most once per input image.
        assert(len(img.shape) == 2)  # Expecting a greyscale image.
        frame = CameraFrame(img, depth, self.params)


        camera_f = self.params.camera_matrix[0, 0]
        camera_cx = self.params.camera_matrix[0, 2]
        camera_cy = self.params.camera_matrix[1, 2]

        if self.key_frame is None:
            # We have just started.
            self.prev_frame = frame
            self.advance_keyframe()
            return self.pose

        # Special case for static scenes, when the robot is not moving.
        if (np.mean(np.abs(self.prev_frame.img.astype(np.int) -
                           img.astype(np.int))) <=
            self.params.max_avg_static_scene_intensity_change):
            self.prev_frame = frame
            self.trajectory.append(self.pose.copy())
            return self.pose

        # Assuming constant motion and constant time between two consecutive
        # frames, guess that the current movement is similar to the previous
        # one.
        # `rt` stands for "rotation & translation".
        if len(self.trajectory) >= 2:
            prev_rt = (inverse_rigid_transform(self.trajectory[-2]) @
                       self.trajectory[-1])
        else:
            # No movement yet.
            prev_rt = no_change_rigid_transform()

        # Reading from the right:
        #  * Convert from points in coordinate system centered at key frame to
        #    a coordinate system centered where the camera started.
        #  * Convert back from that coordinate system to a coordinate system
        #    centered at our current guessed pose, which is based on previous
        #    pose extended with the same movement as observed in previous
        #    update (= assumption of constant velocity and rotation).
        guessed_pose = self.pose @ prev_rt
        initial_estimate = (inverse_rigid_transform(guessed_pose) @
                            self.key_frame_pose)
        assert (tuple(initial_estimate.shape) == (4, 4))
        # This is where we expect feature points in the key frame to be relative
        # to our guessed current position.
        curr_corners_xyz = np.asarray(
            (initial_estimate @ self.key_frame.corners_xyz.T).T)

        # To OpenCV's coordinate system.
        curr_corners_cv_xyz = osgar_xyz_to_opencv(curr_corners_xyz)

        # Initial guess of pixel coordinates of key frame feature points
        # in the current image.
        if curr_corners_cv_xyz.shape[0] > 0:
            dist_coeffs = None
            curr_corners_guess = cv2.projectPoints(
                # Discarding the 1.0 last column from homogeneous
                # coordinates.
                curr_corners_cv_xyz[:, :3],
                np.zeros(3,),  # rvec
                np.zeros(3,),  # tvec
                self.params.camera_matrix,
                dist_coeffs)[0].reshape((-1, 2)).astype(np.float32)
        else:
            # Something not None to avoid crashes later.
            curr_corners_guess = np.zeros((0, 2), dtype=np.float32)

        # Ideally, we would precompute the pyramids and re-use them,
        # but no luck: "Can't pass precalculated pyramids to
        #              calcOpticalFlowPyrLK in Python"
        #              https://github.com/opencv/opencv/issues/4777
        curr_corners, status, _ = cv2.calcOpticalFlowPyrLK(
            self.key_frame.frame.img,
            frame.img,
            self.key_frame.frame.corners,
            # This copy is, strictly speaking not needed, but avoiding
            # OpenCV overwriting existing data makes debugging easier,
            # because it allows us to compare our initial guess with
            # the output of PyrLK detection.
            curr_corners_guess.copy(),
            winSize=(self.params.corner_matching_window_size,
                     self.params.corner_matching_window_size),
            flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
        if curr_corners is None:
            # To keep the code below happy.
            curr_corners = np.zeros((0, 2), dtype=np.float32)
            status = np.zeros((0, 1), dtype=np.int)

        status = status.astype(np.bool).reshape((-1, ))

        # Only keep corners we could track from key frame to the current image.
        kf_corners = self.key_frame.frame.corners[status]
        curr_corners = curr_corners[status]
        kf_corners_xyz = self.key_frame.corners_xyz[status]
        kf_corners_cv_xyz = self.key_frame.corners_cv_xyz[status]

        # If not enough tracked points and key frame is old-ish,
        # turn the previous frame into a key frame and try an update
        # with that.
        if (kf_corners.shape[0] < self.params.min_inliers
                and self.can_advance_keyframe()):
            self.advance_keyframe()
            return self.update(img, depth, camera_orientation, altitude,
                               visualize)

        # Given feature points in key frame, their 3D coordinates relative to
        # the key frame pose and their projection in the current image, estimate
        # the movement between the key frame and current pose.
        ret = False
        inliers = []
        if kf_corners_cv_xyz.shape[0] >= self.params.min_inliers:
            try:
                # `guess` is the same thing as `initial_estimate`, i.e.
                # our estimate of current movement based on previous movement,
                # except it is in OpenCV's coordinate system style (z forward)
                # instead of in Osgar-style system (x forward).
                guess = OSGAR2CV @ initial_estimate @ CV2OSGAR
                guessed_rvec = np.array(cv2.Rodrigues(guess[:3, :3])[0],
                                        dtype=np.float32)
                guessed_tvec = np.array(guess[:3, 3], dtype=np.float32)
                max_reprojection_error = self.params.ransac_reprojection_error
                dist_coeffs = None
                for _ in range(self.params.ransac_attempts):
                    ret, rvec, tvec, inliers = cv2.solvePnPRansac(
                        kf_corners_cv_xyz[:, :3],
                        curr_corners.astype(np.float32),
                        self.params.camera_matrix,
                        dist_coeffs,
                        # Copy not needed, except to make debugging easier.
                        guessed_rvec.copy(),
                        guessed_tvec.copy(),
                        # According to some, but not all, documentation,
                        # only SOLVE_PNP_ITERATIVE uses the initial
                        # estimate. But that one, empirically speaking, does
                        # not work well.
                        # With OpenCV 4.2, enabling usage of the extrinsic guess
                        # indeed does to change the results. At least with
                        # SOLVEPNP_UPNP.
                        useExtrinsicGuess=True,
                        reprojectionError=max_reprojection_error,
                        confidence=self.params.ransac_confidence,
                        flags=self.params.ransac_pnp_method)
                    if ret:
                        break
                    max_reprojection_error *= (
                        self.params.ransac_attempt_reprojection_increase_factor
                    )
            except cv2.error as e:
                g_logger.error('RANSAC is unhappy: {e}')
            if inliers is None:
                # Like in the song by Nazareth: "None hurts ...". Or was it
                # love?
                inliers = []

        # If we don't like the solution and can try with a fresher key frame,
        # we try again with a fresher key frame.
        if ((not ret or len(inliers) < self.params.min_inliers)
                and self.can_advance_keyframe()):
            self.advance_keyframe()
            return self.update(img, depth, camera_orientation, altitude,
                               visualize)

        if ret:  # OK, so we got a solution ...
            transform = as_rigid_transform(xyz=tvec[:, 0],
                                           rot=cv2.Rodrigues(rvec)[0])
            # We need the transform in Osgar's coordinate system.
            transform = CV2OSGAR @ transform @ OSGAR2CV

            # Our RANSAC calculated transform from key frame pose to current
            # frame pose, but we need the other direction.
            transform = inverse_rigid_transform(transform)

            # If we have an external estimate of the orientation of the camera,
            # we can correct the movement estimate.
            if camera_orientation is not None:
                # We are looking for a `correction` rotation such that after
                # we combine current motion estimate (`transform`) with previous
                # pose estimate, the new estimate's orientation matches
                # `camera_orientation`. I.e.:
                #   camera_orientation == key_frame_rotation @
                #                         correction @
                #                         transform_rotation
                inv_kf_pose_rot = inverse_rotation(self.key_frame_pose[:3, :3])
                inv_transform_rot = inverse_rotation(transform[:3, :3])
                assert (camera_orientation.shape == (3, 3))
                correction = as_rigid_transform(rot=(inv_kf_pose_rot @
                                                     camera_orientation @
                                                     inv_transform_rot))
                transform = correction @ transform

            # If we got too far from the key frame and we can switch to a new
            # key frame, we should do that. Otherwise it is safer to not believe
            # the estimate, because exceeding distance or rotation since key
            # frame when key frame is the *previous frame* (!) is very
            # suspicious.
            dist = np.linalg.norm(transform[:3, 3])
            # Take (1, 0, 0) vector, rotate it, measure the (cosine of) angle
            # between the new vector and the original (1, 0, 0) vector.
            rotation_value_cos = (transform[0, 0] /
                                  (1e-9 + np.linalg.norm(transform[0, :3])))
            if (dist > self.params.max_translation_since_key_frame
                    or rotation_value_cos < np.cos(
                        self.params.max_rotation_since_key_frame)):
                if self.can_advance_keyframe():
                    self.advance_keyframe()
                    return self.update(img, depth, camera_orientation,
                                       altitude, visualize)
                else:
                    if camera_orientation is not None:
                        # We can trust the rotation part, but not the
                        # translation part.
                        transform[:3, 3] = 0
                    else:
                        # We should not even trust the rotation part.
                        transform = no_change_rigid_transform()

            # New camera pose consists of transforming from current
            # camera coordinate frame to the coordinate frame of the key frame
            # (using `transform`) and from there to the coordinate system
            # centered where the camera started (using key_frame_pose).
            # Together, this makes a transform from "here" to "where camera
            # started", i.e. "current pose".
            self.trajectory.append(self.key_frame_pose @ transform)
        else:  # Oh no, RANSAC has not found anything!
            # Fallback to assuming constant velocity and rotation, i.e.
            # using `prev_rt` instead of `transform`.

            if camera_orientation is not None:
                # Similar calculation to the correction above, so see the
                # comment there.
                inv_pose_rot = inverse_rotation(self.pose[:3, :3])
                inv_prev_rot = inverse_rotation(prev_rt[:3, :3])
                correction = as_rigid_transform(
                    rot=(inv_pose_rot @ camera_orientation @ inv_prev_rot))
            else:
                correction = no_change_rigid_transform()

            # Similar reasoning as trajectory update above, except using
            # (possibly corrected for rotation) estimate from the previous
            # movement as our best guess.
            self.trajectory.append(self.pose @ correction @ prev_rt)

        # Make sure that despite all the chained matrix multiplications, the
        # rotation submatrix still represents rotation and does not include
        # scale and shear.
        self.pose[:3, :3] = near_rotation_matrix(self.pose[:3, :3])

        # It would be nice to apply some more advanced correction, like we can
        # do with rotation, but for now I don't see a generic one that is
        # guaranteed to exist. So let's do a simple correction.
        #
        # The difficult situation to correct is: Let's say that the estimated
        # movement (without any rotation) says dx,dy,dz==1,1,1. But altitude
        # change says that dz==4. What does it mean for dx and dy? That we
        # should quadruple them, to keep the proportions? Or that the robot
        # changed direction and most of the estimated travelled distance went
        # into dz and therefore dx and dy should be smaller? How much smaller,
        # if dz from atlimeter is a bigger change than the whole estimated
        # movement?
        if altitude is not None:
            self.pose[2, 3] = altitude

        self.prev_frame = frame

        if visualize:
            shown_img = cv2.cvtColor(frame.img, cv2.COLOR_GRAY2BGR)
            inls = np.zeros((curr_corners.shape[0], ), dtype=np.bool)
            inls[inliers] = True
            for kfc, cc, inl in zip(kf_corners, curr_corners, inls):
                # Inliers in green, outliers in red.
                color = (0, 255, 0) if inl else (0, 0, 255)
                kfc = tuple(kfc.astype(np.int))
                cc = tuple(cc.astype(np.int))
                cv2.circle(shown_img, cc, 2, color)
                cv2.line(shown_img, cc, kfc, color)
            cv2.imshow('Tracked features', shown_img)

        return self.pose


class RobotVisualOdometry:
    """Visual odometry in coordinate frame centered where robot started.

    You can use this class directly only if your global coordinate system
    aligns wit where the robot starts. Otherwise you should use
    `GlobalVisualOdometry`.
    """
    def __init__(self, camera_xyz, camera_rpy, odometry_params):
        """Initialize RobotVisualOdometry.

        camera_xyz -- XYZ triplet, in meters, representing position of the
                      camera relative to the center of the robot.
        camera_rpy -- Roll, pitch and yaw of the camera relative to the
                      orientation of the robot.
        odometry_params -- VisualOdometryParams.
        """
        self.camera_odometry = CameraVisualOdometry(odometry_params)

        self.camera_to_robot = as_rigid_transform(xyz=camera_xyz,
                                                  rpy=camera_rpy)
        self.robot_to_camera = inverse_rigid_transform(self.camera_to_robot)

        self.trajectory = [
            self.camera_to_robot @ self.camera_odometry.pose @
            self.robot_to_camera
        ]

    @property
    def xyz(self):
        """XYZ coordinates relative to initial position of the robot."""
        return np.asarray(self.pose)[:3, 3]

    @property
    def rpy(self):
        """Roll, pitch and yaw relative to initial orientation of the robot."""
        return rotation_matrix_to_euler_angles(self.pose[:3, :3])

    @property
    def pose(self):
        """Pose of the robot relative to its initial position.

        4x4 rigid transformation matrix capturing current rotation and
        translation of the robot relative to its initial position.

        I.e., this matrix transforms from coordinate system centered at the
        *current* robot location to a coordinate frame centered at the initial
        location of the robot.

        |R t|
        |0 1|
        """
        return self.trajectory[-1]

    def update(self, img, depth, imu_orientation=None, altitude=None,
               visualize=False):
        """Update the odometry estimate, extend `self.trajectory`.

        Depending on circumstances, key frame may get advanced.

        img -- Greyscale image.
        depth -- np.float32 depth map with the same resolution as `img`.
        imu_orientation -- Optional 3x3 rotation matrix representing
                           orientation *of the robot*, in the coordinate system
                           relative to robot's starting position, that we want
                           to get *after* the update finishes.
        altitude -- Optional altitude *of the robot*, in the coordinate system
                    relative to robot's starting position, that we want to get
                    *after* the update finishes.
        visualize -- If set to `True`, display an extra visualization with
                     detected image features and their tracking.
        """
        camera_orientation = (None if imu_orientation is None else
                              self.robot_to_camera[:3, :3] @ imu_orientation)
        camera_altitude = (None if altitude is None else
                           (self.robot_to_camera @
                            np.array([[0., 0., altitude, 1.]]).T)[2, 0])
        cam_pose = self.camera_odometry.update(img, depth, camera_orientation,
                                               camera_altitude, visualize)
        # To convert robot's center (0, 0, 0) in coordinate frame relative to
        # current position of the robot to a point relative to the initial
        # position of the robot, we need to do the following (reading from the
        # right):
        #  * Convert to the coordinate frame relative to current position of the
        #    camera.
        #  * Apply the odometry transform from camera that maps to coordinate
        #    frame centered at initial position of the camera.
        #  * Map that to the coordinate frame centered where the robot started.
        robot_pose = self.camera_to_robot @ cam_pose @ self.robot_to_camera
        self.trajectory.append(robot_pose)
        return robot_pose


class GlobalVisualOdometry:
    """Visual odometry in a given global coordinate frame.

    If your robot does not start at (0, 0, 0), heading in the direction of axis
    X, this is likely the class you want to use.
    """
    def __init__(self, initial_xyz, initial_rpy, camera_xyz, camera_rpy,
                 odometry_params):
        """Initialize GlobalVisualOdometry.

        initial_xyz -- Initial XYZ of the robot relative to global coordinate
                       coordinate frame, in meters.
        initial_rpy -- Initial roll, pitch and yaw of the robot relative to
                       global coordinate system, in radians.
        camera_xyz -- XYZ triplet, in meters, representing position of the
                      camera relative to the center of the robot.
        camera_rpy -- Roll, pitch and yaw of the camera relative to the
                      orientation of the robot.
        odometry_params -- VisualOdometryParams.
        """
        self.robot_odometry = RobotVisualOdometry(camera_xyz, camera_rpy,
                                                  odometry_params)

        self.robot_to_global = as_rigid_transform(xyz=initial_xyz,
                                                  rpy=initial_rpy)
        self.global_to_robot = inverse_rigid_transform(self.robot_to_global)

        self.trajectory = [self.robot_to_global @ self.robot_odometry.pose]

    @property
    def xyz(self):
        """XYZ coordinates of the robot in global coordinate system."""
        return np.asarray(self.pose)[:3, 3]

    @property
    def rpy(self):
        """Roll, pitch and yaw of the robot in global coordinate system."""
        return rotation_matrix_to_euler_angles(self.pose[:3, :3])

    @property
    def pose(self):
        """Pose of the robot in the global coordinate system.

        4x4 rigid transformation matrix capturing current rotation and
        translation in the fixed global coordinate system.

        I.e., this matrix transforms from coordinate system centered at the
        *current* robot location to a coordinate frame centered at *global*
        xyz=[0, 0, 0], rpy=[0, 0, 0].

        |R t|
        |0 1|
        """
        return self.trajectory[-1]

    def update(self, img, depth, imu_orientation=None, altitude=None,
               visualize=False):
        """Update the odometry estimate, extend `self.trajectory`.

        Depending on circumstances, key frame may get advanced.

        img -- Greyscale image.
        depth -- np.float32 depth map with the same resolution as `img`.
        imu_orientation -- Optional 3x3 rotation matrix representing
                           orientation *of the robot* in the *global* coordinate
                           system that we want to get *after* the update
                           finishes.
        altitude -- Optional altitude *of the robot* in the *global* coordinate
                    system that we want to get *after* the update finishes.
        visualize -- If set to `True`, display an extra visualization with
                     detected image features and their tracking.
        """
        robot_orientation = (None if imu_orientation is None else
                             self.global_to_robot[:3, :3] @ imu_orientation)
        robot_altitude = (None if altitude is None else
                         (self.global_to_robot @
                            np.array([0., 0., altitude, 1.]))[2, 0])
        robot_pose = self.robot_odometry.update(img, depth, robot_orientation,
                                               robot_altitude, visualize)
        # To convert robot's center (0, 0, 0) in coordinate frame relative to
        # current position of the robot to a point relative to the initial
        # position of the robot, we need to do the following (reading from the
        # right):
        #  * Convert to the coordinate frame relative to current position of the
        #    camera.
        #  * Apply the odometry transform from camera that maps to coordinate
        #    frame centered at initial position of the camera.
        #  * Map that to the coordinate frame centered where the robot started.
        global_pose = self.robot_to_global @ robot_pose
        self.trajectory.append(global_pose)
        return global_pose


class Localization(osgar.node.Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        odometry_params = VisualOdometryParams(config['focal-length'],
                                               config['principal-point'],
                                               min_valid_depth=config.get('min-depth', 0.01),
                                               max_valid_depth=config.get('max-depth', 10.0))
        publish_pose3d = functools.partial(self.bus.publish, 'pose3d')
        self.demo = Demo(config['xyz'], np.radians(config['rpy']), odometry_params, callback=publish_pose3d)
        self.bus.register('pose3d')
        cv2.setNumThreads(1)

    def on_origin(self, data):
        if len(data) == 8:
            robot_name, x, y, z, qa, qb, qc, qd = data
            rpy = osgar.lib.quaternion.euler_zyx((qa, qb, qc, qd))[::-1]
            self.demo.on_initial_pose([x, y, z], rpy)

    def on_orientation(self, data):
        quaternion = data
        ypr = osgar.lib.quaternion.euler_zyx(quaternion)
        rpy = ypr[::-1]
        imu_rpy = rpy
        return self.demo.on_imu_rpy(imu_rpy)

    def on_image(self, data):
        grey_img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        color_img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        return self.demo.on_img(self.time.total_seconds(), grey_img, color_img)

    def on_depth(self, data):
        # Converting to meters.
        depth = (data * 1e-3).astype(np.float32)
        return self.demo.on_depth(self.time.total_seconds(), depth)

    def on_baseline(self, data):
        baseline_xyz, baseline_quaternion = data
        baseline_ypr = osgar.lib.quaternion.euler_zyx(baseline_quaternion)
        baseline_rpy = baseline_ypr[::-1]
        self.demo.on_baseline(baseline_xyz, baseline_rpy)

    def run(self):
        try:
            # wait for valid origin
            while True:
                dt, channel, data = self.bus.listen()
                if channel == "origin":
                    self.on_origin(data)
                    if self.demo.odometry is not None:
                        break

            while True:
                dt, channel, data = self.bus.listen()
                self.time = dt
                if channel == "orientation":
                    self.on_orientation(data)
                elif channel == "depth":
                    self.on_depth(data)
                elif channel == "image":
                    self.on_image(data)
                elif channel == "baseline":
                    self.on_baseline(data)
                elif channel == "origin":
                    pass
                else:
                    assert False, "unknown input channel"
        except osgar.bus.BusShutdownException:
            pass


class Demo:
    """Sample application and visualization of GlobalVisualOdometry."""
    def __init__(self, camera_xyz, camera_rpy, odometry_params, visualize=False,
                 output_trajectory_img_path=None, callback=None):
        """Initialize the demo.

        To complete the initialization, you also need to provide initial pose
        to on_initial_pose(). This can come from log or can be a fixed one.

        camera_xyz -- Placement of the camera on the robot.
        camera_rpy -- Roll, pitch and yaw (radians) of camera orientation on the
                      robot.
        odometry_params -- VisualOdometryParams()
        callback -- is called whenever a new estimate is available
        visualize -- If True, show extra visualizations.
        output_trajectory_img_path -- Path to an image file where trajectory
                                      should be stored.
        """
        self.camera_xyz = camera_xyz
        self.camera_rpy = camera_rpy
        self.odometry_params = odometry_params
        self.odometry = None
        self.callback = callback
        self.visualize = visualize
        self.output_trajectory_img_path = output_trajectory_img_path

        # Information about sensor measurements.
        self.depth = None
        self.depth_time = None
        self.grey_img = None
        self.color_img = None
        self.img_time = None
        self.baseline_xyz = None
        self.baseline_rpy = None
        self.baseline_trajectory_xy = []
        self.altitude = None
        self.imu_rpy = None

        # For visualization. Minimum and maximum X and Y values observed in the
        # trajectory. They are merged into the same variables to keep the
        # scaling ratio same for both axes when visualizing the trajectory.
        self.min_xy = self.max_xy = 0
        self._waitKey_timeout = 0

    def on_initial_pose(self, initial_xyz, initial_rpy):
        """Finish initialization by setting global coordinate frame."""
        self.odometry = GlobalVisualOdometry(initial_xyz, initial_rpy,
                                             self.camera_xyz, self.camera_rpy,
                                             self.odometry_params)
        # To keep baseline_trajectory same length as odometry.trajectory, so
        # that pairwise comparison matches related positions.
        initial_pose = as_rigid_transform(xyz=initial_xyz, rpy=initial_rpy)
        self.baseline_trajectory_xy = [np.asarray(
            initial_pose[:2,3]).tolist()]

    def __maybe_update_odometry_and_visualize(self):
        """Update odometry with a matching accumulated image-depth pair.

        If initial pose is not yet known or there is no matching accumulated
        image-depth pair available, do nothing.

        Return True if awaiting more data, False if user requested a stop.
        """
        if self.odometry is None:
            # This can happen before the first information about origin arrives
            # and we need to wait a bit more.
            return True

        if (self.grey_img is None or self.depth is None
                or abs(self.depth_time - self.img_time) > 1e-2):
            return True  # Nothing to do now, but that's OK.

        imu_orientation = (None if self.imu_rpy is None else
                           euler_angles_to_rotation_matrix(*self.imu_rpy))
        global_pose = self.odometry.update(self.grey_img, self.depth,
                                           imu_orientation, self.altitude,
                                           self.visualize)
        xyz = np.asarray(global_pose)[:3, 3]
        rpy = rotation_matrix_to_euler_angles(global_pose[:3, :3])
        quat = osgar.lib.quaternion.from_rotation_matrix(global_pose[:3, :3])
        if self.callback is not None:
            # `tolist` changes numpy scalars to Python scalars
            self.callback([xyz.tolist(), quat])

        # User interface, ha ha.
        g_logger.info('Estimate: xyz={}, rpy={}'.format(xyz, np.degrees(rpy)))
        if self.baseline_xyz is not None:
            drift = np.sqrt(np.sum((xyz - np.asarray(self.baseline_xyz))**2))
            g_logger.info('Baseline: xyz={}, rpy={}'.format(
                self.baseline_xyz, np.degrees(self.baseline_rpy)))
            g_logger.info(f'Drift: {drift}')

        if self.baseline_xyz is not None:
            self.baseline_trajectory_xy.append(self.baseline_xyz[:2])
            self.min_xy = min(self.min_xy, min(*self.baseline_xyz[:2]))
            self.max_xy = max(self.max_xy, max(*self.baseline_xyz[:2]))

        self.min_xy = min(self.min_xy, min(*xyz[:2]))
        self.max_xy = max(self.max_xy, max(*xyz[:2]))

        if self.visualize or self.output_trajectory_img_path:
            trajectory_img = np.zeros((1000, 1000, 3), dtype=np.uint8)

            def img_coord(wx, wy):
                MARGIN = 1
                ix = int(trajectory_img.shape[1] * (wx -
                                                    (self.min_xy - MARGIN)) /
                         ((self.max_xy + MARGIN) - (self.min_xy - MARGIN)))
                iy = int(trajectory_img.shape[0] * (wy -
                                                    (self.min_xy - MARGIN)) /
                         ((self.max_xy + MARGIN) - (self.min_xy - MARGIN)))
                return ix, trajectory_img.shape[0] - MARGIN - iy

            #assert(len(self.baseline_trajectory_xy) == len(self.odometry.trajectory))
            for bxy_a, bxy_b in zip(self.baseline_trajectory_xy[:-2],
                                    self.baseline_trajectory_xy[1:]):
                cv2.line(trajectory_img, img_coord(*bxy_a), img_coord(*bxy_b),
                         (0, 255, 0))
            for g_pose_a, g_pose_b in zip(self.odometry.trajectory[:-2],
                                          self.odometry.trajectory[1:]):
                cv2.line(trajectory_img,
                         img_coord(g_pose_a[0, 3], g_pose_a[1, 3]),
                         img_coord(g_pose_b[0, 3], g_pose_b[1, 3]),
                         (0, 255, 255))
            cv2.imshow(
                'Trajectory: green - baseline, yellow - visual ' + 'odometry',
                trajectory_img)
            if self.output_trajectory_img_path:
                cv2.imwrite(self.output_trajectory_img_path, trajectory_img)

            if self.visualize:
                cv2.imshow('img', self.color_img)
                KEY_Q = ord('q')
                KEY_SPACE = ord(' ')
                key = cv2.waitKey(self._waitKey_timeout) & 0xFF
                if key == KEY_Q:
                    return False
                if key == KEY_SPACE:
                    self._waitKey_timeout = 0 if self._waitKey_timeout > 0 else 1

        # Discard measurements, because they are now obsolete ("consumed.")
        self.imu_rpy = None
        self.altitude = None
        self.grey_img = None
        self.color_img = None
        self.depth = None

        return True

    def on_img(self, timestamp, grey_img, color_img):
        """Accumulate image information and maybe update odometry.

        Update odometry and visualize in case there is a matching image-depth
        pair available now.

        timestamp -- current time in seconds
        grey_img -- Greyscale image used for visual odometry.
        color_img -- color image used for visualization to the user.

        Return True if awaiting more data, False if user requested a stop.
        """
        self.img_time = timestamp
        self.grey_img = grey_img
        self.color_img = color_img
        return self.__maybe_update_odometry_and_visualize()

    def on_depth(self, timestamp, depth):
        """Accumulate depth information and maybe update odometry.

        Update odometry and visualize in case there is a matching image-depth
        pair available now.

        timestamp -- current time in seconds
        depth -- np.float32 depth map in meters

        Return True if awaiting more data, False if user requested a stop.
        """
        self.depth_time = timestamp
        self.depth = depth
        return self.__maybe_update_odometry_and_visualize()

    def on_imu_rpy(self, imu_rpy):
        """Remember measurement from IMU."""
        self.imu_rpy = imu_rpy

    def on_altitude(self, altitude):
        """Remember measurement from altimeter."""
        self.altitude = altitude

    def on_baseline(self, baseline_xyz, baseline_rpy):
        """Remember baseline trajectory."""
        self.baseline_xyz = baseline_xyz
        self.baseline_rpy = baseline_rpy


def main():
    cv2.setNumThreads(1)
    import argparse
    import json

    from osgar.lib.quaternion import euler_zyx
    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id

    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', help='Path to Osgar log.')
    parser.add_argument('--origin-stream',
                        help='Stream id or name with data about robot\'s ' +
                        'coordinates in the starting area.')
    parser.add_argument('--image-stream',
                        help='Stream id or name with image data.')
    parser.add_argument('--depth-stream',
                        help='Stream id or name with depth data.')
    parser.add_argument('--imu-stream',
                        help='Optional stream id or name with imu data.')
    parser.add_argument('--altitude-stream',
                        help='Optional stream id or name with altitude data.')
    parser.add_argument(
        '--baseline-3d-stream',
        help='Optional stream id or name with 3d coordinates ' +
        'to compare to.')
    parser.add_argument('--camera-focal-length',
                        type=float,
                        default=462.1,
                        help='Focal length of the camera in pixels.')
    parser.add_argument('--camera-center',
                        type=float,
                        nargs=2,
                        default=[319.5, 179.5],
                        help='X and Y coordinate of focal point in pixels.')
    parser.add_argument('--camera-xyz',
                        type=float,
                        nargs=3,
                        default=[0.23, 0, 0.19 + 0.06256005],
                        help='Placement of camera on the robot.')
    parser.add_argument('--camera-rpy',
                        type=float,
                        nargs=3,
                        default=[0., 0., 0.],
                        help='Heading of camera on the robot. Roll, pitch ' +
                        'and yaw in degrees.')
    parser.add_argument(
        '--min-depth',
        type=float,
        default=0.01,
        help='Measurements below this distance in meters are ' + 'ignored.')
    parser.add_argument('--max-depth',
                        type=float,
                        default=10.0,
                        help='Measurements beyond this distance in meters ' +
                        'are ignored.')
    parser.add_argument('--other-odometry-params',
                        type=json.loads,
                        default={},
                        help='Other VisualOdometryParams parameters in JSON ' +
                        'fromat.')
    parser.add_argument(
        '--trajectory-image-path',
        type=str,
        default=None,
        help='Where to store a generated image with baseline ' +
        'and estimated trajectory.')
    parser.add_argument(
        '--show',
        default=False,
        action='store_true',
        help='Enable visualization of the scene and detected ' + 'artifacts.')
    args = parser.parse_args()
    assert (args.logfile)
    assert (args.image_stream)
    assert (args.depth_stream)
    assert (args.max_depth > 0)
    assert (args.min_depth >= 0)
    assert (args.max_depth > args.min_depth)

    odometry_params = VisualOdometryParams(args.camera_focal_length,
                                           args.camera_center,
                                           min_valid_depth=args.min_depth,
                                           max_valid_depth=args.max_depth,
                                           **args.other_odometry_params)
    demo = Demo(args.camera_xyz, np.radians(args.camera_rpy), odometry_params,
                args.show, args.trajectory_image_path)

    image_stream_id = lookup_stream_id(args.logfile, args.image_stream)
    depth_stream_id = lookup_stream_id(args.logfile, args.depth_stream)
    streams = [image_stream_id, depth_stream_id]
    if args.origin_stream:
        origin_stream_id = lookup_stream_id(args.logfile, args.origin_stream)
        streams.append(origin_stream_id)
    else:
        origin_stream_id = None
        demo.on_initial_pose([0., 0., 0.], [0., 0., 0.])
    if args.imu_stream:
        imu_stream_id = lookup_stream_id(args.logfile, args.imu_stream)
        streams.append(imu_stream_id)
    else:
        imu_stream_id = None
    if args.altitude_stream:
        altitude_stream_id = lookup_stream_id(args.logfile,
                                              args.altitude_stream)
        streams.append(altitude_stream_id)
    else:
        altitude_stream_id = None
    if args.baseline_3d_stream:
        baseline_3d_stream_id = lookup_stream_id(args.logfile,
                                                 args.baseline_3d_stream)
        streams.append(baseline_3d_stream_id)
    else:
        baseline_3d_stream_id = None

    with LogReader(args.logfile, only_stream_id=streams) as logreader:
        for time, stream, data in logreader:
            data = deserialize(data)
            now = time.total_seconds()

            # In practice, it probably makes lots of sense to keep
            # re-initializing visual odometry as long as information about
            # origin keeps coming. The initial part, when the robot is in a
            # featureless wide area is difficult for visual odometry anyway.
            # Re-initializing would make odometry really start estimating
            # only once the robot enters competition area, which should,
            # presumably, be visually more diverse.
            if stream == origin_stream_id and demo.odometry is None:
                robot_name, x, y, z, qa, qb, qc, qd = data
                rpy = euler_zyx((qa, qb, qc, qd))[::-1]
                demo.on_initial_pose([x, y, z], rpy)
            elif stream == image_stream_id:
                grey_img = cv2.imdecode(np.frombuffer(data, np.uint8),
                                        cv2.IMREAD_GRAYSCALE)
                color_img = cv2.imdecode(np.frombuffer(data, np.uint8),
                                         cv2.IMREAD_COLOR)
                if not demo.on_img(now, grey_img, color_img):
                    break
            elif stream == depth_stream_id:
                # Converting to meters.
                depth = (data * 1e-3).astype(np.float32)
                if not demo.on_depth(now, depth):
                    break
            elif stream == imu_stream_id:
                quaternion = data
                ypr = euler_zyx(quaternion)
                rpy = ypr[::-1]
                imu_rpy = rpy
                demo.on_imu_rpy(imu_rpy)
            elif stream == altitude_stream_id:
                # TODO: Assuming one float in meters. Needs to be converted if
                #       we get, for example, millimeters instead.
                altitude = data
                demo.on_altitude(altitude)
            elif stream == baseline_3d_stream_id:
                baseline_xyz, baseline_quaternion = data
                baseline_ypr = euler_zyx(baseline_quaternion)
                baseline_rpy = baseline_ypr[::-1]
                demo.on_baseline(baseline_xyz, baseline_rpy)


if __name__ == '__main__':
    main()
