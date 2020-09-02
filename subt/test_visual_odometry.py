import unittest
from pathlib import Path

import cv2
import numpy as np

import subt.visual_odometry as vo

curdir = Path(__file__).parent

def odometry_params_for_tests():
    return vo.VisualOdometryParams(
            camera_focal_length=462.1, camera_center=(320.5, 180.5))


def load_test_data():
    bundle = np.load(str(curdir/'test_data/rgbd.npz'))
    return bundle['color_img'], bundle['grey_img'], bundle['depth']


def odom_estimate(odom, observations_sequence):
    for observations in observations_sequence:
        img = observations['img']
        depth = observations.get('depth', np.zeros(img.shape, dtype=np.float32))
        rpy = observations.get('rpy', None)
        rot = None if rpy is None else vo.euler_angles_to_rotation_matrix(*rpy)
        altitude = observations.get('altitude', None)
        odom.update(img, depth, rot, altitude)
    return odom.xyz, odom.rpy


class VisualOdometryTest(unittest.TestCase):
    def test_as_rigid_transform_xyz(self):
        transform = vo.as_rigid_transform(xyz=[1, 2, 3])
        pt = np.asarray([2., 2., 2., 1.])
        transformed_pt = (transform @ pt)[:3]
        self.assertTrue(np.all(transformed_pt == [3, 4, 5]))

    def test_as_rigid_transform_rpy(self):
        transform = vo.as_rigid_transform(rpy=np.radians((0, 0, 90)))
        pt = np.asarray([1., 2., 3., 1.])
        transformed_pt = (transform @ pt)[:3]
        self.assertTrue(np.all(np.isclose(transformed_pt, [-2, 1, 3])))

    def test_as_rigid_transform_rot(self):
        transform = vo.as_rigid_transform(rot=
                vo.euler_angles_to_rotation_matrix(*np.radians((0, 0, 90))))
        pt = np.asarray([1., 2., 3., 1.])
        transformed_pt = (transform @ pt)[:3]
        self.assertTrue(np.all(np.isclose(transformed_pt, [-2, 1, 3])))

    def test_inverse_rotation(self):
        # Some not very specific rotation.
        rot = vo.euler_angles_to_rotation_matrix(0.13, 0.24, 0.42)
        inv_rot = vo.inverse_rotation(rot)
        self.assertTrue(np.all(np.isclose(inv_rot @ rot, np.eye(3))))

    def test_inverse_rigid_transform(self):
        rt = vo.as_rigid_transform(xyz=[1, 2, 3], rpy=[0.13, 0.24, 0.42])
        irt = vo.inverse_rigid_transform(rt)
        self.assertTrue(np.all(np.isclose(irt @ rt, np.eye(4))))

    def test_euler_to_rot_and_back(self):
        orig_rpy = (-np.pi / 3, np.pi / 4., np.pi / 2)
        rot = vo.euler_angles_to_rotation_matrix(*orig_rpy)
        back_rpy = vo.rotation_matrix_to_euler_angles(rot)
        self.assertEqual(back_rpy, orig_rpy)

    def test_near_rotation_matrix(self):
        orig_rot = np.eye(3)
        messed_up_rot = orig_rot * (1 + 1e-3)
        messed_up_rot[0, 2] = 1e-12
        recovered_rot = vo.near_rotation_matrix(messed_up_rot)

        # The disturbance we caused is noticeable ...
        self.assertFalse(np.all(np.isclose(messed_up_rot, orig_rot)))
        # ... and we are able to recover from it.
        self.assertTrue(np.all(np.isclose(recovered_rot, orig_rot)))

    def test_camera_odometry_no_move(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape
        odom = vo.CameraVisualOdometry(odometry_params_for_tests())
        # If the scene hasn't changed, the estimate should say that
        # the camera has not moved.
        xyz, rpy = odom_estimate(
                odom,
                [{'img' : grey_img, 'depth' : depth},
                 {'img' : grey_img, 'depth' : depth}])
        self.assertEqual(xyz.tolist(), [0, 0, 0])
        self.assertEqual(rpy, (0, 0, 0))

    def test_camera_odometry_no_move_less_obvious(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape
        odom = vo.CameraVisualOdometry(odometry_params_for_tests())

        # Next scene, slightly modified so that it is not obvious that it is
        # the same one.
        next_grey_img = grey_img.copy()
        next_grey_img[:64,:] = 0
        next_depth = depth.copy()
        next_depth[:64,:] = 0

        # If the scene hasn't changed, the estimate should say that
        # the camera has not moved.
        xyz, rpy = odom_estimate(
                odom,
                [{'img' : grey_img, 'depth' : depth},
                 {'img' : next_grey_img, 'depth' : next_depth}])
        self.assertLess(np.linalg.norm(xyz), 1e-3)
        self.assertLess(np.linalg.norm(rpy), np.radians(0.01))

    def test_camera_odometry_forward(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape

        # Scaling the image up roughly corresponds to moving directly forward
        # from the point of view of the camera. We need to keep the 16:9 image
        # ratio of the test input during the resize.
        self.assertEqual(grey_img.shape, (360, 640))
        fw_img = cv2.resize(grey_img[9:-9,16:-16], (w, h))
        self.assertEqual(grey_img.shape, fw_img.shape)

        odom = vo.CameraVisualOdometry(odometry_params_for_tests())
        xyz, rpy = odom_estimate(
                odom, [{'img' : grey_img, 'depth' : depth}, {'img' : fw_img}])
        # The estimate should lead to a substantial move in the direction of x
        # axis, i.e. bigger move in that direction than other directions.
        # There also shouldn't be a big change in heading.
        self.assertTrue(xyz[0] > 0)
        A_LOT = 4
        self.assertGreater(xyz[0], A_LOT * abs(xyz[1]))
        self.assertGreater(xyz[0], A_LOT * abs(xyz[2]))
        NOT_MUCH = np.radians(2)
        self.assertTrue(np.all(np.abs(rpy) < NOT_MUCH))

    def test_camera_odometry_rot_left(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape

        # Shifting the image to the right mimics rotation to the left on the
        # spot.
        rot_img = np.zeros_like(grey_img)
        rot_img[:,16:] = grey_img[:,:-16]
        odom = vo.CameraVisualOdometry(odometry_params_for_tests())
        xyz, rpy = odom_estimate(
                odom, [{'img' : grey_img, 'depth' : depth}, {'img' : rot_img}])
        # The expected translation should be small.
        NOT_MUCH = 0.05
        self.assertLess(np.linalg.norm(xyz), NOT_MUCH)
        # The rotation should be bigger than the other rotations.
        A_LOT = 10
        self.assertGreater(rpy[2], A_LOT * rpy[0])
        self.assertGreater(rpy[2], A_LOT * rpy[1])

    def test_camera_odometry_imu_correction(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape

        # Scaling the image up roughly corresponds to moving directly forward
        # from the point of view of the camera. We need to keep the 16:9 image
        # ratio of the test input during the resize.
        self.assertEqual(grey_img.shape, (360, 640))
        fw_img = cv2.resize(grey_img[9:-9,16:-16], (w, h))

        odom = vo.CameraVisualOdometry(odometry_params_for_tests())
        xyz, rpy = odom_estimate(
                odom,
                [{'img' : grey_img, 'depth' : depth},
                 {'img' : fw_img, 'rpy' : (0, 0, 0)}])
        # The estimate should lead to a substantial move in the direction of x
        # axis, i.e. bigger move in that direction than other directions.
        # There also shouldn't be almost any change in heading, because we
        # provided information about heading.
        self.assertTrue(xyz[0] > 0)
        A_LOT = 4
        self.assertGreater(xyz[0], A_LOT * abs(xyz[1]))
        self.assertGreater(xyz[0], A_LOT * abs(xyz[2]))
        NOT_MUCH = np.radians(1e-6)
        self.assertTrue(np.all(np.abs(rpy) < NOT_MUCH))

    def test_camera_odometry_altitude_correction(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape
        # Pretending forward motion.
        fw_img = cv2.resize(grey_img[9:-9,16:-16], (w, h))
        odom = vo.CameraVisualOdometry(odometry_params_for_tests())
        self.assertEqual(odom.xyz[2], 0.)
        xyz, rpy = odom_estimate(
                odom,
                [{'img' : grey_img, 'depth' : depth},
                 {'img' : fw_img, 'altitude' : 0.25}])
        # Despite odometry starting at (0, 0, 0) and estimating pure forward
        # motion, the altitude mesurement should pust the estimate up.
        self.assertEqual(xyz[2], 0.25)


    def test_robot_odometry_forward(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape

        # Scaling the image up roughly corresponds to moving directly forward
        # from the point of view of the camera. We need to keep the 16:9 image
        # ratio of the test input during the resize.
        self.assertEqual(grey_img.shape, (360, 640))
        fw_img = cv2.resize(grey_img[9:-9,16:-16], (w, h))

        odom = vo.RobotVisualOdometry(
                camera_xyz=(0.2, 0.2, 0.2),
                # Camera points to the left.
                camera_rpy=(0, 0, np.pi/2),
                odometry_params=odometry_params_for_tests())
        xyz, rpy = odom_estimate(
                odom, [{'img' : grey_img, 'depth' : depth}, {'img' : fw_img}])

        # The estimate should lead to a substantial move in the direction of y
        # axis, because camera looks to the left and the move was forward from
        # its perspective. I.e. the should be a bigger move in y direction
        # than other directions.
        # There also shouldn't be a big change in heading.
        self.assertTrue(xyz[1] > 0)
        A_LOT = 4
        self.assertGreater(xyz[1], A_LOT * abs(xyz[0]))
        self.assertGreater(xyz[1], A_LOT * abs(xyz[2]))
        NOT_MUCH = np.radians(2)
        self.assertTrue(np.all(np.abs(rpy) < NOT_MUCH))

    def test_global_odometry_forward(self):
        color_img, grey_img, depth = load_test_data()
        h, w = grey_img.shape

        # Scaling the image up roughly corresponds to moving directly forward
        # from the point of view of the camera. We need to keep the 16:9 image
        # ratio of the test input during the resize.
        self.assertEqual(grey_img.shape, (360, 640))
        fw_img = cv2.resize(grey_img[9:-9,16:-16], (w, h))

        odom = vo.GlobalVisualOdometry(
                initial_xyz=(0, 0, 0),
                # Robot starts looking "backwards".
                initial_rpy=(0, 0, np.pi),
                camera_xyz=(0.2, 0.2, 0.2),
                # Camera points to the left of the robot.
                camera_rpy=(0, 0, np.pi/2),
                odometry_params=odometry_params_for_tests())
        xyz, rpy = odom_estimate(
                odom, [{'img' : grey_img, 'depth' : depth}, {'img' : fw_img}])

        # The estimate should lead to a substantial move in the direction of
        # negative y axis, because camera looks to the left of the robot and
        # robot starts at a position looking in the direction of negative X
        # axis and the move was forward from the perspective of the camera.
        # I.e. the should be a bigger move in -y direction than other
        # directions.
        # There also shouldn't be a big change in heading.
        self.assertTrue(xyz[1] < 0)
        A_LOT = 4
        self.assertGreater(-xyz[1], A_LOT * abs(xyz[0]))
        self.assertGreater(-xyz[1], A_LOT * abs(xyz[2]))
        NOT_MUCH = np.radians(2)
        expected_rpy = np.array([0, 0, np.pi])
        self.assertTrue(np.all(np.abs(rpy - expected_rpy) < NOT_MUCH))


if __name__ == '__main__':
    unittest.main()
