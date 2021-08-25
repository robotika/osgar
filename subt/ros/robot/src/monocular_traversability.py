#!/usr/bin/python

import cv2
import numpy as np

import rospy
import tf.transformations
import tf2_ros

from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField


class MonoTraversability:
    def __init__(self):
        self.prev_img = None
        self.prev_pose = None
        self.camera_info = None

        self.world_frame_id = rospy.get_param('world_frame_id', 'odom')
        self.min_baseline = rospy.get_param('min_baseline', 0.08)
        self.max_baseline = rospy.get_param('max_baseline', 0.25)
        self.max_corners = rospy.get_param('max_corners', 800)
        self.corners_quality = rospy.get_param('corners_quality', 30)
        self.min_tracking_confidence = rospy.get_param('min_tracking_confidence', 0.22)
        self.min_mesh_edge_length = rospy.get_param('min_mesh_edge_length', 0.1)
        self.max_mesh_edge_length = rospy.get_param('max_mesh_edge_length', 0.4)
        self.min_mesh_height = rospy.get_param('min_mesh_height', 0.07)
        self.horizontality_threshold = np.cos(np.radians(rospy.get_param('max_slope', 35)))
        self.max_height_above_sensor = rospy.get_param('max_height_above_sensor', 0.7)
        self.debug = rospy.get_param('debug', False)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.image_subscriber = rospy.Subscriber('input/camera', CompressedImage, self.imageCallback)
        self.camera_info_subscriber = rospy.Subscriber('input/camera_info', CameraInfo, self.cameraInfoCallback)
        self.publisher = rospy.Publisher('output/points', PointCloud2, queue_size=5)

        self.feature_detector = cv2.FastFeatureDetector_create(self.corners_quality)

    def cameraInfoCallback(self, msg):
        self.camera_info = msg

    def imageCallback(self, msg):
        curr_img = cv2.imdecode(np.fromstring(msg.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        try:
            trans = self.tf_buffer.lookup_transform(self.world_frame_id, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('tf world-to-cam error ({} -> {}): {}'.format(self.world_frame_id, msg.header.frame_id, e))
            return

        curr_trans = (trans.transform.translation.x,
                      trans.transform.translation.y,
                      trans.transform.translation.z)
        curr_rot = (trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w)
        curr_pose = np.asmatrix(tf.transformations.translation_matrix(curr_trans)) * np.asmatrix(tf.transformations.quaternion_matrix(curr_rot))

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 0
        cloud.width = 0
        cloud.row_step = 0
        cloud.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1)]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.is_dense = False
        obstacles = []

        if self.prev_img is not None and self.camera_info is not None:
            baseline = np.linalg.norm(
                    tf.transformations.translation_from_matrix(curr_pose) -
                    tf.transformations.translation_from_matrix(self.prev_pose))
            if baseline < self.min_baseline:
                self.publisher.publish(cloud)
                return
            if baseline <= self.max_baseline:
                kps = self.feature_detector.detect(curr_img)
                if len(kps) > self.max_corners:
                    kps.sort(key = lambda kp: kp.response)
                    kps = kps[:self.max_corners]
                curr_corners = np.asarray([kp.pt for kp in kps], dtype=np.float32)
                if self.debug:
                    shown = cv2.cvtColor(curr_img, cv2.COLOR_GRAY2BGR)
                    for corner in curr_corners.astype(int):
                        cv2.circle(shown, tuple(corner), 3, (0, 0, 0xFF))
                if curr_corners.shape[0] > 0:
                    prev_corners, status, err = cv2.calcOpticalFlowPyrLK(curr_img, self.prev_img, curr_corners, None)
                    status = status[:,0].astype(np.bool)
                    status = np.logical_and(status, err[:,0] < 1.0 / self.min_tracking_confidence)
                    num_tracked = np.sum(status)
                    if num_tracked >= 3:  # We need at least three points to detect a plane.
                        #relevalt_err = err[status]
                        #rospy.logerr('err {} to {}'.format(np.min(relevalt_err), np.max(relevalt_err)))
                        curr_corners = curr_corners[status]
                        prev_corners = prev_corners[status]

                        if self.debug:
                            for corner in curr_corners.astype(int):
                                cv2.circle(shown, tuple(corner), 3, (0, 0xFF, 0))

                        TO_OPTICAL = np.matrix([[ 0, -1,  0, 0],
                                               [ 0,  0, -1, 0],
                                               [ 1,  0,  0, 0],
                                               [ 0,  0,  0, 1]], dtype=np.float)
                        FROM_OPTICAL = TO_OPTICAL.T  # inverse
                        # projection_matrix = camera_matrix @ camera_pose
                        # https://stackoverflow.com/questions/16101747/how-can-i-get-the-camera-projection-matrix-out-of-calibratecamera-return-value
                        # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
                        # We calculate everything in the previous coordinate frame of the camera.
                        camera_matrix = np.asmatrix(np.asarray(self.camera_info.K).reshape((3, 3)))
                        curr_projection_matrix = camera_matrix * np.asmatrix(np.eye(3, 4))
                        to_prev_camera = np.linalg.inv(self.prev_pose) * curr_pose
                        prev_projection_matrix = camera_matrix * (TO_OPTICAL * to_prev_camera * FROM_OPTICAL)[:3,:]
                        points3d = cv2.triangulatePoints(prev_projection_matrix.astype(np.float64), curr_projection_matrix.astype(np.float64), prev_corners.T.astype(np.float64), curr_corners.T.astype(np.float64)).T
                        points3d = cv2.convertPointsFromHomogeneous(points3d)
                        points3d = points3d[:,0,:]  # Getting rid of the unnecessary extra dimension in the middle.
                        points3d = np.asarray(FROM_OPTICAL[:3,:3] * np.asmatrix(points3d.T)).T

                        coordinates = dict(zip((tuple(uv) for uv in curr_corners), points3d))

                        # Let's create a mesh of triangles.
                        subdiv = cv2.Subdiv2D((0, 0, curr_img.shape[1], curr_img.shape[0]))
                        subdiv.insert(curr_corners.reshape((-1, 1, 2)))
                        mesh = subdiv.getTriangleList()
                        synthetic_corners = set([(0, 0), (0, curr_img.shape[0]), (curr_img.shape[1], 0), (curr_img.shape[1], curr_img.shape[0])])
                        for ax, ay, bx, by, cx, cy in mesh:
                            pt_a = ax, ay
                            pt_b = bx, by
                            pt_c = cx, cy
                            if self.debug:
                                A = tuple(int(v) for v in pt_a)
                                B = tuple(int(v) for v in pt_b)
                                C = tuple(int(v) for v in pt_c)
                                red = 0, 0, 0xFF
                                cv2.line(shown, A, B, red)
                                cv2.line(shown, A, C, red)
                                cv2.line(shown, B, C, red)
                            try:
                                xyz_a = coordinates[pt_a]
                                xyz_b = coordinates[pt_b]
                                xyz_c = coordinates[pt_c]
                                ab = xyz_b - xyz_a
                                ac = xyz_c - xyz_a
                                bc = xyz_c - xyz_b
                                ab_length = np.linalg.norm(ab)
                                ac_length = np.linalg.norm(ac)
                                bc_length = np.linalg.norm(bc)
                                if (ab_length > self.max_mesh_edge_length or
                                    ac_length > self.max_mesh_edge_length or
                                    bc_length > self.max_mesh_edge_length or
                                    ab_length < self.min_mesh_edge_length or
                                    ac_length < self.min_mesh_edge_length or
                                    bc_length < self.min_mesh_edge_length or
                                    xyz_a[2] > self.max_height_above_sensor or
                                    xyz_b[2] > self.max_height_above_sensor or
                                    xyz_c[2] > self.max_height_above_sensor or
                                    np.max([xyz_a[2], xyz_b[2], xyz_c[2]]) -
                                      np.min([xyz_a[2], xyz_b[2], xyz_c[2]]) < self.min_mesh_height or
                                    # Negative depth is a clearly incorrect estimate.
                                    xyz_a[0] <= 0 or
                                    xyz_b[0] <= 0 or
                                    xyz_c[0] <= 0):
                                    continue
                                normal = np.cross(ab, ac)
                                horizontality = abs(normal[2] / np.linalg.norm(normal))
                                if horizontality < self.horizontality_threshold:
                                    obstacles.append(xyz_a)
                                    obstacles.append(xyz_b)
                                    obstacles.append(xyz_c)
                                    if self.debug:
                                        green = (0, 0xFF, 0)
                                        cv2.line(shown, A, B, green)
                                        cv2.line(shown, A, C, green)
                                        cv2.line(shown, B, C, green)
                            except KeyError:
                                # We encountered one of subdiv's synthetic corners.
                                pass

                if self.debug:
                    cv2.imshow('pts', shown)
                    cv2.waitKey(1)

        if obstacles:
            obstacles = np.unique(obstacles, axis=0)
            cloud.width = obstacles.shape[0]
            cloud.height = 1
            cloud.data = obstacles.astype(np.float32).tobytes()
            cloud.row_step = 12 * obstacles.shape[0]
        rospy.logdebug('num obstacles: {}'.format(len(obstacles)))
        self.publisher.publish(cloud)


        self.prev_img = curr_img
        self.prev_pose = curr_pose

if __name__ == "__main__":
    rospy.init_node("~mono_traversability", log_level=rospy.DEBUG)
    overrider = MonoTraversability()
    rospy.spin()
