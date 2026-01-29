import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from message_filters import Subscriber, ApproximateTimeSynchronizer


class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node_3')

        self.bridge = CvBridge()
        self.K = None

        self.T_world_cam = np.eye(4)

        self.prev_image = None
        self.prev_depth = None
        self.prev_keypoints = None

        # Camera info
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)

        # --- RGB-D Synchronization ---
        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        
        self.depth_sub = Subscriber( self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05)
        
        self.sync.registerCallback(self.rgbd_callback)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)

        # Service
        self.create_service(Trigger, 'reset_odom', self.reset_callback)

    # -------------------------------------------------

    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)

    # -------------------------------------------------

    def rgbd_callback(self, img_msg, depth_msg):
        if self.K is None:
            return

        curr_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        curr_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        curr_gray = cv2.cvtColor(curr_image, cv2.COLOR_BGR2GRAY)

        if self.prev_image is None:
            self.prev_image = curr_gray
            self.prev_depth = curr_depth
            self.prev_keypoints = cv2.goodFeaturesToTrack(
                curr_gray, 2000, 0.01, 10
            )
            return

        self.process_frame(curr_gray)

        self.prev_image = curr_gray
        self.prev_depth = curr_depth

    # -------------------------------------------------

    def process_frame(self, curr_gray):

        curr_kp, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_image,
            curr_gray,
            self.prev_keypoints,
            None
        )

        status = status.flatten()
        good_prev = self.prev_keypoints[status == 1].reshape(-1, 2)
        good_curr = curr_kp[status == 1].reshape(-1, 2)

        obj_pts, img_pts = self.get_3d_2d_pairs(
            good_prev, good_curr, self.prev_depth
        )

        if len(obj_pts) < 6:
            self.prev_keypoints = cv2.goodFeaturesToTrack(
                curr_gray, 2000, 0.01, 10
            )
            return

        _, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts,
            img_pts,
            self.K,
            None,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=3.0,
            iterationsCount=100
        )

        self.publish_odometry(rvec, tvec)

        if inliers is None or len(inliers) < 500:
            self.prev_keypoints = cv2.goodFeaturesToTrack(
                curr_gray, 2000, 0.01, 10
            )
        else:
            self.prev_keypoints = img_pts[inliers.flatten()].reshape(-1, 1, 2)

    # -------------------------------------------------

    def get_3d_2d_pairs(self, kp_prev, kp_curr, depth):

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        obj_pts = []
        img_pts = []

        for i, (u, v) in enumerate(kp_prev.astype(int)):
            if 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]:
                z = depth[v, u] * 0.001
                if 0.1 < z < 5.0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    obj_pts.append([x, y, z])
                    img_pts.append(kp_curr[i])

        return np.array(obj_pts, np.float32), np.array(img_pts, np.float32)

    # -------------------------------------------------

    def update_global_pose(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()

        self.T_world_cam = self.T_world_cam @ np.linalg.inv(T)

        return self.T_world_cam

    # -------------------------------------------------

    def rotation_matrix_to_quaternion(self, R):
        # returns x, y, z, w
        q = np.zeros(4)
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            q[3] = 0.25 * S
            q[0] = (R[2,1] - R[1,2]) / S
            q[1] = (R[0,2] - R[2,0]) / S
            q[2] = (R[1,0] - R[0,1]) / S
        return q

    # -------------------------------------------------

    def publish_odometry(self, rvec, tvec):

        T = self.update_global_pose(rvec, tvec)
        R = T[:3, :3]
        t = T[:3, 3]

        q = self.rotation_matrix_to_quaternion(R)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Camera → ROS
        odom.pose.pose.position.x = t[2]
        odom.pose.pose.position.y = -t[0]
        odom.pose.pose.position.z = -t[1]

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

    # -------------------------------------------------

    def reset_callback(self, req, res):
        self.T_world_cam = np.eye(4)
        res.success = True
        res.message = "VO reset"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
