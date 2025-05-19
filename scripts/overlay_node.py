#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import cv2
import message_filters
import tf2_ros
from image_geometry import PinholeCameraModel

class LidarImageOverlay(Node):
    def __init__(self):
        super().__init__('lidar_image_overlay')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.cam_model = PinholeCameraModel()

        # Parameters
        self.declare_parameter('image_topic', '/camera/rgbd/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/rgbd/camera_info')
        self.declare_parameter('points_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/overlay_image')

        img_topic  = self.get_parameter('image_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        pc_topic   = self.get_parameter('points_topic').get_parameter_value().string_value
        out_topic  = self.get_parameter('output_topic').get_parameter_value().string_value

        # Time-synchronized subscribers with larger slop
        img_sub  = message_filters.Subscriber(self, Image,      img_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)
        pc_sub   = message_filters.Subscriber(self, PointCloud2, pc_topic)

        ts = message_filters.ApproximateTimeSynchronizer(
            [img_sub, info_sub, pc_sub],
            queue_size=10,
            slop=0.5
        )
        ts.registerCallback(self.cb)

        # Publisher
        self.pub = self.create_publisher(Image, out_topic, 10)
        self.get_logger().info(f"Publishing overlay on {out_topic}")

    def cb(self, img_msg, info_msg, cloud_msg):
        # 1) Load camera intrinsics and convert to OpenCV image
        self.cam_model.fromCameraInfo(info_msg)
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        # 2) Transform entire PointCloud2 into camera frame
        try:
            cloud_cam = self.tf_buffer.transform(
                cloud_msg,
                self.cam_model.tfFrame()
            )
        except Exception as e:
            self.get_logger().warn(f"PointCloud transform failed: {e}")
            return

        # 3) Draw points (larger filled red circles for visibility)
        count = 0
        for x, y, z in point_cloud2.read_points(
                cloud_cam, field_names=('x','y','z'), skip_nans=True):
            if z <= 0.0:
                continue
            u, v = self.cam_model.project3dToPixel((x, y, z))
            if 0 <= u < cv_img.shape[1] and 0 <= v < cv_img.shape[0]:
                cv2.circle(cv_img, (int(u), int(v)), 5, (0, 0, 255), -1)
                count += 1

        self.get_logger().info(f"Drew {count} points")

        # 4) Publish overlaid image
        out_msg = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
        out_msg.header = img_msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarImageOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
