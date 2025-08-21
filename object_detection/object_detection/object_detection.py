#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from object_detection_msgs.msg import DetectedObjects, DetectedSurfaces

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # --- param for topic (default to your wrist rgbd camera) ---
        self.declare_parameter('image_topic', '/wrist_rgbd_depth_sensor/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # --- QoS typical for camera ---
        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        # --- subs/pubs ---
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, qos)
        self.pub_obj  = self.create_publisher(DetectedObjects, '/object_detected', 10)
        self.pub_surf = self.create_publisher(DetectedSurfaces, '/surface_detected', 10)

        # --- state for throttling/logging ---
        self._count = 0
        self._sent_dummy = False
        self._last_img_log_ns = 0
        self._log_interval_ns = int(2e9)  # ~2s

        # discovery timer: tells you if you’re matched to a publisher
        self._topic = image_topic
        self._disc_timer = self.create_timer(1.0, self._print_discovery)

        # startup logs
        self.get_logger().info("object_detection_node is up")
        self.get_logger().info(f"Subscribing to: {image_topic}")
        self.get_logger().info(f"DetectedObjects fields: {DetectedObjects.get_fields_and_field_types()}")
        self.get_logger().info(f"DetectedSurfaces fields: {DetectedSurfaces.get_fields_and_field_types()}")


    def _print_discovery(self):
        infos = self.get_subscriptions_info_by_topic(self._topic)
        self.get_logger().info(f"Discovery: {len(infos)} publisher(s) matched on {self._topic}")


    def image_callback(self, msg: Image) -> None:
        self._count += 1
        height = msg.height
        width = msg.width
        encoding = msg.encoding.lower()
        bpp = msg.step // msg.width if msg.width else 0
        nbytes = len(msg.data)

        # manual throttle to log about every ~2s
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_img_log_ns >= self._log_interval_ns:
            self.get_logger().info(
                f"Image #{self._count}: {msg.width}x{msg.height} enc={msg.encoding} ~bytes={nbytes} bpp={bpp}"
            )
            self._last_img_log_ns = now_ns

        # Publish a one-time dummy to verify topics/wiring
        if not self._sent_dummy:
            self._sent_dummy = True
            self.pub_obj.publish(DetectedObjects())      # default zeros
            self.pub_surf.publish(DetectedSurfaces())    # default zeros
            self.get_logger().info("Published dummy DetectedObjects/DetectedSurfaces")



def main(args=None) -> None:
    rclpy.init()
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
