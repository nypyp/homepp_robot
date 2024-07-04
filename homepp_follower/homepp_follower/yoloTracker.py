# Python
import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from sensor_msgs.msg import Image
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from yolov8_msgs.msg import Detection, DetectionArray
import message_filters
from cv_bridge import CvBridge
import numpy as np

class YOLOTracker(LifecycleNode):
    def __init__(self):
        super().__init__('yolo_tracker')

        self.cv_bridge = CvBridge()
        self.tracked_person_id = None
        self.tracked_person_distance = None
        self.get_logger().set_level(40)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.closest_person_pub = self.create_publisher(PositionMsg, "/yolo_tracker/tracked_person_pos", 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subs
        image_sub = message_filters.Subscriber(self, Image, "/camera/depth/image_rect_raw", qos_profile=image_qos_profile)
        detections_sub = message_filters.Subscriber(self, DetectionArray, "/yolo/tracking", qos_profile=10)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer((image_sub, detections_sub), 10, 0.5)
        self.synchronizer.registerCallback(self.tracking_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        self.destroy_subscription(self.image_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)

        del self.synchronizer
        self.synchronizer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        return TransitionCallbackReturn.SUCCESS

    def calculate_distance(self, detection, depth_image):
        center_x = int(detection.bbox.center.position.x)
        center_y = int(detection.bbox.center.position.y)
        size = min(int(detection.bbox.size.x), int(detection.bbox.size.y))

        min_size = int(size/3)

        depth_object = depth_image[(center_y-min_size):(center_y+min_size), (center_x-min_size):(center_x+min_size)]
        depth_array = depth_object[~np.isnan(depth_object)]

        if len(depth_array) == 0:
            self.get_logger().warn('Empty depth array. All depth values are nan')
            average_distance = 0.0
        else:
            average_distance = np.mean(depth_array)

        return average_distance

    def find_tracked_person(self, detections):
        for detection in detections:
            if detection.class_name == 'person' and detection.id == self.tracked_person_id:
                return detection
        return None

    def find_new_person(self, detections, depth_image):
        closest_detection = None
        min_distance = float('inf')

        for detection in detections:
            if detection.class_name == 'person':
                average_distance = self.calculate_distance(detection, depth_image)
                if 400 < average_distance < 4000 and average_distance < min_distance:
                    min_distance = average_distance
                    closest_detection = detection

        return closest_detection, min_distance if closest_detection else (None, None)

    def tracking_callback(self, img_msg: Image, detections_msg: DetectionArray) -> None:
        depth_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        self.get_logger().info("Depth image converted.")

        tracked_person = self.find_tracked_person(detections_msg.detections)
        if tracked_person is not None:
            tracked_distance = self.calculate_distance(tracked_person, depth_image)
            self.get_logger().info(f"Tracked person found with distance: {tracked_distance}")
        else:
            tracked_person, tracked_distance = self.find_new_person(detections_msg.detections, depth_image)
            self.get_logger().info(f"New person found with distance: {tracked_distance}")

        if tracked_person is not None:
            self.tracked_person_id = tracked_person.id
            self.tracked_person_distance = tracked_distance
            self.get_logger().info(f"Tracking person with ID: {self.tracked_person_id} and distance: {self.tracked_person_distance}")

            # angle_x = (tracked_person.bbox.center.position.x - depth_image.shape[1] / 2) * tracked_distance / 1000
            angle_x = tracked_person.bbox.center.position.x - 320.0
            angle_y = (tracked_person.bbox.center.position.y - depth_image.shape[0] / 2) * tracked_distance / 1000
            self.get_logger().info(f"Calculated angles - X: {angle_x}, Y: {angle_y}")

            person_info = PositionMsg()
            person_info.angle_x = angle_x 
            person_info.angle_y = angle_y 
            person_info.distance = tracked_distance
            self.closest_person_pub.publish(person_info)
            self.get_logger().info("Published person info.")
        else:
            self.tracked_person_id = None
            self.tracked_person_distance = None
            person_info = PositionMsg()
            person_info.angle_x = 0.0 
            person_info.angle_y = 0.0 
            person_info.distance = 0.0
            self.closest_person_pub.publish(person_info)
            self.get_logger().info("No person found for tracking.")


def main():
    rclpy.init()
    node = YOLOTracker()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
