import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Create publishers for the image and camera info
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        # Initialize OpenCV VideoCapture (0 is typically the default built-in webcam)
        self.cap = cv2.VideoCapture(1)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            
        # Initialize cv_bridge to convert between OpenCV images and ROS Image messages
        self.bridge = CvBridge()
        
        # Set a timer to capture and publish frames at ~30 Hz (0.033 seconds)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Webcam publisher node has been started.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Get current ROS time for message headers
            current_time = self.get_clock().now().to_msg()
            
            # 1. Prepare and publish the Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = current_time
            img_msg.header.frame_id = 'camera_link'
            self.image_pub.publish(img_msg)

            # 2. Prepare and publish the CameraInfo message
            info_msg = CameraInfo()
            info_msg.header.stamp = current_time
            info_msg.header.frame_id = 'camera_link'
            info_msg.height = frame.shape[0]
            info_msg.width = frame.shape[1]
            info_msg.distortion_model = 'plumb_bob'
            
            # Note: The D, K, R, and P matrices are left empty here. 
            # You would populate these using camera calibration data if doing visual odometry/SLAM.
            
            self.camera_info_pub.publish(info_msg)

    def destroy_node(self):
        # Release the webcam before shutting down
        self.cap.release()
        self.get_logger().info("Webcam released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()