import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from ament_index_python.packages import get_package_share_directory
import os
import yaml


class VideoPublisher(Node):

    def __init__(self):
        super().__init__('video_publisher')

        #Getting params
        pkg_dir = get_package_share_directory("system_configuration")
        params_path = os.path.join(os.path.join(pkg_dir, 'config', 'video_publisher_params.yaml'))

        with open(params_path, 'r') as params_file:
            params = yaml.safe_load(params_file)
            self.params = params

        self.publisher_ = self.create_publisher(Image, "video", 10)
        self.timer_period = 1.0/30.0 #20 fps
        self.timer = self.create_timer(self.timer_period, self.publish_frame)
        self.bridge = CvBridge()

        self.start_cap(self.params['video_path'])

        if not self.cap.isOpened():
            print("Cannot open video source")
        

    

    def publish_frame(self):

        ret, frame = self.cap.read()

        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
            # self.log("Published frame!")
        else:
            self.log("Could not capture frame")
            self.log("VIDEO FINISHED!, Restarting from beginning")
            self.start_cap(self.params['video_path'])
        
    def start_cap(self, path):
        self.cap = cv.VideoCapture(path)
    
    def log(self, msg):
        self.get_logger().info(f'VIDEO_PUBLISHER | {msg}')




def main(args = None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)

    video_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()