import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

VIDEO_PATH = "/home/yliu08/workspaces/pennair-application/resource/PennAir 2024 App Dynamic.mp4"

class VideoPublisher(Node):

    def __init__(self):
        super().__init__('video_publisher')

        self.publisher_ = self.create_publisher(Image, "video", 10)
        self.timer_period = 1.0/60.0 #30 fps
        self.timer = self.create_timer(self.timer_period, self.publish_frame)
        self.bridge = CvBridge()

        self.start_cap(VIDEO_PATH)

        if not self.cap.isOpened():
            print("Cannot open video source")
    

    def publish_frame(self):

        ret, frame = self.cap.read()

        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
            self.log("Published frame!")
        else:
            self.log("Could not capture frame")
            self.log("VIDEO FINISHED!, Restarting from beginning")
            self.start_cap(VIDEO_PATH)
        
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