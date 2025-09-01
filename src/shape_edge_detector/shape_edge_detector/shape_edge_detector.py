import rclpy
from rclpy.node import Node
import cv2 as cv
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import yaml
import numpy as np



class ShapeEdgeDetector(Node):

    def __init__(self):
        super().__init__('shape_edge_detector')


        pkg_dir = get_package_share_directory("shape_edge_detector")
        params_file = os.path.join(pkg_dir, "config", "shape_edge_detector_params.yaml")

        with open(params_file, 'r') as params:
            self.params = yaml.safe_load(params)

        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(Image, "/video", self.video_callback, 10)

        # self.log(params_file)
        # self.log(self.params)

        cv.namedWindow("Edge Detection Video", cv.WINDOW_NORMAL)
        cv.resizeWindow("Edge Detection Video", 800, 600)



    def log(self, msg):
        self.get_logger().info(f'SHAPE EDGE DETECTOR | {msg}')
    
    def video_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg)
        edge_detected = self.detect_shapes_2d(cv_img)
        cv.imshow("Edge Detection Video", edge_detected)
        cv.waitKey(1)


    def detect_shapes_2d(self, image: np.ndarray):

        preset = self.params['shape_edge_detector']['preset']
        hsv_bounds = self.params['shape_edge_detector']['bound_presets'][preset]
        # self.log(hsv_bounds)

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        mask = np.zeros((hsv.shape[0], hsv.shape[1]), np.uint8)
        print(mask.shape)
        for bound in hsv_bounds:
            lower_bound = tuple(hsv_bounds[bound]['lower'])
            upper_bound = tuple(hsv_bounds[bound]['upper'])
            mask = cv.bitwise_or(mask, cv.inRange(hsv, lower_bound, upper_bound))
        # lower = (0, 0, 0) #lower green parameterize these
        # upper = (180, 255, 238) #upper green #parameterize these
        # mask = cv.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        eroded = cv.erode(mask, kernel, iterations=1)
        opened = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        closed = cv.morphologyEx(opened, cv.MORPH_CLOSE, kernel)
        contours, hierarchy = cv.findContours(closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # cv.imshow("mask", closed)

        cv.drawContours(image, contours, -1, (0, 0, 255), 2)
        # cv.imshow("mask", eroded)
        print(len(contours))
        for i in contours:
            # print(i.shape)
            M = cv.moments(i)
            # print(M)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv.circle(image, (cx, cy), radius=5, color=(0, 0, 255), thickness=-1)
            cv.putText(
                image,
                text=f"[{cx}, {cy}]",
                org=(cx - 20, cy + 20),
                fontFace = cv.FONT_HERSHEY_SIMPLEX,
                fontScale=0.5,
                color = (0, 0, 255),
                thickness = 2
            )
        
        return image


def main(args = None):
    rclpy.init(args=args)

    shape_edge_detector = ShapeEdgeDetector()
    shape_edge_detector.log("STARTED")
    rclpy.spin(shape_edge_detector)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
