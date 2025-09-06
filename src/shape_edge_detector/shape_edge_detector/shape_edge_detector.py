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

        # Loading Parameter File
        pkg_dir = get_package_share_directory("system_configuration")
        params_file = os.path.join(pkg_dir, "config", "shape_edge_detector_params.yaml")
        with open(params_file, 'r') as params:
            self.params = yaml.safe_load(params)

        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(Image, "/video", self.video_callback, 10)

        self.K = [[2564.3186869,    0,             0],         #fx, s, x0     INTRINSIC CAMERA PARAMETERS
                  [0,               2569.70273111, 0],         #0   fy, y0
                  [0,               0,             1]]         #0,  0,  1
        
        self.pixels_per_inch = 10.46 #calculated from 10 in radius
        self.Z = None


        cv.namedWindow("Edge Detection Video", cv.WINDOW_NORMAL)
        cv.resizeWindow("Edge Detection Video", 1280, 720)


    def log(self, msg):
        self.get_logger().info(f'SHAPE EDGE DETECTOR | {msg}')
    
    def video_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg)

        if self.Z is None:
            self.Z = self.calculate_depth()
        edge_detected = self.detect_shapes(cv_img)
        cv.imshow("Edge Detection Video", edge_detected)
        cv.waitKey(1)


    def detect_shapes(self, image: np.ndarray):
        """Use custom cv algorithm to extract shape edges from background of input image

        Args:
            image (np.ndarray): input image/frame

        Returns:
            np.ndarray : original image but with drawn on contours and real world 3D coordinates
        """        

        # Get necessary params
        params = self.params['shape_edge_detector']
        CLOSE_KERNEL_PIXEL_DIM = params['close_kernel_pixel_dim']
        MORPH_CLOSE_ITERATIONS = params['morph_close_iterations']
        CONTOUR_PIXEL_THRESHOLD = params['contour_pixel_threshold']

        #main algorithm
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        canny = cv.Canny(gray, params['canny_lower'], params['canny_upper'])
        close_kernel = cv.getStructuringElement(cv.MORPH_RECT, (CLOSE_KERNEL_PIXEL_DIM, CLOSE_KERNEL_PIXEL_DIM))
        reverse_closed = ~cv.morphologyEx(canny, cv.MORPH_CLOSE, close_kernel, iterations=MORPH_CLOSE_ITERATIONS)

        contours, hierarchy = cv.findContours(reverse_closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # dims = []
        for i in contours:
            if i.shape[0] < CONTOUR_PIXEL_THRESHOLD:
                continue
            # dims.append(i.shape[0])
            cv.drawContours(image, [i], -1, (0, 255, 100), 2)
            M = cv.moments(i)
            if M['m00'] == 0:
                continue
            
            #Draw center circle and coordinates text
            center_x = int(M['m10'] / M['m00'])
            center_y = int(M['m01'] / M['m00'])
            cv.circle(image, (center_x, center_y), radius=5, color=(0, 255, 100), thickness=-1)
            coords_inches = self.get_3D_coords(center_x, center_y)
            cv.putText(
                image,
                text=f"[Depth: {coords_inches[2]} X:{coords_inches[0]}, Y:{coords_inches[1]}]",
                org=(center_x - 20, center_y + 20),
                fontFace = cv.FONT_HERSHEY_SIMPLEX,
                fontScale=0.5,
                color = (0, 255, 100),
                thickness = 2
            )
        # self.log(sorted(dims)) debugging
        return image


        """         """
    def calculate_depth(self):
        """Calculates depth of image (inches) based on focal length and pixel to inch ratio

        Returns:
            float: Average of the depths calculated from fx and fy
        """
        f_x = self.K[0][0]
        f_y = self.K[1][1]
        # real_width = self.width / self.pixels_per_inch #inches
        Z_x = f_x / self.pixels_per_inch
        Z_y = f_y / self.pixels_per_inch

        return (Z_x + Z_y) / 2 #average

    def get_3D_coords(self, x, y): #assuming cx, cy = 0
        """Generates 3D real world coordinates using calculated depth and pinhole camera equations

        Args:
            x (int): images' x coordinate in pixels
            y (int): image's y coordinate in pixels 

        Returns:
            tuple(double, double, double): Real world X, Y, Z coordinates in inches. Truncated to 2 decimals
        """
        f_x = self.K[0][0] #focal length x
        f_y = self.K[1][1] #focal length y
        X = self.Z * (x/f_x)
        Y = self.Z * (y/f_y)

        return (truncate_decimals(X), truncate_decimals(Y), truncate_decimals(self.Z))

def truncate_decimals(num):
    return int(num * 100) / 100.0

def main(args = None):
    rclpy.init(args=args)

    shape_edge_detector = ShapeEdgeDetector()
    shape_edge_detector.log("STARTED")
    rclpy.spin(shape_edge_detector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


    # *********HSV FILTERING --> too hardcoded ************
    # hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # mask = np.zeros((hsv.shape[0], hsv.shape[1]), np.uint8)
    # print(mask.shape)
    # for bound in hsv_bounds:
    #     lower_bound = tuple(hsv_bounds[bound]['lower'])
    #     upper_bound = tuple(hsv_bounds[bound]['upper'])
    #     mask = cv.bitwise_or(mask, cv.inRange(hsv, lower_bound, upper_bound))
    # lower = (0, 0, 0) #lower green parameterize these
    # upper = (180, 255, 238) #upper green #parameterize these
    # mask = cv.inRange(hsv, lower, upper)

    # **** Used to calculate pixel radius of circle, in order to get the inch to pixel conversion rate (using 'test' bound preset)
    #AVG_RADIUS ~ 104.6 
    #rate ~ 10.46 pixels/inch
    # radius_total = 0 
    # i = 0
    # center, rad = cv.minEnclosingCircle(contours[0])
    # radius_total += rad
    # i += 1
    # self.log(f"AVERAGE RADIUS: {radius_total / i}")
        
    # cv.imshow("mask", closed)
