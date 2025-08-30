import cv2 as cv
from shape_detector import ShapeDetector
import json
import argparse

STATIC_PATH = "./resources/PennAir 2024 App Static.png"
# STATIC_PATH = "./resources/PennAir_Dynamic_SS.png"

def main(args):

    print(args.conf_path)
    with open(args.conf_path, 'r') as config:
        hsv_bounds = json.load(config)
        print(hsv_bounds)

    shape_detector = ShapeDetector(hsv_bounds)
    img = cv.imread(STATIC_PATH)

    detected_img = shape_detector.detect_shapes_2d(img)

    try:
        while True:
            cv.imshow("Edge Detection", detected_img)
            cv.waitKey(100)
    except KeyboardInterrupt:
        print('exited')
    finally:
        cv.destroyAllWindows()

    





if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "--conf_path",
        required=True,
        default="./config/conf_green.json",
        help="path for config file (JSON)"
        )
    args = argparser.parse_args()
    main(args)