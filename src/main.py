import cv2 as cv
from shape_detector import ShapeDetector
import json
import argparse

# STATIC_IMG = "./resource/PennAir 2024 App Static.png"
STATIC_IMG = "./resource/dynamic_hard.png"
video = "./resource/PennAir 2024 App Dynamic Hard.mp4"
# video = "./resource/PennAir 2024 App Dynamic.mp4"
# STATIC_IMG = "../resource/solid_bckground.png"

def main():


    shape_detector = ShapeDetector()
    # static = cv.imread(STATIC_IMG)
    cap = cv.VideoCapture(video)
    cv.namedWindow("detected", cv.WINDOW_NORMAL)
    cv.resizeWindow("detected", 1280, 720)

    cv.namedWindow("closed", cv.WINDOW_NORMAL)
    cv.resizeWindow("closed", 1280, 720)

    try:
        while True:
            # ret, img = cap.read()
            img = cv.imread(STATIC_IMG)
            detected = shape_detector.detect(img)
            cv.imshow("detected", detected)
            cv.waitKey(100)
    except KeyboardInterrupt:
        print('exited')
    finally:
        cv.destroyAllWindows()

    





if __name__ == "__main__":
    main()