import cv2 as cv
import numpy as np


class ShapeDetector:
    def __init__(self):
        pass


    def detect(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        canny = cv.Canny(gray, 30, 80)
        cv.imshow("canny", canny)
        close_kernel = cv.getStructuringElement(cv.MORPH_RECT, (9, 9))


        # opened = cv.morphologyEx(reverse_canny, cv.MORPH_OPEN, open_kernel, iterations=5)
        closed = cv.morphologyEx(canny, cv.MORPH_CLOSE, close_kernel, iterations=3)
        # opened = cv.morphologyEx(~closed, cv.MORPH_OPEN, (20, 20), iterations=2)
        cv.imshow("closed", ~closed)
        # cv.imshow('opened', opened)
        contours, hierarchy = cv.findContours(~closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # c = list(contours)
        # c = sorted(c, key=lambda x: x.shape[0], reverse=True)
        # for i in range(5):
        #     print(c[i].shape[0], end=" ")
        # print()

        cv.drawContours(image, contours, -1, (0, 0, 255), 2)
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

    ############### NOT USED, GOT RID OF HSV BOUNDS ################
    def detect_shapes_2d(self, image: np.ndarray):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        mask = np.zeros((hsv.shape[0], hsv.shape[1]), np.uint8)
        print(mask.shape)
        for hsv_bound in self.hsv_bounds["bounds"]:
            print(hsv_bound)
            lower_bound = tuple(hsv_bound['lower'])
            upper_bound = tuple(hsv_bound['upper'])
            mask = cv.bitwise_or(mask, cv.inRange(hsv, lower_bound, upper_bound))
        # lower = (0, 0, 0) #lower green parameterize these
        # upper = (180, 255, 238) #upper green #parameterize these
        # mask = cv.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        eroded = cv.erode(mask, kernel, iterations=1)
        opened = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        closed = cv.morphologyEx(opened, cv.MORPH_CLOSE, kernel)
        contours, hierarchy = cv.findContours(closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        cv.imshow("mask", closed)

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


        













        # cv.imshow('static', image)
        # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # # _, thresh_img = cv.threshold(gray, 150, 255, cv.THRESH_BINARY)
        # # cv.imshow("threshold", thresh_img)



        # # cv.imshow("gray", gray)
        # # cv.imshow('hsv', hsv)

        # # _, thresh_img = cv.threshold
        # # cv.imshow("gray", gray)

        # blurred_img = cv.medianBlur(gray, 5)
        # # blurred_img = cv.GaussianBlur(gray, (5, 5), 1.4)
        
        # edges = cv.Canny(blurred_img, 70, 126)
        # cv.imshow("blurred", blurred_img)

        # kernel = np.ones((3, 3), np.uint8)
        # img = cv.GaussianBlur(edges, (5, 5), 1.4)
        # # eroded = cv.erode(edges, kernel, iterations=1)
        # # opening = cv.morphologyEx(edges, cv.MORPH_CLOSE, kernel)
        # cv.imshow("eroded", img)


        # cv.imshow("static", edges)
    


