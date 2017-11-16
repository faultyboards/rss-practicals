import copy
import glob
import time

import cv2
import numpy as np

wall_lower_range = np.array([0, 50, 0], dtype=np.uint8)
wall_upper_range = np.array([60, 200, 255], dtype=np.uint8)

#poi_color_lower = np.array([0, 0, 225], dtype=np.uint8)
#poi_color_upper = np.array([0, 0, 255], dtype=np.uint8)


poi_color_lower = np.array([49, 17, 194], dtype=np.uint8)
poi_color_upper = np.array([137, 133, 255], dtype=np.uint8)


class Vision:
    def __init__(self, IO):
        self.IO = IO
        self._poi_position = None
        self.IO.cameraSetResolution("low")
        # fixed
        width = 120
        height = 160
        self.center_image = ((160) / 2, 120 / 2)

    def poi_present(self):
        return self._poi_position is not None

    def get_centerline_distance(self):
        pass

    def grab_image(self, n=5):
        for i in range(n):
            self.IO.cameraGrab()

        return self.IO.cameraRead()

    def get_poi_location(self, frame=None, n=5, debug=False):
        if frame is None:
            frame = self.grab_image(n)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        start = time.time()
        wall_img = cv2.inRange(hsv, wall_lower_range, wall_upper_range)
        wall_img = cv2.GaussianBlur(wall_img, (11, 1), 1, 1, wall_img.shape[0])
        if debug:
            print("Wall detection took: {}".format(time.time() - start))

        wall_img_filtered = copy.deepcopy(wall_img)

        start = time.time()
        frame[wall_img_filtered > 150] = 0
        if debug:
            print("Wall removal took: {}".format(time.time() - start))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        poi_img = cv2.inRange(hsv, poi_color_lower, poi_color_upper)
        start = time.time()
        poi_img_canny = cv2.Canny(poi_img, 10, 10)
        _, contours0, hierarchy = cv2.findContours(poi_img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if debug:
            print("Canny detection took: {}".format(time.time() - start))

        max_area = 0
        max_bound_rect = None
        max_contours_id = None
        areas = []

        start = time.time()
        for i, cnt in enumerate(contours0):
            approx_cnt = cv2.approxPolyDP(cnt, 3, True)
            bound_rect = cv2.boundingRect(approx_cnt)
            cnt_area = bound_rect[2] * bound_rect[3]
            areas.append(cnt_area)

            if cnt_area > max_area:
                max_area = cnt_area
                max_bound_rect = bound_rect
                max_contours_id = i

        if debug:
            print("contours localization took: {}".format(time.time() - start))

        if not max_bound_rect:
            return None, None

        mean_area = np.mean(areas)

        if np.abs(max_area - mean_area) > 0.5:
            # return the centre of the POI in the camera frame
            moments = cv2.moments(max_bound_rect)

            cx = float(moments["m10"] / moments["m00"])
            cy = float(moments["m01"] / moments["m00"])
            self._poi_position = (cx, cy)

            poi_contours = np.zeros_like(poi_img_canny)
            cv2.drawContours(poi_contours, contours0, max_contours_id, (150, 0, 255))

            cv2.rectangle(poi_contours, (max_bound_rect[0], max_bound_rect[1]),
                          (max_bound_rect[0] + max_bound_rect[2], max_bound_rect[1] + max_bound_rect[3]),
                          (150, 0, 255), 2)

            

            self.IO.imshow('frmaasdasdas', poi_contours)
            self.IO.imshow('frm', frame)
            return self._poi_position, moments

        return None, None


def compute_error_matrix(m, central_value_method="mean"):
    # compute mean value
    central_value = []
    if central_value_method == "mean":
        for i in range(m.shape[0]):
            central_value.append(np.mean(m[i]))

    elif central_value_method == "median":
        for i in range(m.shape[0]):
            central_value.append(np.median(m[i]))
    else:
        raise ValueError("Illegal argument for central_value parameter: {}".format(central_value_method))

    # Remove the median value of a row from each element in it
    m_centered = np.where(m > 148, np.sqrt(np.square(m - np.expand_dims(central_value, axis=1))), np.zeros_like(m))

    return m_centered


data = {
    "h_lower": 90,
    "s_lower": 71,
    "v_lower": 235,
    "h_upper": 110,
    "s_upper": 81,
    "v_upper": 255
}


def vision_test2():
    frame = cv2.imread("img/poi_1.png")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    start = time.time()
    wall_img = cv2.inRange(copy.deepcopy(hsv), wall_lower_range, wall_upper_range)
    wall_img = cv2.GaussianBlur(wall_img, (11, 1), 1, 1, wall_img.shape[0])
    print("Wall detection took: {}".format(time.time() - start))

    wall_img_filtered = copy.deepcopy(wall_img)

    start = time.time()
    frame[wall_img_filtered > 150] = 0
    print("Wall removal took: {}".format(time.time() - start))

    # cv2.imshow("hsv", frame)

    poi_img = cv2.inRange(hsv, poi_color_lower, poi_color_upper)
    # cv2.imshow("poi_img", poi_img)

    start = time.time()
    poi_img_canny = cv2.Canny(poi_img, 10, 10)
    _, contours0, hierarchy = cv2.findContours(poi_img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print("Canny detection took: {}".format(time.time() - start))

    max_area = 0
    max_bound_rect = None
    max_contours_id = None
    areas = []

    start = time.time()
    for i, cnt in enumerate(contours0):
        approx_cnt = cv2.approxPolyDP(cnt, 3, True)
        bound_rect = cv2.boundingRect(approx_cnt)
        cnt_area = bound_rect[2] * bound_rect[3]
        areas.append(cnt_area)

        if cnt_area > max_area:
            max_area = cnt_area
            max_bound_rect = bound_rect
            max_contours_id = i

    print("contours localization took: {}".format(time.time() - start))

    if not max_bound_rect:
        return None

    mean_area = np.mean(areas)

    if np.abs(max_area - mean_area) > 0.5:
        moments = cv2.moments(max_bound_rect)

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        poi_position = (cx, cy)

        poi_contours = np.zeros_like(poi_img_canny)
        cv2.drawContours(poi_contours, contours0, max_contours_id, (150, 0, 255))

        cv2.rectangle(poi_contours, (max_bound_rect[0], max_bound_rect[1]),
                      (max_bound_rect[0] + max_bound_rect[2], max_bound_rect[1] + max_bound_rect[3]),
                      (150, 0, 255), 2)

        # cv2.imshow("poi", poi_contours)
        print(poi_position)

        cv2.waitKey(-1)

        cv2.destroyAllWindows()


def check_threshold():
    def _check_threshold(frame_filename):
        frame = cv2.imread(frame_filename)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        start = time.time()
        wall_img = cv2.inRange(hsv, wall_lower_range, wall_upper_range)
        wall_img = cv2.GaussianBlur(wall_img, (11, 1), 1, 1, wall_img.shape[0])
        print("Wall detection took: {}".format(time.time() - start))

        wall_img_filtered = copy.deepcopy(wall_img)

        start = time.time()
        frame[wall_img_filtered > 150] = 0
        print("Wall removal took: {}".format(time.time() - start))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # cv2.imshow("frame", hsv)

        def change_h_lower_param(x):
            global data
            data["h_lower"] = x
            poi_color_lower = np.array([data["h_lower"], data["s_lower"], data["v_lower"]], dtype=np.uint8)
            poi_color_upper = np.array([data["h_upper"], data["s_upper"], data["v_upper"]], dtype=np.uint8)

            img = cv2.inRange(hsv.copy(), poi_color_lower, poi_color_upper)
            # cv2.imshow("frame", img)

        def change_s_lower_param(x):
            global data

            data["s_lower"] = x

            poi_color_lower = np.array([data["h_lower"], data["s_lower"], data["v_lower"]], dtype=np.uint8)
            poi_color_upper = np.array([data["h_upper"], data["s_upper"], data["v_upper"]], dtype=np.uint8)

            img = cv2.inRange(hsv.copy(), poi_color_lower, poi_color_upper)
            # cv2.imshow("frame", img)

        def change_v_lower_param(x):
            global data

            data["v_lower"] = x
            poi_color_lower = np.array([data["h_lower"], data["s_lower"], data["v_lower"]], dtype=np.uint8)
            poi_color_upper = np.array([data["h_upper"], data["s_upper"], data["v_upper"]], dtype=np.uint8)

            img = cv2.inRange(hsv.copy(), poi_color_lower, poi_color_upper)

            # cv2.imshow("frame", img)

        def change_h_upper_param(x):
            global data
            data["h_upper"] = x

            poi_color_lower = np.array([data["h_lower"], data["s_lower"], data["v_lower"]], dtype=np.uint8)
            poi_color_upper = np.array([data["h_upper"], data["s_upper"], data["v_upper"]], dtype=np.uint8)

            img = cv2.inRange(hsv.copy(), poi_color_lower, poi_color_upper)
            # cv2.imshow("frame", img)

        def change_s_upper_param(x):
            global data

            data["s_upper"] = x

            poi_color_lower = np.array([data["h_lower"], data["s_lower"], data["v_lower"]], dtype=np.uint8)
            poi_color_upper = np.array([data["h_upper"], data["s_upper"], data["v_upper"]], dtype=np.uint8)

            img = cv2.inRange(hsv.copy(), poi_color_lower, poi_color_upper)
            # cv2.imshow("frame", img)

        def change_v_upper_param(x):
            global data

            data["v_upper"] = x

            poi_color_lower = np.array([data["h_lower"], data["s_lower"], data["v_lower"]], dtype=np.uint8)
            poi_color_upper = np.array([data["h_upper"], data["s_upper"], data["v_upper"]], dtype=np.uint8)

            img = cv2.inRange(hsv.copy(), poi_color_lower, poi_color_upper)
            # cv2.imshow("frame", img)

        # {'s_lower': 20, 'h_lower': 41, 'v_upper': 255, 'v_lower': 187, 'h_upper': 171, 's_upper': 139}
        cv2.namedWindow('frame')
        cv2.createTrackbar('h_lower', 'frame', 0, 171, change_h_lower_param)
        cv2.createTrackbar('s_lower', 'frame', 0, 255, change_s_lower_param)
        cv2.createTrackbar('v_lower', 'frame', 0, 255, change_v_lower_param)
        cv2.createTrackbar('h_upper', 'frame', 0, 171, change_h_upper_param)
        cv2.createTrackbar('s_upper', 'frame', 0, 255, change_v_upper_param)
        cv2.createTrackbar('v_upper', 'frame', 0, 255, change_s_upper_param)

        cv2.waitKey(-1)
        cv2.destroyAllWindows()

        return data

    poi_thresholds = []
    for name in glob.glob("img/poi*.png"):
        print("Analyzing poi image {}".format(name))

        data = _check_threshold(name)
        poi_thresholds.append(data)

    print(data)

def vision_test():
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Display the resulting frame
        lower_range = np.array([169, 100, 100], dtype=np.uint8)
        upper_range = np.array([189, 255, 255], dtype=np.uint8)

        cv2.namedWindow('frame')
        # cv2.imshow('frame', frame)
        cv2.resizeWindow('frame', 100, 100)
        mask = cv2.inRange(hsv, lower_range, upper_range)

        cv2.namedWindow('mask')
        # cv2.imshow('mask', mask)
        cv2.resizeWindow('mask', 100, 100)

        if np.mean(mask) > 0.5:
            print("Detected red object in image!")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # vision_test2()
    check_threshold()
