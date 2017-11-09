import copy
import time

import cv2
import numpy as np

wall_lower_range = np.array([0, 50, 0], dtype=np.uint8)
wall_upper_range = np.array([60, 200, 255], dtype=np.uint8)
poi_color_lower = np.array([0, 0, 150], dtype=np.uint8)
poi_color_upper = np.array([200, 50, 200], dtype=np.uint8)

class Vision:
    def __init__(self, IO):
        self.IO = IO
        self._poi_position = None
        self.IO.cameraSetResolution("low")

    def poi_present(self):
        return self._poi_position is not None

    def get_centerline_distance(self):
        pass

    def grab_image(self, n=5):
        for i in range(n):
            self.IO.cameraGrab()

        return self.IO.cameraRead()

    def get_poi_location(self, frame, raw=False):
        # frame = self._grab_image()
        # self.IO.imshow("frame", frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        start = time.time()
        wall_img = cv2.inRange(hsv, wall_lower_range, wall_upper_range)
        wall_img = cv2.GaussianBlur(wall_img, (11, 1), 1, 1, wall_img.shape[0])
        print("Wall detection took: {}".format(time.time() - start))

        wall_img_filtered = copy.deepcopy(wall_img)
        last_points = [0 for _ in range(wall_img.shape[1])]

        start = time.time()
        frame[wall_img_filtered > 150] = 0
        # for j in range(wall_img_filtered.shape[1]):
        #     for i in range(wall_img_filtered.shape[0]):
        #         if wall_img_filtered[i][j] > 150 and i > last_points[j]:
        #             last_points[j] = i
        #     point = last_points[j]
        #     wall_img_filtered[:point + 1, j] = np.zeros(point + 1)
        #     frame[:point + 1, j] = np.zeros((point + 1, 3))
        print("Wall removal took: {}".format(time.time() - start))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        poi_img = cv2.inRange(hsv, poi_color_lower, poi_color_upper)

        # start = time.time()
        # red_channel = frame[:, :, 2]
        # blue_channel = frame[:, :, 0]
        # green_channel = frame[:, :, 1]
        #
        # rc_centered = compute_error_matrix(red_channel, "mean")
        # bc_centered = compute_error_matrix(blue_channel, "mean")
        # gc_centered = compute_error_matrix(green_channel, "mean")
        # qwe = np.log((rc_centered * bc_centered * gc_centered) + 0.000005)
        # error_map = cv2.applyColorMap(qwe.astype(np.uint8), cv2.COLORMAP_JET)
        # print("Error matrix computation took: {}".format(time.time() - start))

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
            # return the centre of the POI in the camera frame
            if raw:
                moments = cv2.moments(max_bound_rect)

                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                self._poi_position = (cx, cy)

                poi_contours = np.zeros_like(poi_img_canny)
                cv2.drawContours(poi_contours, contours0, max_contours_id, (150, 0, 255))

                cv2.rectangle(poi_contours, (max_bound_rect[0], max_bound_rect[1]),
                              (max_bound_rect[0] + max_bound_rect[2], max_bound_rect[1] + max_bound_rect[3]),
                              (150, 0, 255), 2)

                return self._poi_position, poi_contours
            else:
                # TODO: transform point in the camera frame to the world reference frame
                return None, None
        return None, None


def detected_colored_object(image, color_lower_range, color_upper_range):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, color_lower_range, color_upper_range)

    return np.mean(mask) > 0.5


def get_robot_position_from_camera(image):
    pass


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


def vision_test2():
    frame = cv2.imread("img/img1.jpg")
    wall_lower_range = np.array([0, 50, 0], dtype=np.uint8)
    wall_upper_range = np.array([60, 200, 255], dtype=np.uint8)
    poi_color_lower = np.array([0, 0, 150], dtype=np.uint8)
    poi_color_upper = np.array([200, 50, 200], dtype=np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    wall_img = cv2.inRange(hsv, wall_lower_range, wall_upper_range)

    wall_img = cv2.GaussianBlur(wall_img, (11, 1), 1, 1, wall_img.shape[0])
    wall_img_filtered = copy.deepcopy(wall_img)
    last_points = [0 for i in range(wall_img.shape[1])]

    for j in range(wall_img_filtered.shape[1]):
        for i in range(wall_img_filtered.shape[0]):
            if wall_img_filtered[i][j] > 150 and i > last_points[j]:
                last_points[j] = i
        point = last_points[j]
        wall_img_filtered[:point + 1, j] = np.zeros(point + 1)
        frame[:point + 1, j] = np.zeros((point + 1, 3))

    blurred_frame = cv2.GaussianBlur(frame, (11, 11), 100)
    cv2.imwrite("img/wall_img_filtered.jpg", wall_img_filtered)
    cv2.imwrite("img/wall_img.jpg", wall_img)
    cv2.imwrite("img/poi_filtered.jpg", frame)

    red_channel = frame[:, :, 2]
    blue_channel = frame[:, :, 0]
    green_channel = frame[:, :, 1]

    rc_centered = cv2.GaussianBlur(compute_error_matrix(red_channel, "mean"), (31, 15), 100)
    bc_centered = cv2.GaussianBlur(compute_error_matrix(blue_channel, "mean"), (31, 15), 100)
    gc_centered = cv2.GaussianBlur(compute_error_matrix(green_channel, "mean"), (31, 15), 100)

    # rc_centered = cv2.GaussianBlur(row_base_mean_centered(red_channel), (31, 15), 100)
    # bc_centered = cv2.GaussianBlur(row_base_mean_centered(blue_channel), (31, 15), 100)
    # gc_centered = cv2.GaussianBlur(row_base_mean_centered(green_channel), (31, 15), 100)
    # qwe = np.log10(np.log10(np.log10((rc_centered*bc_centered*gc_centered)+1)+1)+1)
    qwe = np.log((rc_centered * bc_centered * gc_centered) + 0.000005)
    # print(np.max(qwe))
    # print(np.mean(qwe))
    # plt.plot(np.arange((np.sort(qwe.flatten())).shape[0]), np.sort(qwe.flatten()))
    # plt.savefig('img/foo.png')
    # qwe /= np.max(qwe)
    #  print((cv2.applyColorMap(np.expand_dims(rc_centered, axis=2), cv2.COLORMAP_JET)).shape)
    error_map = cv2.applyColorMap(qwe.astype(np.uint8), cv2.COLORMAP_JET)
    cv2.imwrite("img/error.png", error_map)

    # canny_error_map = cv2.Canny(error_map.copy(), 500, 200)
    # cv2.imwrite("img/canny.png", canny_error_map)


    #
    # def change_fst_param(x):
    #     global data
    #     data["param1"] = x
    #
    #     cannied_img = cv2.Canny(error_map.copy(), data["param1"], data["param2"], data["param3"])
    #     cv2.imshow("canny", cannied_img)
    #
    # def change_snd_param(x):
    #     global data
    #
    #     data["param2"] = x
    #     cannied_img = cv2.Canny(error_map.copy(), data["param1"], data["param2"], data["param3"])
    #     cv2.imshow("canny", cannied_img)
    #
    # def change_trd_param(x):
    #     global data
    #
    #     data["param3"] = x
    #     cannied_img = cv2.Canny(error_map.copy(), data["param1"], data["param2"], data["param3"])
    #
    # cv2.namedWindow('canny')
    # cv2.createTrackbar('canny_param1', 'canny', 1, 1000, change_fst_param)
    # cv2.createTrackbar('canny_param2', 'canny', 1, 1000, change_snd_param)
    # cv2.createTrackbar('canny_param3', 'canny', 1, 1000, change_trd_param)
    #
    # cv2.waitKey(-1)
    #
    # cv2.destroyAllWindows()

    error_map = cv2.Canny(error_map.copy(), 10, 10)
    _, contours0, hierarchy = cv2.findContours(error_map.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    max_contours_id = None
    max_area = 0
    max_bound_rect = None
    areas = []

    for i, cnt in enumerate(contours0):
        approx_cnt = cv2.approxPolyDP(cnt, 3, True)
        bound_rect = cv2.boundingRect(approx_cnt)
        cnt_area = bound_rect[2] * bound_rect[3]
        areas.append(cnt_area)

        if cnt_area > max_area:
            max_area = cnt_area
            max_contours_id = i
            max_bound_rect = bound_rect

    mean_area = np.mean(areas)

    if np.abs(max_area - mean_area) > 0.5:
        return True

    return False
    #
    # poi_contours = np.zeros_like(error_map)
    # cv2.drawContours(poi_contours, contours0, max_contours_id, (150, 0, 255))
    #
    # cv2.rectangle(poi_contours, (max_bound_rect[0], max_bound_rect[1]), (max_bound_rect[0]+max_bound_rect[2], max_bound_rect[1] + max_bound_rect[3]), (150, 0, 255), 2)
    # cv2.imshow("contours", poi_contours)
    # cv2.imshow("error_map", error_map)
    # cv2.waitKey(-1)


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
        cv2.imshow('frame', frame)
        cv2.resizeWindow('frame', 100, 100)
        mask = cv2.inRange(hsv, lower_range, upper_range)

        cv2.namedWindow('mask')
        cv2.imshow('mask', mask)
        cv2.resizeWindow('mask', 100, 100)

        if np.mean(mask) > 0.5:
            print("Detected red object in image!")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print(vision_test2())
