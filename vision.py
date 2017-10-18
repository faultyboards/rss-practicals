import copy

import cv2
import numpy as np


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
    m_centered = np.where(m > 145, np.sqrt(np.square(m - np.expand_dims(central_value, axis=1))), np.zeros_like(m))

    return m_centered


def vision_test2():
    frame = cv2.imread("img/poi.png")
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

    blurred_frame = cv2.GaussianBlur(frame, (51, 51), 5, 1, 5)
    cv2.imwrite("img/wall_img_filtered.jpg", wall_img_filtered)
    cv2.imwrite("img/wall_img.jpg", wall_img)
    cv2.imwrite("img/poi_filtered.jpg", frame)

    hsv_filtered = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_filtered = cv2.inRange(hsv_filtered, poi_color_lower, poi_color_upper)
    frame_filtered = cv2.bitwise_and(frame, frame, mask=hsv_filtered)
    frame_filtered = cv2.medianBlur(frame_filtered, 5)
    frame_filtered = cv2.morphologyEx(frame_filtered, cv2.MORPH_OPEN, (5, 5))

    cv2.imwrite("img/poi_filtered_mask.jpg", frame_filtered)

    # code for error matrix calculation
    # red_channel = blurred_frame[:, :, 0]
    # blue_channel = blurred_frame[:, :, 1]
    # green_channel = blurred_frame[:, :, 2]
    #
    # rc_centered = compute_error_matrix(red_channel, "mean")
    # bc_centered = compute_error_matrix(blue_channel, "mean")
    # gc_centered = compute_error_matrix(green_channel, "mean")
    #
    # error_matrix = rc_centered + bc_centered + gc_centered
    #
    # _, img_error = cv2.threshold(error_matrix, np.median(error_matrix), np.max(error_matrix), cv2.THRESH_BINARY)
    #
    # cv2.imwrite("img/error.png", img_error)


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
    vision_test2()
