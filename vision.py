import cv2
import numpy as np


def detected_colored_object(image, color_lower_range, color_upper_range):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, color_lower_range, color_upper_range)

    return np.mean(mask) > 0.5


def get_robot_position_from_camera(image):
    pass


def vision_test():
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if frame is None:
            print('No camera!')
            break

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

def check_image():
    frame  = cv2.imread('img2.jpg')
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
   

if __name__ == "__main__":
    vision_test()
