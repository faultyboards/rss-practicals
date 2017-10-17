import cv2
import numpy as np
import time


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


class OpticalFlow():
    '''
    A device used to extract information about angular displacement of the robot
    by performing very rudimentary optical flow on the centre of the image.
    ''' 
    def __init__(self, cap):
        # The VideoCapture opject shared by all the vision agents
        self.cap = cap 
        # A structure meant to hold information extracted from the last image
        self.last_centre = {'img': None, 'timestamp': None}

        # Determines how small is the central region we convolute
        self.cropping_ratio

        # Setting this enables operating on smaller resolution camera images if
        # performance is unsatisfactory
        self.downscaling_ratio = 1

    def snap(self, min_time=None):
        '''
        Returns the estimated angular displacement of the robot since the last snap
        or None if it's the first time we've 'snapped'.
        '''

        # Capture a new image and extract it's centre for next 'snap's use
        ret, frame = self.cap.read()

        img_w, img_h, dimmmm = frame.shape
        crop_w, crop_h = (img_w/crop_ratio, img_h/crop_ratio)


        res_w, res_h = (img_w-crop_w+1, img_h-crop_h+1)
        i=0
        img = frame
        lr_img = cv2.resize(img, (img_h/self.downscaling_ratio, img_w/self.downscaling_ratio))
        # print lr_img.shape
        if crop_img is not None:
            # print crop_img.shape
            res = cv2.matchTemplate(lr_img, crop_img, eval(method))
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            # pdb.set_trace()
            movement = (max_loc[0]-zero_movement_pt[crop_ratio][0], max_loc[1]-zero_movement_pt[crop_ratio][1])
            crop_img = lr_img[(img_w-crop_w)/2:img_w-(img_w-crop_w)/2,(img_h-crop_h)/2:img_h-(img_h-crop_h)/2]

        if self.last_centre['img'] is not None:


if __name__ == "__main__":
    vision_test()
