import cv2
import numpy as np
import time
from util import tpl2np, np2tpl


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


class OpticalFlow:
    '''
    A device used to extract information about angular displacement of the robot
    by performing very rudimentary optical flow on the centre of the image.
    ''' 
    def __init__(self, IO):
        # The IO object for reading from the camera
        self.IO = IO

        # A structure meant to hold information extracted from the last image
        self.last_centre = {'img': None, 'timestamp': 0}

        # Determines how small is the central region we convolute
        self.crop_ratio = 1.5

        # Setting this enables operating on smaller resolution camera images if
        # performance is unsatisfactory
        self.downscaling_ratio = 4

        # Method used for cross-convolution
        self.ccv_method = cv2.TM_CCOEFF
        # self.ccv_method = cv2.TM_CCOEFF_NORMED
        # self.ccv_method = cv2.TM_CCORR
        # self.ccv_method = cv2.TM_CCORR_NORMED
        # self.ccv_method = cv2.TM_SQDIFF
        # self.ccv_method = cv2.TM_SQDIFF_NORMED

        # Angular displacement coefficient for converting from pixel data
        self.ang_disp_coef = 1 # TODO callibrate

    def snap(self):
        '''
        Returns the time elapsed and estimated angular displacement of the robot since
        the last snap or None if it's the first time we've 'snapped'.
        '''

        # Capture a new image and extract it's centre for next 'snap's use
        self.IO.cameraGrab()
        frame = self.IO.cameraRead()

        time_elapsed = time.time() - self.last_centre['timestamp']
        self.last_centre['timestamp'] += time_elapsed
        ang_disp_pix = None

        # Figure out sizes of everything
        img_w, img_h, dimmmm = frame.shape
        img_sh = np.array([img_h, img_w])
        frame = frame
        if self.downscaling_ratio > 1:
            dwnscld_sh = img_sh/self.downscaling_ratio
            dwnscld_frame = cv2.resize(frame, np2tpl(dwnscld_sh))
        else:
            dwnscld_frame = frame
            dwnscld_sh = img_sh

        crop_sh = dwnscld_sh/self.crop_ratio
        res_sh = dwnscld_sh-crop_sh+1
        zero_movement_pt = {4:  res_sh/2+np.array([19,13]),
                            1.5: res_sh/2+np.array([-162/6,-20.5])}

        if self.last_centre['img'] is not None:
            # Estimate the angular displacement since the last call.
            res = cv2.matchTemplate(dwnscld_frame, self.last_centre['img'], self.ccv_method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            # detector = cv2.SimpleBlobDetector()
            # keypoints = detector.detect(res)
            # print(keypoints)
            # self.IO.imshow('frame',frame)
            # self.IO.imshow('dwnscld_frame',dwnscld_frame)
            # self.IO.imshow('self.last_centre[\'img\']',self.last_centre['img'])
            # self.IO.imshow('res',(res-min_val)/(max_val-min_val))
            ang_disp_pix = tpl2np(max_loc) - zero_movement_pt[self.crop_ratio]
            # ang_disp_pix = tpl2np(max_loc)
            print(zero_movement_pt[self.crop_ratio])

        # Crop the centre of this image for future use
        from_coords = (dwnscld_sh-crop_sh)/2
        to_coords = dwnscld_sh-(dwnscld_sh-crop_sh)/2

        self.last_centre['img'] = dwnscld_frame[from_coords[1]:to_coords[1],from_coords[0]:to_coords[0]]

        if ang_disp_pix is not None:
            return {'displacement': ang_disp_pix*self.ang_disp_coef, 'time_elapsed': time_elapsed}
        else:
            return None

if __name__ == "__main__":
    vision_test()
