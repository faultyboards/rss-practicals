import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Display the resulting frame
    lower_range = np.array([169, 100, 100], dtype=np.uint8)
    upper_range = np.array([189, 255, 255], dtype=np.uint8)

    cv2.imshow('frame', frame)
    mask = cv2.inRange(hsv, lower_range, upper_range)

    cv2.imshow('mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
