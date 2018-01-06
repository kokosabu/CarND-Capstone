from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        RED_MIN1 = np.array([  0.0/360*255, 120, 120], np.uint8)
        RED_MAX1 = np.array([ 35.0/360*255, 255, 255], np.uint8)
        RED_MIN2 = np.array([330.0/360*255, 120, 120], np.uint8)
        RED_MAX2 = np.array([360.0/360*255, 255, 255], np.uint8)
        frame_threshed = cv2.inRange(hsv_image, RED_MIN1, RED_MAX1)
        red  = cv2.countNonZero(frame_threshed)
        frame_threshed = cv2.inRange(hsv_image, RED_MIN2, RED_MAX2)
        red += cv2.countNonZero(frame_threshed)
        if red > 50:
            return TrafficLight.RED

        YELLOW_MIN = np.array([40.0/360*255, 120, 120], np.uint8)
        YELLOW_MAX = np.array([66.0/360*255, 255, 255], np.uint8)
        frame_threshed = cv2.inRange(hsv_image, YELLOW_MIN, YELLOW_MAX)
        yellow = cv2.countNonZero(frame_threshed)
        if yellow > 50:
            return TrafficLight.YELLOW

        GREEN_MIN = np.array([ 90.0/360*255, 120, 120], np.uint8)
        GREEN_MAX = np.array([140.0/360*255, 255, 255], np.uint8)
        frame_threshed = cv2.inRange(hsv_image, GREEN_MIN, GREEN_MAX)
        green = cv2.countNonZero(frame_threshed)
        if green > 50:
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
