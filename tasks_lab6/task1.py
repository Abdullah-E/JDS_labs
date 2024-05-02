import cv2
import robomaster
from robomaster import robot
from robomaster import vision
import numpy as np


class MarkerInfo:

    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def text(self):
        return self._info


markers = []


def get_distance(width):
    
    fc = 630
    Wr = 0.1524
    img_width = 1280
    
    dist = (Wr * fc)/(width*img_width)
    return dist
    
def get_angle(x_pixel):
    angle = x_pixel*120
    angle -= 60
    return angle


def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(MarkerInfo(x, y, w, h, info))
        # print("marker:{0} x:{1}, y:{2}, w:{3}, h:{4}".format(info, x, y, w, h))
        distance = get_distance(w)
        angle = get_angle(x)
        # print(distance, angle)
        angle_red = angle * (np.pi/180)
        x_coordinate = distance * np.cos(angle_red)
        y_coordinate = distance * np.sin(angle_red)
        print(x_coordinate, y_coordinate)
        # print()
        


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera

    ep_camera.start_video_stream(display=False)
    result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

    for i in range(0, 5000):
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        for j in range(0, len(markers)):
            cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
            cv2.putText(img, markers[j].text, markers[j].center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        cv2.imshow("Markers", img)
        # get width of img
        # w = img.shape[1]
        # h = img.shape[0]
        # print(w, h)
        cv2.waitKey(1)
    cv2.destroyAllWindows()

    result = ep_vision.unsub_detect_info(name="marker")
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()