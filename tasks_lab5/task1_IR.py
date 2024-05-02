import robomaster
from robomaster import robot
import time


def sub_data_handler(sub_info):
    distance = sub_info
    print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    time.sleep(60)
    ep_sensor.unsub_distance()
    ep_robot.close()
