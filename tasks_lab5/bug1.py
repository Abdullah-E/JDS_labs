#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from 
# from geometry_msgs.msg import Twist
# from math import pow, atan2, sqrt

class TurtleChase:

        def __init__(self):
            rospy.init_node('rover_controller', anonymous=True)
            self.subscriber = rospy.Subscriber('/serial_node/ultrasonic', Int32, self.sonar_update)
            # self.publisher = rospy.Publisher('/chaser/cmd_vel', Twist, queue_size=10)
            # self.chaser_subscriber = rospy.Subscriber('/chaser/pose', Pose, self.chaser_pose_update)
            self.front_distance = 0

        def sonar_update(self, data):
            self.front_distance = data.data
            print(self.front_distance)
            # self.chaser_pose.x = data.x
            # self.chaser_pose.y = data.y
            # self.chaser_pose.theta = data.theta
            # self.chaser_pose.linear_velocity = data.linear_velocity
            # self.chaser_pose.angular_velocity = data.angular_velocity
        
        
        
if __name__ == '__main__':
    print("started")
    try:

        x = TurtleChase()
        # x.check_chaser_pose()
        while not rospy.is_shutdown():
            # x.check_chaser_pose()
            pass
    except rospy.ROSInterruptException:
        pass
