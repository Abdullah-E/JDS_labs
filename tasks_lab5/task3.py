# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import robomaster
from robomaster import robot
import time
import math
import numpy as np

class Controller():

    def __init__(self, position, goal_x, goal_y):
        self.robot_x, self.robot_y, self.robot_theta = position
        self.time_prev = time.time()

        self.path_x = []
        self.path_y = []

        self.obstacle_x = []
        self.obstacle_y = []

        self.points = []
        self.points_to_goal = []

        self.linear_vel = 0.1

        self.final_goal_x = goal_x
        self.final_goal_y = goal_y

        self.init_obstacle_point_x = 0
        self.init_obstacle_point_y = 0
        
        self.robot = robot.Robot()
        self.robot.initialize(conn_type="ap")
        self.chassis = self.robot.chassis
    
        self.sensor = self.robot.sensor
        self.sensor.sub_distance(freq=5, callback=self.sub_data_handler)
        self.chassis.sub_position(freq=10, callback=self.position_update)
        self.chassis.sub_attitude(freq=10, callback=self.angle_update)

    def euclidean_distance(self, x_init, y_init, x_final, y_final):
        return math.sqrt(math.pow((x_final - x_init), 2) + math.pow((y_final - y_init), 2))
    
    def goal_theta(self, x_init, y_init, x_final, y_final):
        return math.atan2(y_final - y_init, x_final - x_init)
    
    def potential_field(self, error, K_gains):
        return K_gains * error
    
    def go_to_goal_pf(self, goal_position, obstacle_flag):
        goal_x, goal_y = goal_position
        distance_tolerance = 0.2
        
        self.path_x.append(self.robot_x)
        self.path_y.append(self.robot_y)
        
        error = np.array([goal_x - self.robot_x, goal_y - self.robot_y])
        distance = np.linalg.norm(error)
        K_gains = np.array([0.3, 0.5])

        time.sleep(1)
        while distance > distance_tolerance:
            # print(distance)
            
            R_inv = np.array([
                [np.cos(self.robot_theta), np.sin(self.robot_theta)],
                [-np.sin(self.robot_theta), np.cos(self.robot_theta)]
            ])

            u = self.potential_field(error, K_gains)
            components = R_inv @ u
            linear_velocity = components[0]
            angular_velocity = components[1]

            linear_x = linear_velocity * math.cos(self.robot_theta)
            linear_y = linear_velocity * math.sin(self.robot_theta)
            # print(linear_x)
            self.chassis.drive_speed(x=linear_velocity, y=0, z=angular_velocity)
            
            next_x_position = round(min(linear_x, 1), 3)
            next_y_position = round(min(linear_y, 1), 3)

            # self.robot_x += next_x_position
            # self.robot_y += next_y_position
            # self.robot_theta += angular_velocity

            self.path_x.append(self.robot_x)
            self.path_y.append(self.robot_y)


            if obstacle_flag == 1:
                self.obstacle_x.append(self.robot_x)
                self.obstacle_y.append(self.robot_y)
                
                point = np.array([self.robot_x, self.robot_y], dtype=float)
                self.points.append(point)

                point_to_goal = round(self.euclidean_distance(point[0], point[1], self.final_goal_x, self.final_goal_y), 3)
                self.points_to_goal.append(point_to_goal)

            error = np.array([goal_x - self.robot_x, goal_y - self.robot_y])
            distance = np.linalg.norm(error)
            self.linear_vel = linear_velocity

            if obstacle_flag == 0:
                self.init_obstacle_point_x = self.robot_x
                self.init_obstacle_point_y = self.robot_y
            time.sleep(1)

        
        self.chassis.drive_speed(x=0, y=0, z=0)
        return
        return self.path_x, self.path_y, 1


    def go_back(self):
        print(self.points)
        min_index = np.argmin(self.points_to_goal)
        min_point = self.points[min_index]

        print(min_point)
        for int_goal in self.points[::-1]:
            self.go_to_goal_pf(int_goal, 0)

            if min_point[0] == int_goal[0] and min_point[1] == int_goal[1]:
                break
        
        self.go_to_goal_pf([self.final_goal_x, self.final_goal_y], 0)

    def turn_left(self):
        # turning the robot left upon obstacle
        self.chassis.drive_speed(x=1, y=0, z=-10)
        time.sleep(1)
        self.chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(1)
        

    def circle_obstacle(self, flag, goals):
        for goal in goals:
            path_x, path_y, flag = self.go_to_goal_pf(goal, flag)

        coordinates = zip(path_x, path_y)
        for c in coordinates:
            self.points.append(c)
            d = self.euclidean_distance(c[0], c[1], self.final_goal_x, self.final_goal_y)
            self.points_to_goal.append(d)


        return path_x, path_y, flag

# , self.robot_y, self.robot_theta

    def position_update(self, info):
        x,y,z = info
        self.robot_x = x
        self.robot_y = y
        print(f'{x}, {y}')
    def angle_update(self, info):

        yaw, pitch, roll = info
        self.robot_theta = (yaw*np.pi)/180
        
    def sub_data_handler(self, sub_info):
        self.distance = sub_info[0]
    # print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))


if __name__ == '__main__':
    
    robot = Controller([0, 0, 0], 2, 0)
    
    robot.go_to_goal_pf([2, 0], 0)
    robot.go_to_goal_pf([2, -2], 0)
    robot.go_to_goal_pf([4, 0], 0)
    robot.go_to_goal_pf([1, 0], 0)
    robot.go_to_goal_pf([1, 0], 0)
    robot.go_to_goal_pf([1, 0], 0)
    print("NOW TURN")
    robot.turn_left()
    
    