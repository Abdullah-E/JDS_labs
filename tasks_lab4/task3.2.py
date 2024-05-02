import time
import numpy as np
from robomaster import robot
from math import atan2, pi

robo_x, robo_y = (0,0)
robo_t = 0


def euclidean_dist(x1, y1, x2, y2):
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5


def potential_field(err, K):
    return np.multiply(K, err)

# go to goal using potential field
def goto_goal(x, y):
    dist_tolerance = 0.15
    # dist_err = euclidean_dist(robo_x, robo_y, x, y)
    # lin_vel = 1
    
    RInv = np.array([[np.cos(robo_t), np.sin(robo_t)], [-np.sin(robo_t), np.cos(robo_t)]])
    goal = np.array([x, y])
    robot = np.array([robo_x, robo_y])
    error = goal - robot
    K_gains = np.array([0.5, 1.5])
    
    dist = np.linalg.norm(error)
    
    while dist > dist_tolerance:
        dist = np.linalg.norm(error)
        
        RInv = np.array([[np.cos(robo_t), np.sin(robo_t)], [-np.sin(robo_t), np.cos(robo_t)]])
        robot = np.array([robo_x, robo_y])
        error = goal - robot
        u = potential_field(error, K_gains)
        vels = np.matmul(RInv, u)
        lin_vel = vels[0]
        ang_vel = (vels[1]*180)/pi
        # if dist <= dist_tolerance + 0.2:
        #     ang_vel = vels[1]*1000
        # else:
        #     ang_vel = vels[1] *25
        
        ep_chassis.drive_speed(x=lin_vel, y=0, z=ang_vel, timeout = 0.1)
        time.sleep(0.1)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout = 0.1)

def position_update(info):
    global robo_x, robo_y
    x,y,z = info
    robo_x = x
    robo_y = y
    print(f'{x}, {y}')
def angle_update(info):
    global robo_t
    yaw, pitch, roll = info
    robo_t = (yaw*pi)/180
    print(robo_t)
    
if __name__ == '__main__':
    try:
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        ep_chassis = ep_robot.chassis
        ep_chassis.sub_position(freq=10, callback=position_update)
        ep_chassis.sub_attitude(freq=10, callback=angle_update)
        
        # goto_goal(1,-1)
        goto_goal(2,0)
        goto_goal(2,2)
        # goto_goal(3,-3)
        # goto_goal(0,-3)
        # goto_goal(0,0)
        ep_robot.close()
    except KeyboardInterrupt as e:
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout = 0.1)
        ep_robot.close()
        pass