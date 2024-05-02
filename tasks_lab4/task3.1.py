import time
import keyboard
from robomaster import robot
from math import atan2, pi
robo_x, robo_y = (0,0)
robo_t = 0

def euclidean_dist(x1, y1, x2, y2):
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

def PID(err, prev_err, acc_err):
    
    kp = 1
    kd = 0.6
    ki = 0.3
    potential = kp*err
    if prev_err == 0:
        prev_err = err
    differential = kd*(err - prev_err)
    integral = ki*acc_err
    
    PID = potential + differential + integral
    angle_scale = 50
    return PID * angle_scale
    
    

def goto_goal(x, y):
    dist_tolerance = 0.1
    
    dist_err = euclidean_dist(robo_x, robo_y, x, y)
    linear_vel = 1
    angle_err_acc = 0
    prev_err = 0
    while dist_err > dist_tolerance:
        # print("dist_err: ", dist_err)
        
        dist_err = euclidean_dist(robo_x, robo_y, x, y)
        linear_vel = dist_err*0.8
        angle_to_goal = atan2(y - robo_y, x - robo_x)
        angle_err = angle_to_goal - robo_t
        if angle_err > pi:
            angle_err -= 2*pi
        elif angle_err < -pi:
            angle_err += 2*pi
        angular_vel = PID(angle_err, 0, angle_err_acc)
        # print(robo_x, robo_y, angle_err, angular_vel)
        print(f"current pos: {robo_x}, {robo_y}")
        ep_chassis.drive_speed(x=linear_vel, y=0, z=angular_vel, timeout = 0.1)
        time.sleep(0.1)
        
        
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout = 0.1)
    print("Reached Goal!")
    pass

def position_update(info):
    global robo_x, robo_y
    x,y,z = info
    robo_x = x
    robo_y = y
    # print(f'{x}, {y}')
def angle_update(info):
    global robo_t
    yaw, pitch, roll = info
    robo_t = (yaw*pi)/180
if __name__ == '__main__':
    try:
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        ep_chassis = ep_robot.chassis
        ep_chassis.sub_position(freq=10, callback=position_update)
        ep_chassis.sub_attitude(freq=10, callback=angle_update)
        
        goto_goal(1,0)
        goto_goal(0,0)
        ep_robot.close()
    except KeyboardInterrupt as e:
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout = 0.1)
        ep_robot.close()
        pass
    