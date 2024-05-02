import time
from robomaster import robot


if __name__ == '__main__':
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    x_val = 0.1


    ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
    time.sleep(3)
    # stop when keyboard pressed:
    
        
    