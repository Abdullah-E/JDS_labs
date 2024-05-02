import cv2
import robomaster
from robomaster import robot
import numpy as np
import time
from math import atan2, pi
robo_x, robo_y = (0,0)
robo_t = 0

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
def get_distance(width):
    
    fc = 630
    Wr = 0.075
    # img_width = 1280
    
    dist = (Wr * fc)/(width)
    return dist
    
def get_angle(x_pixel):
    angle = (x_pixel*120)/1280
    angle -= 60
    angle_rad = angle * (np.pi/180)
    return angle_rad

markers = []

# R G B Format
lower_bound = np.array([70, 70, 125])
upper_bound = np.array([200, 255, 255])


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=10, callback=position_update)
    ep_chassis.sub_attitude(freq=10, callback=angle_update)
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_gripper.close(power =50)
    time.sleep(1)
    ep_camera.start_video_stream(display=False)

    dist_tolerance = 0.3

    try:
        reached = False  
        drive = 1
        for i in range(0, 500):
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            
            # converting image to hsv
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # creating mask
            mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
            
            # applying mask to img
            img = cv2.bitwise_and(img, img, mask=mask)
            
            # finding contoures
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)        
            
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # drawing bounding box
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                dist = get_distance(w)
                angle = get_angle(x + (w/2))
                print(dist)
                
                if(angle > abs(0.01)):
                    if angle > 0:
                        ep_chassis.drive_speed(x=0, y=0, z=0.5, timeout=5)
                    else:
                        ep_chassis.drive_speed(x=0, y=0, z=-0.5, timeout=5)
                    time.sleep(0.5)
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    time.sleep(0.5)
                    
                if (dist > dist_tolerance) :
                    ep_chassis.drive_speed(x=0.1, y=0, z=0, timeout=5)
                else:
                    reached = True
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    break
                    

                    
            cv2.imshow("Masked Object Detection", img)
            cv2.waitKey(1)
        
        if reached:
            ep_gripper.open(power=50)
            time.sleep(1)
            # ep_gripper.pause()
            ep_chassis.move(x= 0.1, y = 0, z =0, xy_speed = 0.1).wait_for_completed()
            ep_gripper.close(power =50)
            time.sleep(1)
            # ep_gripper.pause()
            
        cv2.destroyAllWindows()

        
        
    except:
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()
    ep_robot.close()


