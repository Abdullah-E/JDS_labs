import cv2
import robomaster
from robomaster import robot
import numpy as np
import time
import math
robo_x, robo_y = (0,0)
robo_t = 0

def hough_lines(img):
    src = img
    
    
    dst = cv2.Canny(src, 50, 200, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    
    
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
    
    cv2.imshow("Source", src)
    # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    cv2.waitKey(1)

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
    while True:
        # print("dist_err: ", dist_err)
        
        # angle_to_goal = math.atan2(y - robo_y, x - robo_x)
        angle_err = angle_to_goal - robo_t
        if angle_err > math.pi:
            angle_err -= 2*math.pi
        elif angle_err < -math.pi:
            angle_err += 2*math.pi
        angular_vel = PID(angle_err, 0, angle_err_acc)
        # print(robo_x, robo_y, angle_err, angular_vel)
        print(f"current pos: {robo_x}, {robo_y}")
        ep_chassis.drive_speed(x=linear_vel, y=0, z=angular_vel, timeout = 0.1)
        time.sleep(0.1)
        
        
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout = 0.1)
    print("Reached Goal!")
    pass

def move():
    
    dist_tolerance = 0.1
    
    linear_vel = 0.5
    while True:
        # print("dist_err: ", dist_err)
        
        dist_err = euclidean_dist(robo_x, robo_y, x, y)
        linear_vel = 0.5
        angle_to_goal = math.atan2(y - robo_y, x - robo_x)
        angle_err = angle_to_goal - robo_t
        if angle_err > math.pi:
            angle_err -= 2*math.pi
        elif angle_err < -math.pi:
            angle_err += 2*math.pi
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
    robo_t = (yaw*math.pi)/180
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

# RGB Mask
sensitivity = 55
lower_bound = np.array([100, 17, 255 - sensitivity])
upper_bound = np.array([255, 100, 255])


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
    linear_vel = 0.3
    
    ang_f = (linear_vel * 1.5) + 1.0

    try:
        reached = False  
        drive = 1
        stop = 0
        for i in range(0, 5000):
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            cv2.imshow("lol", img)
        
            # continue
            # converting image to hsv
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # creating mask
            mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
            # 
            # applying mask to img
            img = cv2.bitwise_and(img, img, mask=mask)
            
            # cropping image
            height = 720
            width = 1280
            middle = width/2

            crop_percentage = 0.4
            cropped_height = int(height * (1 - crop_percentage))
            img = img[:height, :]
            mask = mask[:height]
            
            # blackout all the pixels except the mask
            mask[:, 450:750] = 0

            
            # finding contoures
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)        
            if len(contours) > 0:
                if (stop == 0):
                    # for contour in contours:
                    contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
                    if len(contours) == 1:
                        
                        contour1 = contours_sorted[0]
                        x, y, w, h = cv2.boundingRect(contour1)
                        # x2, y2, w2, h2 = width/4, width/4,width/4, width/4
                        x2, y2, w2, h2 = 0, 0, 0, 0

                        # contour2 = None
                    else:
                        
                        contour1 = contours_sorted[0]
                        contour2 = contours_sorted[1]
                        
                        x, y, w, h = cv2.boundingRect(contour1)
                        # h, w = 30, 30
                        x2, y2, w2, h2 = cv2.boundingRect(contour2)
                        # w2, h2 = 30, 30
                    
                    if x < x2:
                        left_edge = x + w
                        right_edge = x2
                    else:
                        left_edge = x2 + w2
                        right_edge = x
                        
                    # left_edge = x if x < x2 else x2
                    
                    # right_edge = x + w if x > x2 else x2+w
                    center = (right_edge - left_edge)/2
                    center += left_edge
                    
                    cv2.line(img, (int(center), 0), (int(center), 720), (255, 255, 0), 2)
                    cv2.line(img, (int(middle), 0), (int(middle), 720), (255, 0, 0), 2)
                        # x, y, w, h = cv2.boundingRect(contour)
                    cv2.line(img, (int(left_edge), 0), (int(left_edge), 720), (255, 100, 0), 2)
                    cv2.line(img, (int(right_edge), 0), (int(right_edge), 720), (255, 100, 0), 2)
                            
                    # drawing bounding box
                    
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.rectangle(img, (x2, y2), (x2+w2, y2+h2), (0, 255, 0), 2)
                    dist = get_distance(w)
                    angle = get_angle(x + (w/2))
                        
                    x_from_robo = dist*np.cos(angle)
                    y_from_robo = dist*np.sin(angle)
                    global_x = x_from_robo + robo_x
                    global_y = y_from_robo + robo_y
                        
                    # goto_goal(global_x, global_y)
                    # time.sleep(0.1)
                    diff = (center - middle)
                    
                    angular_vel = diff * ang_f
                    middle -= center*0.5                  
                    # ep_chassis.drive_speed(x=linear_vel, y=0, z=angular_vel, timeout=5)
            else:
                stop = 1
            # cv2.imshow("Masked Object Detection", img)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    
    
    except:
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()
    ep_robot.close()


