import time
import threading
import keyboard
from robomaster import robot

# Define global variables
x_val = 0.0
y_val = 0.0
z_val = 0.0
speed = 1.5
z_speed = 100
running = True

# Function to read keyboard input
def keyboard_control():
    global x_val, y_val, z_val, running

    while running:
        if keyboard.is_pressed('w'):
            x_val = speed
            
        elif keyboard.is_pressed('s'):
            x_val = -speed
        else:
            x_val = 0.0

        if keyboard.is_pressed('a'):
            z_val = -z_speed
        elif keyboard.is_pressed('d'):
            z_val = z_speed
        else:
            z_val = 0.0

        if keyboard.is_pressed(' '):  # Stop
            x_val = 0.0
            y_val = 0.0
            z_val = 0.0

        time.sleep(0.1)
def sub_attitude_info_handler(info):
    yaw, pith, roll = info
    print("theta: ", yaw)
def sub_position_info_handler(info):
    x, y, z, = info
    print(f"position: x:{round(x,3)}, y:{round(y,3)}, z:{round(z,3)}")
# Main function
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    # ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_chassis.sub_position(freq=10, callback=sub_position_info_handler)

    # Start keyboard listener in a separate thread
    keyboard_thread = threading.Thread(target=keyboard_control)
    keyboard_thread.start()

    try:
        
        while True:
            # Update robot movement
            ep_chassis.drive_speed(x=x_val, y=y_val, z=z_val, timeout=0.1)
            time.sleep(0.1)
            if keyboard.is_pressed(" "):
                running = False
                break
        ep_robot.close()
    except KeyboardInterrupt:
        ep_robot.close()
        running = False
        keyboard_thread.join()
