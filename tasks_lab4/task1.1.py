from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    x_val = 1


    ep_chassis.move(x=-x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()


    ep_robot.close()
