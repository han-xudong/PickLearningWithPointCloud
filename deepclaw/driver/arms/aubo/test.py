from robotcontrol import Auboi5Robot
import time

Auboi5Robot.initialize()
robot = Auboi5Robot()
handle = robot.create_context()
ip = '192.168.0.100'
port = 8899
result = robot.connect(ip, port)
robot.robot_startup()
robot.set_collision_class(7)
joint_status = robot.get_joint_status()
robot.set_joint_maxvelc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))

robot.move_to_target_in_cartesian([0.5, 0.2, 0.2], [180, 0, 0])

    

