from fetch_api.robot import Robot

if __name__ == '__main__':
    rospy.init_node("go_home")
    robot = Robot()
    
    robot.torso.move_to(0.3)
    robot.arm.tuck()
    robot.torso.move_to(0.0)
    robot.head.move_home()
