from fetch_api.robot import Robot
import rospy
import thread

if __name__ == '__main__':
    rospy.init_node("go_home")
    robot = Robot()
    robot.scene.clear()
    pause = [False]
    thread = thread.start_new_thread(robot.scene.update, (pause, ))

    robot.torso.move_to(0.3)
    robot.arm.tuck()
    robot.torso.move_to(0.0)
    robot.head.move_home()
