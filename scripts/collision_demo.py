from fetch_api.robot import Robot
import rospy
from numpy import pi
import thread
import time

if __name__ == '__main__':
    rospy.init_node("test_pick_place")

    robot = Robot()

    ###Uncomment if you want collision avoidance (recommended)

    robot.scene.clear()
    pause = [False]
    thread = thread.start_new_thread(robot.scene.update, (pause, ))
    time.sleep(3)
    pause[0] = True

    # robot.torso.move_to(.3)
    time.sleep(3)
    robot.arm.tuck()
    time.sleep(3)
    robot.arm.stow()
    time.sleep(3)
    robot.arm.tuck()
    time.sleep(2)
    robot.arm.stow()

    pause[0] = True
