from fetch_api.robot import Robot
import rospy
import time
import thread

if __name__ == '__main__':
    rospy.init_node("go_home")
    robot = Robot()
    obj_name = raw_input("Which object do you want to pick?")
    print(robot.yoloDetector.get_item_list())
    coord_3d, up = robot.yoloDetector.get_item_3d_coordinates(obj_name, robot.rgbCamera.getRGBImage())
    print("Picking " + obj_name + ": ", coord_3d[0])

    #
    # robot.scene.clear()
    # pause = [False]
    # thread = thread.start_new_thread(robot.scene.update, (pause, ))
    #
    # robot.torso.move_to(0.3)
    # robot.arm.safe_stow()
    # pause[0] = [True]
    # time.sleep(2)
    # pause[0] = [False]
    # robot.arm.safe_tuck()
    # robot.scene.clear()
    # pause = [False]
    # thread = thread.start_new_thread(robot.scene.update, (pause, ))
    #
    # robot.torso.move_to(0.3)
    # robot.arm.tuck()
    # robot.torso.move_to(0.0)
    # robot.head.move_home()
