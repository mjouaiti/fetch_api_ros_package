from fetch_api.robot import Robot
import rospy
from numpy import pi
import thread
import time

if __name__ == '__main__':
    rospy.init_node("test_pick_place")

    robot = Robot()

    ###Uncomment if you want collision avoidance (recommended)

    # robot.scene.clear()
    # pause = [False]
    # thread = thread.start_new_thread(robot.scene.update, (pause, ))
    # time.sleep(2)
    #
    # pause[0] = True
    robot.torso.move_to(.3)
    # robot.arm.safe_tuck()
    # robot.head.move_home()
    # robot.head.move_down_center(0.6)
    time.sleep(2)
    robot.arm.safe_stow()

    #

    while 1:
        print(robot.yoloDetector.get_item_list())
        obj_name = raw_input("Which object do you want to pick?")
        # obj_name = "cup"
        coord_3d, up = robot.yoloDetector.get_item_3d_coordinates(obj_name, robot.rgbCamera.getRGBImage())
        print("Picking " + obj_name + ": ", coord_3d[0])
        coord_3d = coord_3d[0]

        robot.vertical_pick(coord_3d)

        robot.vertical_place([coord_3d[0], coord_3d[1], coord_3d[2]])
        break
