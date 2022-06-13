from fetch_api.robot import Robot
import rospy
from numpy import pi
import thread
from fetch_api.base_control import BaseControl

if __name__ == '__main__':
    rospy.init_node("test_pick_place")

    robot = Robot()
    base_control = BaseControl()

    ###Uncomment if you want collision avoidance (recommended)
    '''
    robot.scene.clear()
    pause = [False]
    thread = thread.start_new_thread(robot.scene.update, (pause, ))
    '''

    robot.torso.move_to(.3)
    robot.arm.safe_stow()
    robot.torso.move_to(.3)


    while 1:
        print(robot.yoloDetector.get_item_list())
        # obj_name = raw_input("Which object do you want to pick?")
        obj_name = "cup"
        coord_3d, up = robot.yoloDetector.get_item_3d_coordinates(obj_name, robot.rgbCamera.getRGBImage())
        print("Picking " + obj_name + ": ", coord_3d[0])
        coord_3d = coord_3d[0]
        coord_3d_initial=[coord_3d[0],coord_3d[1],coord_3d[2]]
        robot.head.look_at(coord_3d_initial[0],coord_3d_initial[1],coord_3d_initial[2])
        while coord_3d[0]>0.75:
            base_control.move_forward()
            robot.head.look_at(coord_3d_initial[0],coord_3d_initial[1],coord_3d_initial[2])
            coord_3d, up = robot.yoloDetector.get_item_3d_coordinates(obj_name, robot.rgbCamera.getRGBImage())
            print(coord_3d)
            if len(coord_3d)==0:
                coord_3d=history
            history=coord_3d
            try:
                coord_3d = coord_3d[0]
            except:
                continue
        robot.head.look_at(coord_3d_initial[0],coord_3d_initial[1],coord_3d_initial[2]-0.45)
        coord_3d, up = robot.yoloDetector.get_item_3d_coordinates(obj_name, robot.rgbCamera.getRGBImage())
        print("Picking " + obj_name + ": ", coord_3d)
        coord_3d = coord_3d[0]
        robot.vertical_pick(coord_3d)

        robot.vertical_place([coord_3d[0]+0.02, coord_3d[1], coord_3d[2]])
        break
