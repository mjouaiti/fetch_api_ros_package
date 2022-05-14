from fetch_api.robot import Robot
import rospy
from numpy import pi
import thread

if __name__ == '__main__':
    rospy.init_node("test_pick_place")

    robot = Robot()
    
    ###Uncomment if you want collision avoidance (recommended)
    '''
    robot.scene.clear()
    pause = [False]
    thread = thread.start_new_thread(robot.scene.update, (pause, ))
    '''

    robot.torso.move_to(.3)
    robot.arm.stow()

    while 1:
        print(yolo.get_item_list())
        obj_name = raw_input("Which object do you want to pick?")
        coord_3d, up = yolo.get_item_3d_coordinates(obj_name, rgb.getRGBImage())
        print("Picking " + obj_name + ": ", coord_3d, up)
        coord_3d = coord_3d[0]

        robot.vertical_pick(coord_3d)
        
        robot.vertical_place([coord_3d[0]-0.02, coord_3d[1]+0.2, coord_3d[2]])
