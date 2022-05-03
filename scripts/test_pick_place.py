from fetch_api.pick_place import PickPlace


if __name__ == '__main__':
    rospy.init_node("test_pick_place")

    from fetch_api.arm_control import ArmControl
    from fetch_api.camera_modules import *
    from fetch_api.torso_control import TorsoControl
    from fetch_api.gripper_control import GripperControl, CLOSED_POS
    
    arm = ArmControl()
    yolo = YoloDetector()
    rgb = RGBCamera()
    gripper = GripperControl()
    torso = TorsoControl()

    torso.move_to(.3)
    arm.stow()
    gripper.open()

    while 1:
        print(yolo.get_item_list())
        obj_name = raw_input("Which object do you want to pick?")
        coord_3d, up = yolo.get_item_3d_coordinates(obj_name, rgb.getRGBImage())
        print("Picking " + obj_name + ": ", coord_3d, up)
        coord_3d = coord_3d[0]

        arm.move_cartesian_position([coord_3d[0] - 0.2, coord_3d[1], coord_3d[2] + 0.3], [0, 0, 0])
        time.sleep(1.0)
        
        arm.move_cartesian_position([coord_3d[0] - 0.2, coord_3d[1], coord_3d[2] + 0.1], [0, 0, 0])
        time.sleep(1.0)
        
        arm.move_cartesian_position([coord_3d[0]-0.05, coord_3d[1], coord_3d[2]+.1], [0, pi/2, 0])
        time.sleep(1.0)
        gripper.close(CLOSED_POS, 60)
        time.sleep(2.0)

        arm.stow()
