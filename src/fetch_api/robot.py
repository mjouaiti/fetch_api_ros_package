from gripper_control import GripperControl
from speech_control import SpeechControl
from camera_modules import *
from torso_control import TorsoControl
from base_control import BaseControl
from head_control import HeadControl
from arm_control import ArmControl
#from perception import Perception

import moveit_commander
import rospy

class Robot():

    def __init__(self):
        self. = moveit_commander.RobotCommander()
        self.gripper = GripperControl()
        self.speaker = SpeechControl()
        self.rgbCamera = RGBCamera()
        self.yoloDetector = YoloDetector()
        self.torso = TorsoControl()
        self.base = BaseControl()
        self.head = HeadControl()
        self.arm = ArmControl()
        self.scene = Scene()
        rospy.loginfo("Robot initialised!")

    def getRobotState(self):
        return self.get_current_state().joint_state.name, self.get_current_state().joint_state.position

    def getBasePosition(self):
        return self.base.get_pose()

    def goto(self, x, y, theta ):
        self.base.goto(x, y, theta, "map")

    def getRGBImage(self):
        return self.rgbCamera.curr_image

    def getDetectionImage(self):
        return self.yoloDetector.detection_image
        
    def vertical_pick(self, coord_3d):
        self.gripper.open()
        self.arm.move_cartesian_position([coord_3d[0]+.01, coord_3d[1], coord_3d[2] + 0.3], [0, pi/2, 0])
        time.sleep(2.0)

        self.arm.move_cartesian_position([coord_3d[0]+.01, coord_3d[1], coord_3d[2] + 0.2], [0, pi/2, 0])
        time.sleep(2.0)
        self.gripper.close(CLOSED_POS, 40)
        time.sleep(2.0)#don't remove this sleep
        self.arm.move_cartesian_position([coord_3d[0]+.01, coord_3d[1], coord_3d[2] + 0.5], [0, pi/4, 0]) #raise object from the table
        time.sleep(2.0)
        self.arm.stow()
    
    def horizontal_pick(self, coord_3d):
        self.gripper.open()
        self.arm.move_cartesian_position([coord_3d[0] - 0.2, coord_3d[1], coord_3d[2] + 0.25], [0, 0, 0])
        time.sleep(2.0)

        self.arm.move_cartesian_position([coord_3d[0] - 0.2, coord_3d[1], coord_3d[2] + 0.04], [0, 0, 0])
        time.sleep(2.0)

        self.arm.move_cartesian_position([coord_3d[0]-0.1, coord_3d[1], coord_3d[2]+.02], [0, 0, 0])
        time.sleep(2.0)
        self.gripper.close(CLOSED_POS, 60)
        time.sleep(2.0)
        
         self.arm.stow()
        
    def horizontal_place(self, coord_3d):
        self.arm.move_cartesian_position([coord_3d[0] - 0.2, coord_3d[1], coord_3d[2] + 0.25], [0, 0, 0])
        time.sleep(2.0)

        self.arm.move_cartesian_position([coord_3d[0] - 0.2, coord_3d[1], coord_3d[2] + 0.04], [0, 0, 0])
        time.sleep(2.0)
        self.gripper.open()
        
    def vertical_place(self, coord_3d):
        self.arm.move_cartesian_position([coord_3d[0]+.01, coord_3d[1], coord_3d[2] + 0.3], [0, pi/2, 0])
        time.sleep(2.0)

        self.arm.move_cartesian_position([coord_3d[0]+.01, coord_3d[1], coord_3d[2] + 0.2], [0, pi/2, 0])
        self.gripper.open()
    

if __name__ == '__main__':
    rospy.init_node("robot")

    robot = Robot()
    print("Base position: ", robot.getBasePosition())
    print("Joint positions: ", robot.getRobotState())
