import rospy
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction
from moveit_python import MoveGroupInterface
import actionlib
from geometry_msgs.msg import Pose
# from motion_planning_msg.msg import SimplePoseConstraint
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes, OrientationConstraint, Constraints, MotionPlanRequest
from scene import *
from tf.transformations import quaternion_from_euler
from math import pi
import thread
from socket import *
import math
# from torso_control import TorsoControl
# from head_control import HeadControl
# from gripper_control import GripperControl

IP = "kd-pc29.local"
PORT = 8080

s = socket(AF_INET, SOCK_STREAM)
try:
    s.connect((IP, PORT))
except error:
    s = None



class ArmControl(object):

    """ Move arm """

    def __init__(self):
        self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm controller....")
        self.client.wait_for_server()
        if self.client.wait_for_server():
            rospy.loginfo("Got arm controller")
        else:
            rospy.logerr(
                "Couldn't connect to arm controller.... Did you roslaunch fetch_moveit_config move_group.launch?")

        self.actual_positions = [0, 0, 0, 0, 0, 0, 0]
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                            "upperarm_roll_joint", "elbow_flex_joint",
                            "forearm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.move_group = MoveGroupInterface("arm", "base_link")

        print(
            self.arm_group.has_end_effector_link(), self.arm_group.get_end_effector_link(), self.arm_group.get_joints())
        self.arm_group.set_planning_time(20)
        self.arm_group.set_planner_id("RRTConnectkConfigDefault")
        self.arm_group.allow_replanning(True)
        self.arm_group.clear_path_constraints()

        self.pause = [True]
        '''
        self.scene = Scene(PERCEPTION_MODE)
        self.scene.clear()
        self.thread = thread.start_new_thread(self.scene.update, (self.pause, ))
	'''
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_joint_positions(self, positions, blocking=True):
        self.pause[0] = False
        while not rospy.is_shutdown():
            # result = self.move_group.moveToJointPosition(self.joint_names, positions,{'max_velocity_scaling_factor':0.5},wait=blocking)
            result = self.move_group.moveToJointPosition(self.joint_names, positions,wait=blocking, max_velocity_scaling_factor=0.5)
            if s:
                s.send(",".join([str(d) for d in list(self.get_pose())]))
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.pause = [True]
                return


    def move_joint_position(self, joint_name, position, blocking=True):
        self.pause[0] = False
        positions = list(self.get_pose())
        positions[self.joint_names.index(joint_name)] = position
        while not rospy.is_shutdown():
            # result = self.move_group.moveToJointPosition(self.joint_names, positions, {'max_velocity_scaling_factor':0.5},wait=blocking)
            result = self.move_group.moveToJointPosition(self.joint_names, positions, wait=blocking, max_velocity_scaling_factor=0.5)
            if s:
                s.send(",".join([str(d) for d in list(self.get_pose())]))
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.pause[0] = True
                return

    def move_cartesian_position(self, position, orientation=[0.0, 0.0, 0.0]):
        ''' position: xyz
            orientation: rpy'''
        self.pause[0] = False
        self.arm_group.set_start_state(self.robot.get_current_state())
        quaternion = quaternion_from_euler(orientation[0], orientation[1], orientation[2])

        pose_target = Pose()
        pose_target.position.x = position[0]
        pose_target.position.y = position[1]
        pose_target.position.z = position[2]
        pose_target.orientation.w = quaternion[3]
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]

        # constraints = Constraints()
        # constraints.name = "Keep gripper horizontal"
        #
        # orientation_constraint = OrientationConstraint()
        # orientation_constraint.header = self.arm_group.get_current_pose(self.arm_group.get_end_effector_link()).header
        # orientation_constraint.link_name = self.arm_group.get_end_effector_link()
        #

        # orientation_constraint.orientation.w = quaternion[3]
        # orientation_constraint.orientation.x = quaternion[0]
        # orientation_constraint.orientation.y = quaternion[1]
        # orientation_constraint.orientation.z = quaternion[2]
        #
        # orientation_constraint.absolute_x_axis_tolerance = 0.2
        # orientation_constraint.absolute_y_axis_tolerance = 0.2
        # orientation_constraint.absolute_z_axis_tolerance = 0.2
        # orientation_constraint.weight = 1.0
        # constraints.orientation_constraints.append(orientation_constraint)
        # self.arm_group.set_path_constraints(constraints)

        result = self.arm_group.set_pose_target(pose_target)
        plan = self.arm_group.go(wait=True)
        # result = self.arm_group.set_rpy_target([.0,.0,.0])
        # plan = self.arm_group.go(wait=True)

        self.arm_group.stop()
        # self.arm_group.clear_path_constraints()
        self.pause[0] = True
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def tuck(self): # local method
        self.pause[0] = False
        self.move_joint_positions([1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0])
        self.pause[0] = True

    def zero(self):
        self.pause[0] = False
        self.move_joint_positions(self.joint_names, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pause[0] = True

    def stow(self):
        self.pause[0] = False
        self.move_joint_positions([1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0])
        self.pause[0] = True

    def intermediate_stow(self):
        self.pause[0] = False
        self.move_joint_positions(self.joint_names, [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0])
        self.pause[0] = True

    def get_pose(self):
        self.actual_positions = self.robot.get_current_state().joint_state.position[6:13]
        return self.actual_positions

    def _del_(self):
        self.tuck()


def to_radian(degree):  # helper function does not work
    rad = round((degree / 180) * pi, 2)
    return rad


if __name__ == '__main__':
    import time

    initial_time = time.time()
    rospy.init_node("test_arm")
    from torso_control import TorsoControl

    torso = TorsoControl()
    arm_module = ArmControl()
    torso.move_to(0.3)
    time.sleep(2.)

    arm_module.tuck()
    torso.move_to(0.2)

    # arm_module.move_cartesian_position([0.5478926483367018, 0.16955347545496427, 0.5426119154023802])
    #
    # pose = list(arm_module.get_pose())
    # pose[-2] += 0.5
    # s.send(",".join([str(d) for d in list(pose)]))

    # arm_module.tuck()
    # arm_module.stow()
    # torso.move_to(0.4)
    # arm_module.tuck()
    # time.sleep(5)
    # arm_module.move_joint_positions(
    #     [1.6, -0.3, -0.2, -1.6, 1.2, 0, 0])  # move the elbow_flex_joint with a sinusoidal function center around -1.6
    # time.sleep(5)
    # arm_module.move_joint_position("shoulder_pan_joint", .5)
    # time.sleep(5)



    coord_3d = [0.8, 0, 0.5] #center  # [0.6478926483367018, -0.16955347545496427, 0.8426119154023802]
    coord_3d_b = [0.65, 0.3, 0.6] # human right
    coord_3d_c= [0.65, -0.3, 0.6]  # human left

    # arm_module.tuck()
    # arm_module.move_cartesian_position([coord_3d[0], coord_3d[1], coord_3d[2]], [pi / 2, 0.0, 2 * pi + 0.01])


    # y left to right, x into the page and out of the page, z height, angle of the gripper roll, pitch, yaw in rads, yaw may needs to be dynamic
    # print("somehow reached goal even though it makes no sense")
    # time.sleep(15)
    # arm_module.tuck()
    # time.sleep(5)
    #torso.move_to(0.0)
    # arm_module.scene.clear()
