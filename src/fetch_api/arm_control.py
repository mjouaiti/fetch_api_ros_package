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
import sys
import geometry_msgs.msg

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
        moveit_commander.roscpp_initialize(sys.argv)#new planning codes
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.move_group = MoveGroupInterface("arm", "base_link")
        self.eef_link = self.arm_group.get_end_effector_link()#new planning codes
        self.group_names = self.robot.get_group_names()#new planning codes



        print(
            self.arm_group.has_end_effector_link(), self.arm_group.get_end_effector_link(), self.arm_group.get_joints())
        self.arm_group.set_planning_time(5)
        self.arm_group.set_planner_id("RRTConnectkConfigDefault")
        self.arm_group.allow_replanning(True)
        self.arm_group.clear_path_constraints()
        self.scene = moveit_commander.PlanningSceneInterface()#new planning codes

        self.box_name = None
        self.tuck_joint_value = {}
        self.stow_joint_value = {}
        # self.inter_stow_joint_value = {}
        self.zero_joint_value = {}
        self.tuck_values = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        self.stow_values = [1.6, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        #self.inter_stow_values = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0]
        self.zero_values = [0, 0, 0, 0, 0, 0, 0]
        for j, joint in enumerate(self.joint_names):
            self.tuck_joint_value[joint] = self.tuck_values[j]
            self.stow_joint_value[joint] = self.stow_values[j]
            #self.inter_stow_joint_value[joint] = self.inter_stow_values[j]
            self.zero_joint_value[joint] = self.zero_values[j]

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
            if not blocking:
                return
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.pause = [True]
                return


    def move_joint_position(self, joint_name, position, blocking=True):
        self.pause[0] = False
        positions = list(self.get_pose())
        positions[self.joint_names.index(joint_name)] = position
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(self.joint_names, positions, wait=blocking, max_velocity_scaling_factor=0.2)
            if s:
                s.send(",".join([str(d) for d in list(self.get_pose())]))
            if not blocking:
                return
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

    def safe_tuck(self, planning=True, **kwargs):
        p1 = self.add_box()
        print('p1', p1)
        time.sleep(1)
        p2 = self.attach_box()
        print('p2', p2)
        # time.sleep(1)
        if planning:
            result = self.plan_joint_position(self.tuck_joint_value)
        else:
            if kwargs is not None:
                if "max_velocity_scaling_factor" not in kwargs:
                    kwargs["max_velocity_scaling_factor"] = 0.05
            else:
                kwargs = {"max_velocity_scaling_factor": 0.05}
            # while not rospy.is_shutdown():
            plan = self.move_group.moveToJointPosition(self.joint_names, self.tuck_values,
                                                       0.02, **kwargs)
            result = plan.error_code

        p3 = self.detach_box()
        print('p3', p3)
        p4 = self.remove_box()
        print('p4', p4)
        print('plan', result)


    def zero(self):
        self.pause[0] = False
        self.move_joint_positions(self.joint_names, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pause[0] = True

    def stow(self):
        self.pause[0] = False
        self.move_joint_positions([1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0])
        self.pause[0] = True

    def safe_stow(self, planning=True, **kwargs):
        p1 = self.add_box()
        print('p1', p1)
        # time.sleep(1)
        p2 = self.attach_box()
        print('p2', p2)
        # time.sleep(1)
        if planning:
            result = self.plan_joint_position(self.stow_joint_value)
        else:
            if kwargs is not None:
                if "max_velocity_scaling_factor" not in kwargs:
                    kwargs["max_velocity_scaling_factor"] = 0.05
            else:
                kwargs = {"max_velocity_scaling_factor": 0.05}
            # while not rospy.is_shutdown():
            plan = self.move_group.moveToJointPosition(self.joint_names, self.stow_values,
                                                       0.02, **kwargs)
            result = plan.error_code
        p3 = self.detach_box()
        print('p3', p3)
        p4 = self.remove_box()
        print('p4', p4)
        print(result)

    def intermediate_stow(self):
        self.pause[0] = False
        self.move_joint_positions(self.joint_names, [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0])
        self.pause[0] = True

    def get_pose(self):
        self.actual_positions = self.robot.get_current_state().joint_state.position[6:13]
        return self.actual_positions

    def _del_(self):
        self.tuck()
    #==================================================

    def plan_joint_position(self, joint_angles):
        self.arm_group.clear_pose_targets()
        self.arm_group.set_joint_value_target(joint_angles)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        return plan

    def add_box(self, timeout=4):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "wrist_roll_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 1.5
        self.box_name = 'box'
        self.scene.add_box(self.box_name, box_pose, size=(0.22, 0.19, 0.15))  # (0.22, 0.19, 0.15)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        grasping_group = 'gripper'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        self.scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=8):

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = self.box_name in self.scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

def to_radian(degree):  # helper function does not work
    rad = round((degree / 180) * pi, 2)
    return rad


if __name__ == '__main__':
    import time

    initial_time = time.time()
    rospy.init_node("test_arm")
    from torso_control import TorsoControl
    from scene import Scene

    scene = Scene()
    scene.clear()
    torso = TorsoControl()
    arm_module = ArmControl()
    # torso.move_to(0.3)
    # time.sleep(2.)
    #
    # arm_module.stow()

    torso.move_to(0.)
    arm_module.move_cartesian_position([0.5478926483367018, 0.16955347545496427, 0.25], [0, pi / 2, 0.0]) #pick up off the floor
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



    # coord_3d = [0.65, -0.1, 0.84] #center  # [0.6478926483367018, -0.16955347545496427, 0.8426119154023802]
    # coord_3d_b = [0.65, 0.3, 0.6] # human right
    # coord_3d_c= [0.65, -0.3, 0.6]  # human left

    # arm_module.tuck()
    # arm_module.move_cartesian_position([coord_3d[0], coord_3d[1], coord_3d[2]], [pi / 2, 0.0, 0.0])


    # y left to right, x into the page and out of the page, z height, angle of the gripper roll, pitch, yaw in rads, yaw may needs to be dynamic
    # print("somehow reached goal even though it makes no sense")
    # time.sleep(15)
    # arm_module.tuck()
    # time.sleep(5)
    #torso.move_to(0.0)
    # arm_module.scene.clear()
