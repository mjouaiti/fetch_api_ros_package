# Author: H J Kashyap, T Hwu
import rospy
import trajectory_msgs.msg
import control_msgs.msg
import time
import actionlib
import moveit_commander
from socket import *
IP="kd-pc29.local"
PORT=8080

from control_msgs.msg import PointHeadAction, PointHeadGoal

s = socket(AF_INET, SOCK_STREAM)
try:
    s.connect((IP, PORT))
except error:
    s = None

class HeadControl(object):
    """ Head control interface """

    def __init__(self):
        self.actual_positions = (0, 0)

        self.robot = moveit_commander.RobotCommander()

        ## initialize head action client
        self.cli_head = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        ## wait for the action server to establish connection
        rospy.loginfo("Waiting for head controller....")
        if not self.cli_head.wait_for_server():
            rospy.logerr("Couldn't connect to head controller... ")
        else:
            rospy.loginfo("Got head controller ")
        self.cli_head_cart = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        if not self.cli_head_cart.wait_for_server():
            rospy.logerr("Couldn't connect to cartesian head controller... ")
        else:
            rospy.loginfo("Got cartesian head controller ")
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_head(self, pan_pos, tilt_pos, time=rospy.Time(1.0), blocking=True):
        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [pan_pos, tilt_pos]
        p.velocities = [0, 0]
        p.time_from_start = time
        traj.points = [p]
        goal.trajectory = traj
        self.cli_head.send_goal(goal)
        if blocking:
            self.cli_head.wait_for_result()
        rospy.sleep(1)
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def look_at(self, x, y, z, duration=1.0, blocking=True):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = "base_link"
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.cli_head_cart.send_goal(goal)
        if blocking:
            self.cli_head_cart.wait_for_result()

    def move_down_center(self, offset=0.5):
        self.get_pose()
        self.move_head(0.0, self.actual_positions[1]+abs(offset), rospy.Time(.4))


    def head_sweep(self,pan_pos):
        self.move_head(pan_pos, 0.0, rospy.Time(3))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_up(self, offset=0.5):
        self.get_pose()
        self.move_head(self.actual_positions[0], self.actual_positions[1]-abs(offset), rospy.Time(.4))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_down(self, offset=0.5):
        self.get_pose()
        self.move_head(self.actual_positions[0], self.actual_positions[1]+abs(offset), rospy.Time(.4))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_left(self, offset=0.5):
        self.get_pose()
        self.move_head(self.actual_positions[0]+abs(offset), self.actual_positions[1], rospy.Time(.4))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_right(self, offset=0.5):
        self.get_pose()
        self.move_head(self.actual_positions[0]-abs(offset), self.actual_positions[1], rospy.Time(.4))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_home(self):
        self.move_head(0.0, 0.0, rospy.Time(1.5))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_down_center(self, offset=0.5):
        self.get_pose()
        self.move_head(0.0, self.actual_positions[1]+abs(offset), rospy.Time(.4))

    def move_left_45(self):
        self.get_pose()
        self.move_head(self.actual_positions[0]+0.775, self.actual_positions[1], rospy.Time(2.5))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_right_45(self):
        self.get_pose()
        self.move_head(self.actual_positions[0]-0.775, self.actual_positions[1], rospy.Time(2.5))
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def get_pose(self):
        self.actual_positions = self.robot.get_current_state().joint_state.position[4:6]
        return self.actual_positions

    # def __del__(self):
        # traj = trajectory_msgs.msg.JointTrajectory()
        # goal.trajectory = traj
        # self.cli_head.send_goal(goal)
        # self.cli_head.wait_for_result()
        # rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node("test_head")
    head_control = HeadControl()
    head_control.look_at(0.1,0.2,1.5)
    time.sleep(3)
    # head_control.move_left()
    # time.sleep(2)
    # head_control.move_home()
    # time.sleep(2)
    # head_control.move_right()
    # head_control.head_sweep(0.5)
    # time.sleep(3)
    # head_control.head_sweep(-0.5)
    head_control.move_home()

    # head_control.move_down()
    # time.sleep(3)
    # head_control.move_left()
    # time.sleep(2)
    # head_control.move_home()
    # time.sleep(2)
    # head_control.move_right()
    # time.sleep(3)
    # head_control.move_home()
