import rospy
import geometry_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg
import yaml
import time
import actionlib
import moveit_commander
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import sin, cos
from socket import *
IP="kd-pc29.local"
PORT=8080

s = socket(AF_INET, SOCK_STREAM)
try:
    s.connect((IP, PORT))
except error:
    s = None

class BaseControl(object):
    """ Move base and navigation """

    def __init__(self):
        ## Create publisher to move the base
        self._pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        ##action client for navigation
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        if not self.client.wait_for_server():
            rospy.logerr("Could not connect to move_base... Did you roslaunch fetch_navigation fetch_nav.launch map_file:=/home/fetch_admin/map_directory/5427_updated.yaml?")
        else:
            rospy.loginfo("Got move_base")

        self.actual_positions = (0, 0, 0)
        self.actual_vel = (0, 0, 0)

        ## subscribe to odom to get the robot current position
        def callback(msg):
            p = msg.pose.pose.position
            self.actual_positions = (p.x, p.y, p.z)
        self._sub = rospy.Subscriber("/odom", Odometry, callback)
        self.actual_positions = (0, 0, 0)

        while self._pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_forward(self, speed=0.2):
        tw = geometry_msgs.msg.Twist()
        tw.linear.x =abs(speed)
        self._pub.publish(tw)
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_backward(self, speed=-0.2):
         tw = geometry_msgs.msg.Twist()
         tw.linear.x = -abs(speed)
         self._pub.publish(tw)
         if s:
             s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_left(self, angle=0.2):
        tw = geometry_msgs.msg.Twist()
        tw.angular.z = abs(angle)
        self._pub.publish(tw)
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def move_right(self, angle=-0.2):
        tw = geometry_msgs.msg.Twist()
        tw.angular.z = -abs(angle)
        self._pub.publish(tw)
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    ## get 3D position of the robot
    def get_pose(self):
        return self.actual_positions

    def __del__(self):
        self._pub.publish(geometry_msgs.msg.Twist())
        move_goal = MoveBaseGoal()
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

    ##### Navigation
    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(move_goal)
        self.client.wait_for_result()
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

    def goto_relative(self, dx, dy, dtheta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = self.actual_positions[0] + dx
        move_goal.target_pose.pose.position.y = self.actual_positions[1] + dy
        move_goal.target_pose.pose.orientation.z = self.actual_positions[2] + sin(dtheta/2.0)
        move_goal.target_pose.pose.orientation.w = self.actual_positions[2] + cos(dtheta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(move_goal)
        self.client.wait_for_result()
        if s:
            s.send(",".join([str(d) for d in list(self.get_pose())]))

if __name__ == '__main__':
    rospy.init_node("test_base")
    base_control = BaseControl()
    
    base_control.goto([-3.686,-2.244,1.566])
    time.sleep(2)
    
    for i in range(0,10):
        base_control.move_forward()

    # base_control.move_right(45)
    #base_control.move_forward(1.0)
    #time.sleep(2)
    #base_control.move_forward()
    # base_control.move_right()
