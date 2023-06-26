#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
from std_msgs.msg import Bool

class GenerateTrajectory:
    def __init__(self):
        rospy.init_node('generate_trajectory')
        self.top_right_point = PoseStamped()
        self.top_left_point = PoseStamped()
        self.current_pose = PoseStamped()

        self.z_step = 1
        self.y_step = 36
        self.z_stop = 6 #6

        self.right_to_left_movement = True

        # Publishers and subscribers
        rospy.Subscriber('/red/pose', PoseStamped, self.pose_callback)
        self.input_pose_pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=1, latch=True)

        self.finish_publisher = rospy.Publisher('/scanning_finished', Bool, queue_size=10)
        self.finished_msg = Bool()

    def pose_callback(self, msg):
        self.current_pose = msg

    def go_to_position(self, x, y, z):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        # Publish the goal pose
        self.input_pose_pub.publish(goal_pose)

        # Wait for the drone to reach the goal pose
        while not self.is_close_to_goal(goal_pose):
            rospy.sleep(0.1)

    def is_close_to_goal(self, goal_pose):
        tolerance = 0.1  # Tolerance in meters

        if (
            abs(self.current_pose.pose.position.x - goal_pose.pose.position.x) <= tolerance and
            abs(self.current_pose.pose.position.y - goal_pose.pose.position.y) <= tolerance and
            abs(self.current_pose.pose.position.z - goal_pose.pose.position.z) <= tolerance
        ):
            return True

        return False

    def generate_trajectory(self):
        z_values_range = np.arange(self.top_left_point.pose.position.z, self.z_stop - self.z_step, -self.z_step).tolist()
        y_from_right_to_left = np.arange(self.top_right_point.pose.position.y, self.top_left_point.pose.position.y + self.y_step, self.y_step).tolist()
        y_from_left_to_right = np.arange(self.top_left_point.pose.position.y, self.top_right_point.pose.position.y - self.y_step, -self.y_step).tolist()
        
        for z in z_values_range:
            if z < 9: #9
                x = self.top_right_point.pose.position.x - 1.5
            else:
                x = self.top_right_point.pose.position.x
            if self.right_to_left_movement:
                for y in y_from_right_to_left:
                    self.go_to_position(x, y, z)
                    time.sleep(1)  # Stop for 1 second
                self.right_to_left_movement = False
            else:
                for y in y_from_left_to_right:
                    self.go_to_position(x, y, z)
                    time.sleep(1)  # Stop for 1 second
                self.right_to_left_movement = True

        #Send message that trajectory generator is done        
        self.finished_msg.data = True
        self.finish_publisher.publish(self.finished_msg)

    def run(self):
        rospy.loginfo('Generate trajectory node is running.')
        self.generate_trajectory()
        rospy.spin()


if __name__ == '__main__':
    generate_trajectory = GenerateTrajectory()
    top_right_corner = [7.5, -8, 12] 
    top_left_corner = [7.5, 28, 12]

    generate_trajectory.top_right_point.pose.position.x = top_right_corner[0]
    generate_trajectory.top_right_point.pose.position.y = top_right_corner[1]
    generate_trajectory.top_right_point.pose.position.z = top_right_corner[2]

    generate_trajectory.top_left_point.pose.position.x = top_left_corner[0]
    generate_trajectory.top_left_point.pose.position.y = top_left_corner[1]
    generate_trajectory.top_left_point.pose.position.z = top_left_corner[2]

    generate_trajectory.run()