#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Global variables
positions = []  # Store robot positions
scanning_finished = False  # Flag indicating if scanning is finished

# Callback function for pose messages
def pose_callback(msg):
    global positions, scanning_finished
    if not scanning_finished:
        positions.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

# Callback function for scanning finished message
def scanning_finished_callback(msg):
    global scanning_finished
    scanning_finished = msg.data

# Initialize ROS node and subscribers
rospy.init_node('position_recorder')
rospy.Subscriber('/red/pose', PoseStamped, pose_callback)
rospy.Subscriber('/scanning_finished', Bool, scanning_finished_callback)

# Spin ROS event loop
rate = rospy.Rate(10)  # Adjust the rate as per your requirements
while not rospy.is_shutdown() and not scanning_finished:
    rate.sleep()

# Plot positions
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Set axis ranges
ax.set_xlim(11, 5)
ax.set_ylim(30, -10)
ax.set_zlim(0, 13)

x_positions, y_positions, z_positions = zip(*positions)
ax.plot(x_positions, y_positions, z_positions)
ax.set_xlabel('X os [m]')
ax.set_ylabel('Y os [m]')
ax.set_zlabel('Z os [m]')
ax.set_title('Pozicija letjelice')
fig.savefig('Path_visualizer.png')
