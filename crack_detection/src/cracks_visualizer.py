#!/usr/bin/env python3
import pickle
import rospy
import tf
from vision_msgs.msg import Detection2DArray
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt


class CrackVisualizer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('crack_visualizer')

        # Subscribe to the crack detection topic
        rospy.Subscriber('/yolov7/yolov7', Detection2DArray, self.callback_detections)

        # Subscribe to the depth camera topic
        rospy.Subscriber('/red/camera/depth/image_raw', Image, self.callback_depth)

        rospy.Subscriber('/scanning_finished', Bool, self.scanning_finished_callback)
        self.scanning_finished = False

        # Create a publisher for the voxel map
        #self.voxel_map_pub = rospy.Publisher('/cracks_voxel_map_topic', Octomap, queue_size=10)

        # Define the camera matrix using the intrinsic parameters
        self.intrinsic_matrix = np.array([[381.36246688113556, 0.0, 320.5],
                                            [0.0, 381.36246688113556, 240.5],
                                            [0.0, 0.0, 1.0]])

        # Create a TF listener
        self.listener = tf.TransformListener()

        # Wait for the transformation to become available
        self.listener.waitForTransform('world', 'red/camera', rospy.Time(), rospy.Duration(4.0))

        # Define voxel grid
        x_range = (5, 11)
        y_range = (-10, 30)
        z_range = (0, 13)
        resolution = 0.5

        self.x_bins = np.arange(x_range[0], x_range[1] + resolution, resolution)
        self.y_bins = np.arange(y_range[0], y_range[1] + resolution, resolution)
        self.z_bins = np.arange(z_range[0], z_range[1] + resolution, resolution)

        self.voxel_votes = np.zeros((len(self.x_bins)-1, len(self.y_bins)-1, len(self.z_bins)-1), dtype=int)
        self.detection_threshold = 11

    def scanning_finished_callback(self, msg):
        self.scanning_finished = msg.data
        if self.scanning_finished:
            crack_voxels = np.argwhere(self.voxel_votes >= 1)

            # Extract the coordinates and vote counts of crack voxels
            crack_voxel_coordinates = []
            crack_voxel_votes = []
            for crack_voxel in crack_voxels:
                x_index, y_index, z_index = crack_voxel
                x_center = (self.x_bins[x_index] + self.x_bins[x_index + 1]) / 2
                y_center = (self.y_bins[y_index] + self.y_bins[y_index + 1]) / 2
                z_center = (self.z_bins[z_index] + self.z_bins[z_index + 1]) / 2
                crack_voxel_coordinates.append((x_center, y_center, z_center))
                crack_voxel_votes.append(self.voxel_votes[x_index, y_index, z_index])

            # Creating figures
            fig = plt.figure(figsize=(10, 10))
            ax = fig.add_subplot(111, projection='3d')

            # Set axis ranges
            ax.set_xlim(11, 5)
            ax.set_ylim(30, -10)
            ax.set_zlim(0, 13)

            crack_voxel_votes = np.array(crack_voxel_votes)
            crack_voxel_votes[crack_voxel_votes > self.detection_threshold] = np.max(crack_voxel_votes)
            crack_voxel_votes = crack_voxel_votes.tolist()

            # Creating the 3D heatmap
            img = ax.scatter(
                [coord[0] for coord in crack_voxel_coordinates],
                [coord[1] for coord in crack_voxel_coordinates],
                [coord[2] for coord in crack_voxel_coordinates],
                marker='s', s=200, c=crack_voxel_votes/np.max(crack_voxel_votes), cmap='Greens', vmin=0, vmax=1
            )

            # Adding color bar
            color_bar = fig.colorbar(img, ax=ax, shrink=0.6)
            color_bar.set_label('Vjerojatnost pukotine')

            # Adding title and labels
            ax.set_title("3D graf vjerojatnosti postojanja pukotina")
            ax.set_xlabel('X os [m]')
            ax.set_ylabel('Y os [m]')
            ax.set_zlabel('Z os [m]')

            # Save the 3D heatmap
            fig.savefig('heatmap_3d.png')

            # Create a new figure for the YZ plane
            fig_yz = plt.figure(figsize=(16, 8))
            ax_yz = fig_yz.add_subplot(111)

            # Creating the YZ plane heatmap
            img_yz = ax_yz.scatter(
                [coord[1] for coord in crack_voxel_coordinates],
                [coord[2] for coord in crack_voxel_coordinates],
                marker='s', s=200, c=crack_voxel_votes/np.max(crack_voxel_votes), cmap='Greens', vmin=0, vmax=1
            )

            # Set axis ranges for YZ plane
            ax_yz.set_ylim(0, 13)
            ax_yz.set_xlim(30, -10)

            # Adding color bar
            color_bar_yz = fig_yz.colorbar(img_yz, ax=ax_yz, shrink=0.6)
            color_bar_yz.set_label('Vjerojatnost pukotine')

            # Adding title and labels
            ax_yz.set_title("Graf vjerojatnosti postojanja pukotina")
            ax_yz.set_xlabel('Y os [m]')
            ax_yz.set_ylabel('Z os [m]')

            # Save the YZ plane heatmap
            fig_yz.savefig('heatmap_yz.png')



            crack_voxels = np.argwhere(self.voxel_votes >= self.detection_threshold)

            # Extract the coordinates and vote counts of crack voxels
            crack_voxel_coordinates = []
            crack_voxel_votes = []
            for crack_voxel in crack_voxels:
                x_index, y_index, z_index = crack_voxel
                x_center = (self.x_bins[x_index] + self.x_bins[x_index + 1]) / 2
                y_center = (self.y_bins[y_index] + self.y_bins[y_index + 1]) / 2
                z_center = (self.z_bins[z_index] + self.z_bins[z_index + 1]) / 2
                crack_voxel_coordinates.append((x_center, y_center, z_center))
                crack_voxel_votes.append(self.voxel_votes[x_index, y_index, z_index])

            # # Combine neighboring detections within 0.5m at each coordinate
            # combined_crack_voxel_coordinates = []
            # combined_crack_voxel_votes = []
            # visited = set()
            # for i in range(len(crack_voxel_coordinates)):
            #     if i in visited:
            #         continue
            #     coord = crack_voxel_coordinates[i]
            #     votes = crack_voxel_votes[i]
            #     combined_coord = list(coord)
            #     combined_votes = votes

            #     for j in range(i + 1, len(crack_voxel_coordinates)):
            #         if j in visited:
            #             continue
            #         other_coord = crack_voxel_coordinates[j]
            #         if all(abs(coord[k] - other_coord[k]) <= 0.5 for k in range(3)):
            #             visited.add(j)
            #             for k in range(3):
            #                 combined_coord[k] = (combined_coord[k] + other_coord[k]) / 2
            #             combined_votes += crack_voxel_votes[j]

            #     combined_crack_voxel_coordinates.append(tuple(combined_coord))
            #     combined_crack_voxel_votes.append(combined_votes)

            # # Print the locations and vote counts of detected cracks
            # print('Scanning finished, printing detected cracks:')
            # for i, (coord, votes) in enumerate(zip(combined_crack_voxel_coordinates, combined_crack_voxel_votes)):
            #     x, y, z = coord
            #     print(f"Crack {i+1}: x={x}, y={y}, z={z}, Votes={votes}")
            
            #Print the locations and vote counts of detected cracks
            print('Scanning finished, printing detected cracks:')
            for i, (coord, votes) in enumerate(zip(crack_voxel_coordinates, crack_voxel_votes)):
                x, y, z = coord
                print(f"Crack {i+1}: x={x}, y={y}, z={z}, Votes={votes}")


    def callback_detections(self, detection_msg):
        # Extract crack locations from detection_msg
        cracks_locations = self.extract_crack_locations(detection_msg)

        for crack_location in cracks_locations:
            x_world, y_world, z_world = crack_location.point.x, crack_location.point.y, crack_location.point.z
            self.update_voxel_map(x_world, y_world, z_world)

    def callback_depth(self, depth_msg):
        # Convert the depth image to a NumPy array
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        self.depth_image = depth_image

    def extract_crack_locations(self, detection_msg):
        cracks_locations = []
        for detection in detection_msg.detections:
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)
            depth_value = self.depth_image[center_y, center_x]

            # Convert pixel coordinates to 3D point
            x_camera = (center_x - self.intrinsic_matrix[0, 2]) * depth_value / self.intrinsic_matrix[0, 0]
            y_camera = (center_y - self.intrinsic_matrix[1, 2]) * depth_value / self.intrinsic_matrix[1, 1]
            point_camera = PointStamped()
            point_camera.header.frame_id = 'red/camera'
            point_camera.header.stamp = rospy.Time(0)  # Use the latest available transform
            point_camera.point.x = x_camera
            point_camera.point.y = y_camera
            point_camera.point.z = depth_value

            # Transform the point to the world frame
            try:
                point_world = self.listener.transformPoint('world', point_camera)
                x_world = point_world.point.x
                y_world = point_world.point.y
                z_world = point_world.point.z
                if not self.scanning_finished:
                    print("Detected crack at coordinates:")
                    print("x:", x_world)
                    print("y:", y_world)
                    print("z:", z_world)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('Transform lookup failed.')

            cracks_locations.append(point_world) 


        return cracks_locations

    def update_voxel_map(self, x, y, z):
        # Convert 3D point to voxel coordinates
        x_index = np.digitize(x, self.x_bins) - 1
        y_index = np.digitize(y, self.y_bins) - 1
        z_index = np.digitize(z, self.z_bins) - 1

        if not self.scanning_finished:
            # Check if indices are within valid range
            if 0 <= x_index < len(self.x_bins) - 1 and 0 <= y_index < len(self.y_bins) - 1 and 0 <= z_index < len(self.z_bins) - 1:
                # Increment vote count for the corresponding voxel
                self.voxel_votes[x_index, y_index, z_index] += 1
            else:
                rospy.logwarn('Detected crack outside given voxel map dimensions.')


    def run(self):
        # Main loop
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = CrackVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
