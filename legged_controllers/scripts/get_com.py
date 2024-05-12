#!/usr/bin/env python3
import rospy
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import LinkStates # Make sure this import is correct
# from yr_description.msg import ComInertiaMass


class CoMAndInertiaCalculator:
    def __init__(self):
        rospy.init_node('com_and_inertia_calculator')

        self.robot_description = rospy.get_param('/legged_robot_description_sim')
        # print(self.robot_description)
        self.robot = URDF.from_xml_string(self.robot_description)
        _, self.tree = treeFromUrdfModel(self.robot)

        self.link_poses = {}  # Stores the current poses of all links
        self.link_names = [link.name for link in self.robot.links]

        self.listener = rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback)
        # self.com_inertia_mass_pub = rospy.Publisher('/exo/com_inertia_mass', ComInertiaMass, queue_size=10)
        self.height_from_ground = 0

    def link_states_callback(self, msg):
        for i, full_name in enumerate(msg.name):
            # This splits the name on '::' and uses the last part, which should match the URDF link names.
            link_name = full_name.split('::')[-1]
            if link_name in self.link_names:
                self.link_poses[link_name] = msg.pose[i]

        self.calculate_com_and_inertia()

    def calculate_com_and_inertia(self):
        total_mass = 0.0
        weighted_pos_sum = PyKDL.Vector(0, 0, 0)
        total_inertia = np.zeros((3, 3))
        max_height = 0
        for link_name, pose in self.link_poses.items():
            
            link = self.robot.link_map[link_name]
            if link.inertial is not None:
                pos = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
                ori = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                frame = PyKDL.Frame(ori, pos)
                com_pos = frame * PyKDL.Vector(*link.inertial.origin.xyz)
                # base_pose = []
                # foot_pose = []
                if link_name == "base_link":
                    base_pose = com_pos
                    print("base link pose",com_pos)
                
                if link_name == "leg_r5_link":
                    foot_pose = com_pos
                    print("foot link pose",com_pos)
                # print("difference", base_pose-foot_pose)
                
                weighted_pos_sum += link.inertial.mass * com_pos
                total_mass += link.inertial.mass
                max_height = max(-1*com_pos[2], max_height)
                self.height_from_ground = max(max_height, self.height_from_ground)

                # Inertia tensor calculation here as previously

        if total_mass > 0:
            com = weighted_pos_sum * (1.0 / total_mass)
            print("Com Position ",com)
            # com_inertia_mass_msg = ComInertiaMass()
            # com_inertia_mass_msg.com = Point(x=com[0], y=com[1], z=com[2] + self.height_from_ground)
            # com_inertia_mass_msg.inertia = list(total_inertia.flatten())
            # com_inertia_mass_msg.mass = total_mass
            # self.com_inertia_mass_pub.publish(com_inertia_mass_msg)
        else:
            rospy.logwarn("Total mass is zero, can't compute CoM or Inertia.")

if __name__ == '__main__':
    calculator = CoMAndInertiaCalculator()
    rospy.spin()
