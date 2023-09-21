#!/usr/bin/env python3

import rospy
import open_manipulator_msgs.srv

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import OpenManipulatorState, KinematicsPose

class Director(object):

    def __init__(self):

        # --- Joint Space Variables
        self.joint_name = ["joint1","joint2","joint3","joint4","gripper"]
        self.joint_position = [0.00,-1.05,0.35,0.70,0.01]

        self.current_joint_position = JointState()

        self.joint_positions = {
            "home": [0.00,-1.05,0.35,0.70],
            "gripper_open": [0.01],
            "gripper_closed": [-0.01],
        }


        # --- Task Space Variables

        self.task_srv_name = "gripper"
        self.task_srv_position = [0.287, 0.0, 0.239]

        self.current_task_pose = Pose()
        
        self.task_positions = {
            "home": [0.136, 0.0, 0.236],

            "high_1": [0.230, -0.115, 0.195],
            "high_2": [0.230, 0.0, 0.195],
            "high_3": [0.230, 0.115, 0.195],

            "mid_1": [0.230, -0.115, 0.150],
            "mid_2": [0.230, 0.0, 0.150],
            "mid_3": [0.230, 0.115, 0.150],

            "low_1": [0.230, -0.115, 0.050],
            "low_2": [0.230, 0.0, 0.050],
            "low_3": [0.230, 0.115, 0.050],
        }


        # --- ROS Parameters
        rospy.init_node("director_node")
        
        self.path_time = 3.0
        self.extra_time = 0.1

        self.freq = 10
        self.rate = rospy.Rate(self.freq)

        # --- Robot State Parameter
        self.robot_moving = False
        self.actuator_enable = False


        # --- ROS Topics
        self.stateSubscriber = rospy.Subscriber("/states",OpenManipulatorState, 
                                              self.manipulator_state_callback)
        self.poseSubscriber = rospy.Subscriber("/gripper/kinematics_pose",KinematicsPose, 
                                              self.kinematics_pose_callback)
        self.jointSubscriber = rospy.Subscriber("/joint_states",JointState, 
                                              self.joint_states_callback)


        # --- ROS Services
        rospy.wait_for_service("goal_joint_space_path")
        rospy.wait_for_service("goal_tool_control")
        rospy.wait_for_service("goal_task_space_path_position_only")
        
        self.goal_joint_space_path = rospy.ServiceProxy("goal_joint_space_path",
                                                        open_manipulator_msgs.srv.SetJointPosition)

        self.goal_tool_control = rospy.ServiceProxy("goal_tool_control",
                                                    open_manipulator_msgs.srv.SetJointPosition)
        
        self.goal_task_space_path_position_only = rospy.ServiceProxy("goal_task_space_path_position_only", 
                                                                     open_manipulator_msgs.srv.SetKinematicsPose)
        
        rospy.sleep(0.25)
        

    def manipulator_state_callback(self,msg):

        if msg.open_manipulator_moving_state == '"IS_MOVING"':
            self.robot_moving = True
        elif msg.open_manipulator_moving_state == '"STOPPED"':
            self.robot_moving = False

        if  msg.open_manipulator_actuator_state == '"ACTUATOR_ENABLED"':
            self.actuator_enable = True
        elif msg.open_manipulator_actuator_state == '"ACTUATOR_DISABLED"':
            self.actuator_enable = False


    def kinematics_pose_callback(self,msg):
        self.current_task_pose.position = msg.pose.position
        self.current_task_pose.orientation = msg.pose.orientation


    def joint_states_callback(self,msg):
        self.current_joint_position.name = msg.name
        self.current_joint_position.position = list(msg.position)


    def send_tool_joint_position(self,tool_state):
        
        req = open_manipulator_msgs.srv.SetJointPositionRequest()

        req.joint_position.joint_name = self.current_joint_position.name

        if tool_state:
            req.joint_position.position = self.current_joint_position.position[:-1] + [0.01]
        else:
            req.joint_position.position = self.current_joint_position.position[:-1] + [-0.01]

        req.path_time = self.path_time

        if not self.robot_moving and self.actuator_enable:

            try:
                self.goal_tool_control(req)

                rospy.loginfo("Request sent succesfully.")
                rospy.sleep(self.path_time + self.extra_time)
            
            except:
                rospy.logerr("Request failed.")

        else:
            rospy.logerr("Actuators are disabled. Request canceled.")


    def send_goal_joint_position(self, key):

        req = open_manipulator_msgs.srv.SetJointPositionRequest()
        
        # --- Only from joint_1 to joint_4
        req.joint_position.joint_name = self.joint_name[:-1]
        req.joint_position.position = self.joint_positions[key]

        req.path_time = self.path_time

        if not self.robot_moving and self.actuator_enable:

            try:
                self.goal_joint_space_path(req)

                rospy.loginfo("Request sent succesfully.")
                rospy.sleep(self.path_time + self.extra_time)
            
            except:
                rospy.logerr("Request failed.")

        else:
            rospy.logerr("Actuators are disabled. Request canceled.")


    def send_goal_task_position(self, key):

        # --- Initialize a new Pose() instance

        goal_pose = Pose()

        goal_pose.position.x = self.task_positions[key][0]
        goal_pose.position.y = self.task_positions[key][1]
        goal_pose.position.z = self.task_positions[key][2]

        goal_pose.orientation.x = 0
        goal_pose.orientation.y = 0
        goal_pose.orientation.z = 0
        goal_pose.orientation.w = 1

        # --- ROS service

        req = open_manipulator_msgs.srv.SetKinematicsPoseRequest()

        req.end_effector_name = self.task_srv_name

        req.kinematics_pose.pose = goal_pose

        req.path_time = self.path_time

        if not self.robot_moving and self.actuator_enable:

            try:
                self.goal_task_space_path_position_only(req)
                rospy.loginfo("Request sent succesfully.")
                rospy.sleep(self.path_time + self.extra_time)
            
            except:
                rospy.logerr("Request failed.")

        else:
            rospy.logerr("Actuators are disabled. Request canceled.")


def main():
    
    director = Director()

    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    main()
