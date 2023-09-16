#!/usr/bin/env python3

import rospy
import open_manipulator_msgs.srv

from geometry_msgs.msg import Pose
from open_manipulator_msgs.msg import OpenManipulatorState

class Director(object):

    def __init__(self):
        
        self.joint_name = ["joint1","joint2","joint3","joint4"]

        self.srv_name = []
        self.srv_position = []

        self.positions = {
            "home": [0.00,-1.05,0.35,0.70],
            "one": [-0.35,0.62,0.67,-1.26],
            "two": [-0.35,-0.07,0.51,-0.39],
            "three": [0.39,-0.30,0.72,-0.33],
            "gripper_open": [0.01],
            "gripper_closed": [-0.01],
        }

        self.sequence = ["home", "gripper_open", "one", "gripper_closed", "two", "three", "gripper_open"]

        self.path_time = 2.0
        self.extra_time =0.25

        rospy.init_node("director_node")

        self.freq = 10
        self.rate = rospy.Rate(self.freq)

        self.robot_moving = False
        self.actuator_enable = False

        self.last_grip = False

        # --- ROS Topics
        self.posSubscriber = rospy.Subscriber("/states",OpenManipulatorState, 
                                              self.update_manipulator_state)
        rospy.sleep(0.005)

        # --- ROS Services
        rospy.wait_for_service("goal_joint_space_path")
        rospy.wait_for_service("goal_tool_control")
        
        self.goal_joint_space_path = rospy.ServiceProxy("goal_joint_space_path",
                                                   open_manipulator_msgs.srv.SetJointPosition)

        self.goal_tool_control = rospy.ServiceProxy("goal_tool_control",
                                                   open_manipulator_msgs.srv.SetJointPosition)
        

    def update_joint_position(self,key):

        try:
            if "gripper" in key:
                self.srv_name = self.joint_name + ["gripper"]
                
                if self.last_grip:
                    self.srv_position[-1] = self.positions[key]

                else:
                    self.srv_position = self.srv_position + self.positions[key]

                self.last_grip = True

                return self.last_grip

            else:
                self.srv_name = self.joint_name
                self.srv_position = self.positions[key]
                self.last_grip = False

                return self.last_grip
        
        except KeyError:
            print("Step not defined in directory.")


    def update_manipulator_state(self,msg):

        if msg.open_manipulator_moving_state == '"IS_MOVING"':
            self.robot_moving = True
        elif msg.open_manipulator_moving_state == '"STOPPED"':
            self.robot_moving = False

        if  msg.open_manipulator_actuator_state == '"ACTUATOR_ENABLED"':
            self.actuator_enable = True
        elif msg.open_manipulator_actuator_state == '"ACTUATOR_DISABLED"':
            self.actuator_enable = False


    def send_joint_position(self):

        req = open_manipulator_msgs.srv.SetJointPositionRequest()
        
        req.joint_position.joint_name = self.srv_name
        req.joint_position.position = self.srv_position

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


    def send_tool_position(self):

        req = open_manipulator_msgs.srv.SetJointPositionRequest()
        
        req.joint_position.joint_name = self.srv_name
        req.joint_position.position = self.srv_position

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
        
        
def main():

    director = Director()

    while not rospy.is_shutdown():

        if director.actuator_enable:
            for key in director.sequence:

                uses_tool = director.update_joint_position(key)

                if(uses_tool):
                    director.send_tool_position()
                else:
                    director.send_joint_position()


if __name__ == '__main__':
    main()