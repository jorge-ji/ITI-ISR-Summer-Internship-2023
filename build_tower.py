#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import zmq
from kortex_driver.srv import *
from kortex_driver.msg import *

context = zmq.Context()

#  Socket to talk to server
print("Connecting to OpenPose server...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

block_1_start = ConstrainedPose()
block_1_target = ConstrainedPose()
block_2_start = ConstrainedPose()
block_2_target = ConstrainedPose()
block_3_start = ConstrainedPose()
block_3_target = ConstrainedPose()

class BuildTower:
    def __init__(self):
        try:
            rospy.init_node('build_tower_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
            
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def robot_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                time.sleep(0.6)
                return 1

    def robot_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True
        

    def send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True
        
    def parse_information(self, argv_n):
        f = open(sys.argv[argv_n], 'r')

        # y = robot; p = left; b = blue
        turn_order = f.readline()

        # 1 = square; 2 = rectangle; 3 = semicircle; 4 = bridge
        object_order = f.readline()

        if f.readline() == "B1 start\n":
            block_1_start.target_pose.x = float(f.readline())
            block_1_start.target_pose.y = float(f.readline())
            block_1_start.target_pose.z = float(f.readline())
            block_1_start.target_pose.theta_x = float(f.readline())
            block_1_start.target_pose.theta_y = float(f.readline())
            block_1_start.target_pose.theta_z = float(f.readline())
        if f.readline() == "B1 target\n":
            block_1_target.target_pose.x = float(f.readline())
            block_1_target.target_pose.y = float(f.readline())
            block_1_target.target_pose.z = float(f.readline())
            block_1_target.target_pose.theta_x = float(f.readline())
            block_1_target.target_pose.theta_y = float(f.readline())
            block_1_target.target_pose.theta_z = float(f.readline())
        if f.readline() == "B2 start\n":
            block_2_start.target_pose.x = float(f.readline())
            block_2_start.target_pose.y = float(f.readline())
            block_2_start.target_pose.z = float(f.readline())
            block_2_start.target_pose.theta_x = float(f.readline())
            block_2_start.target_pose.theta_y = float(f.readline())
            block_2_start.target_pose.theta_z = float(f.readline())
        if f.readline() == "B2 target\n":
            block_2_target.target_pose.x = float(f.readline())
            block_2_target.target_pose.y = float(f.readline())
            block_2_target.target_pose.z = float(f.readline())
            block_2_target.target_pose.theta_x = float(f.readline())
            block_2_target.target_pose.theta_y = float(f.readline())
            block_2_target.target_pose.theta_z = float(f.readline())
        if f.readline() == "B3 start\n":
            block_3_start.target_pose.x = float(f.readline())
            block_3_start.target_pose.y = float(f.readline())
            block_3_start.target_pose.z = float(f.readline())
            block_3_start.target_pose.theta_x = float(f.readline())
            block_3_start.target_pose.theta_y = float(f.readline())
            block_3_start.target_pose.theta_z = float(f.readline())
        if f.readline() == "B3 target\n":
            block_3_target.target_pose.x = float(f.readline())
            block_3_target.target_pose.y = float(f.readline())
            block_3_target.target_pose.z = float(f.readline())
            block_3_target.target_pose.theta_x = float(f.readline())
            block_3_target.target_pose.theta_y = float(f.readline())
            block_3_target.target_pose.theta_z = float(f.readline())

        return (turn_order, object_order)
    
    
    def set_speed(self, cons_pose, translation, orientation):
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = translation # m/s
        my_cartesian_speed.orientation = orientation # deg/s
        cons_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

    def go_to_position(self, pos):

        self.set_speed(pos, 1, 100)

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(pos)
        req.input.name = "pose"
        req.input.handle.action_type = ActionType.REACH_POSE

        rospy.loginfo("Sending pose...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose")
            return False
        else:
            rospy.loginfo("Waiting for pose to finish...")

        self.wait_for_action_end_or_abort()

        return True

    def control_gripper(self, value): # 0 is open, 1 is fully closed
        if self.is_gripper_present:
            self.send_gripper_command(value)
            rospy.loginfo("Moving gripper...")
            return True
        else:
            rospy.logwarn("No gripper is present on the arm.")
            return False
    

    def move_block(self, pos1, pos2, turn_n, block_shape):

        self.control_gripper(0)

        pos1.target_pose.z += 0.045
        self.go_to_position(pos1)

        pos1.target_pose.z -= 0.045
        self.go_to_position(pos1)

        if block_shape == 1:
            self.control_gripper(0.08)
        elif block_shape == 2:
            self.control_gripper(0.7)
        elif block_shape == 3:
            self.control_gripper(0.2)
        elif block_shape == 4:
            self.control_gripper(0.7)

        pos1.target_pose.z += 0.045 + turn_n*0.045
        self.go_to_position(pos1)

        pos2.target_pose.z += 0.045
        self.go_to_position(pos2)

        pos2.target_pose.z -= 0.045
        self.go_to_position(pos2)

        self.control_gripper(0)

        pos2.target_pose.z += 0.045
        self.go_to_position(pos2)

        return True


    def look_at_tower(self):
        tower_pos = ConstrainedPose()
        tower_pos.target_pose.x = 0.50
        tower_pos.target_pose.y = 0
        tower_pos.target_pose.z = 0.30
        tower_pos.target_pose.theta_x = 110
        tower_pos.target_pose.theta_y = 0
        tower_pos.target_pose.theta_z = 90
        # Look at tower
        self.go_to_position(tower_pos)


    def look_at_person_1(self):
        person_1_pos = ConstrainedPose()
        person_1_pos.target_pose.x = 0.5
        person_1_pos.target_pose.y = 0.15
        person_1_pos.target_pose.z = 0.50
        person_1_pos.target_pose.theta_x = 80
        person_1_pos.target_pose.theta_y = 0
        person_1_pos.target_pose.theta_z = 105
        # Look at person 1
        self.go_to_position(person_1_pos)
    

    def look_at_person_2(self):
        person_2_pos = ConstrainedPose()
        person_2_pos.target_pose.x = 0.5
        person_2_pos.target_pose.y = -0.15
        person_2_pos.target_pose.z = 0.50
        person_2_pos.target_pose.theta_x = 80
        person_2_pos.target_pose.theta_y = 0
        person_2_pos.target_pose.theta_z = 75
        # Look at person 2
        self.go_to_position(person_2_pos)

    def look_at_ground(self):
        ground_pose = ConstrainedPose()
        ground_pose.target_pose.x = 0.3
        ground_pose.target_pose.y = 0
        ground_pose.target_pose.z = 0.1
        ground_pose.target_pose.theta_x = 180
        ground_pose.target_pose.theta_y = 0
        ground_pose.target_pose.theta_z = 90
        # Look at ground
        self.go_to_position(ground_pose)

    def send_turn_order(self, turn_order):
        print("Sending turn order...")
        socket.send_string(turn_order)
            
        #  Get the reply.
        message = socket.recv()
        print(f"Received reply [ {message} ]")

    def turn(self, turn_order, object_order, turn_n, robot_turn_n, config_n, condition_n):

        if turn_order[turn_n] == 'y':
            self.home_the_robot()
            block_shape = int(object_order[robot_turn_n])
            if robot_turn_n == 0:
                self.move_block(block_1_start, block_1_target, turn_n, block_shape)
            elif robot_turn_n == 1:
                self.move_block(block_2_start, block_2_target, turn_n, block_shape)
            elif robot_turn_n == 2:
                self.move_block(block_3_start, block_3_target, turn_n, block_shape)
            self.home_the_robot()
            return 1

        elif turn_order[turn_n] == 'p':

            if (condition_n == 1 and (config_n == 1 or config_n == 2)) or (condition_n == 2 and (config_n == 3 or config_n == 4)):
                self.look_at_person_1()
            else:
                self.look_at_ground()

            # Wait until person 1 starts doing his action
            print("Sending P1 Arm Extended request...")
            socket.send_string("P1: Arm Extended Check")
                
            #  Get the reply.
            message = socket.recv()
            print(f"Received reply [ {message} ]")

            if (condition_n == 1 and (config_n == 1 or config_n == 2)) or (condition_n == 2 and (config_n == 3 or config_n == 4)):
                self.home_the_robot()

                self.look_at_tower()

            # Wait until person 1 finishes his action
            print("Sending P1 Arm Retracted request...")
            socket.send_string("P1: Arm Retracted Check")
            
            #  Get the reply.
            message = socket.recv()
            print(f"Received reply [ {message} ]")
            if (condition_n == 1 and (config_n == 1 or config_n == 2)) or (condition_n == 2 and (config_n == 3 or config_n == 4)):
                self.home_the_robot()

            return 0

        elif turn_order[turn_n] == 'b':

            if (condition_n == 1 and (config_n == 1 or config_n == 2)) or (condition_n == 2 and (config_n == 3 or config_n == 4)):
                self.look_at_person_2()
            else:
                self.look_at_ground()

            # Wait until person 2 starts doing his action
            print("Sending P2 Arm Extended request...")
            socket.send_string("P2: Arm Extended Check")
                
            #  Get the reply.
            message = socket.recv()
            print(f"Received reply [ {message} ]")

            if (condition_n == 1 and (config_n == 1 or config_n == 2)) or (condition_n == 2 and (config_n == 3 or config_n == 4)):
                self.home_the_robot()

                self.look_at_tower()

            # Wait until person 2 finishes his action
            print("Sending P2 Arm Retracted request...")
            socket.send_string("P2: Arm Retracted Check")
                
            #  Get the reply.
            message = socket.recv()
            print(f"Received reply [ {message} ]")
            if (condition_n == 1 and (config_n == 1 or config_n == 2)) or (condition_n == 2 and (config_n == 3 or config_n == 4)):
                self.home_the_robot()

            return 0

    
    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:

            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.robot_clear_faults()
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.robot_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.subscribe_to_a_robot_notification()

            #*******************************************************************************

            self.home_the_robot()

            condition_n = int(sys.argv[1])

            argc = len(sys.argv)
            config_n = 1

            while config_n <= argc:

                input("Press Enter to start configuration " + str(config_n))

                # Get all the objects starting and target positions
                info = self.parse_information(config_n + 1)
                turn_order = info[0]
                object_order = info[1]
                turn_n = 0
                robot_turn_n = 0

                self.send_turn_order(turn_order)

                while turn_n < len(turn_order)-1:
                    if self.turn(turn_order, object_order, turn_n, robot_turn_n, config_n, condition_n) == 1:
                        robot_turn_n += 1
                    turn_n += 1

                print("Configuration number " + str(config_n) +" finished")
                socket.send_string("Configuration finished")
                message = socket.recv()

                config_n += 1
            
            # Movement finished, send to home position
            success &= self.home_the_robot()
            

            success &= self.all_notifs_succeeded

            success &= self.all_notifs_succeeded

        
        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = BuildTower()
    ex.main()
