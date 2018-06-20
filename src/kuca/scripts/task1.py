#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1 2014-01-14
    
    Use forward kinemtatics to move the arm to a specified set of joint angles
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from copy import deepcopy

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        GRIPPER_OPEN = [0.05]
        GRIPPER_CLOSED = [0.00]
        GRIPPER_NEUTRAL = [0.01]
 
        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('arm')
        
        # Connect to the right_gripper move group
        right_gripper = moveit_commander.MoveGroupCommander('gripper')
                
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        right_arm.set_goal_joint_tolerance(0.001)
        right_arm.set_goal_position_tolerance(0.0001)
        right_gripper.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "contract" pose stored in the SRDF file
    #    right_arm.set_named_target('contract')
    #    right_arm.go()
    #    rospy.sleep(2)
        
        # Start the arm target in "right" pose stored in the SRDF file
        right_arm.set_named_target('right')
        
        # Plan a trajectory to the goal configuration
        traj = right_arm.plan()
         
        # Execute the planned trajectory
        right_arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)
        start_pose = right_arm.get_current_pose().pose

        # Set the gripper target to neutal position using a joint value target
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        
        # Plan and execute the gripper motion
        right_gripper.go()
        rospy.sleep(2)
        

        # Open the gripper as if letting something go
        right_gripper.set_joint_value_target(GRIPPER_OPEN)
        right_gripper.go()
        rospy.sleep(2)
       
        waypoints = []
        wpose = deepcopy(start_pose)
        waypoints.append(deepcopy(wpose))

        wpose.position.x -= 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.3
        waypoints.append(deepcopy(wpose))

        wpose.position.z -= 0.5
        waypoints.append(deepcopy(wpose))

        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = right_arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    0.01,        # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions
            
            # Increment the number of attempts 
            attempts += 1
            
            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
    
            right_arm.execute(plan)
                        
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
        
        # Close the gripper as if picking something up
        right_gripper.set_joint_value_target(GRIPPER_CLOSED)
        right_gripper.go()
        rospy.sleep(2)


        waypoints = []
        start_pose = right_arm.get_current_pose().pose
        wpose = deepcopy(start_pose)
        waypoints.append(deepcopy(wpose))

        wpose.position.z += 0.25
        waypoints.append(deepcopy(wpose))

        #wpose.position.y -= 0.30
        #waypoints.append(deepcopy(wpose))
       

        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = right_arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    0.01,        # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions
            
            # Increment the number of attempts 
            attempts += 1
            
            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
    
            right_arm.execute(plan)
                        
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  


        # Open the gripper as if letting something go
        right_gripper.set_joint_value_target(GRIPPER_OPEN)
        right_gripper.go()
        rospy.sleep(2)
         
        
        joint_positions = right_arm.get_current_joint_values()
        joint_positions[0] = - 2*(joint_positions[0])
        right_arm.set_joint_value_target(joint_positions)
        right_arm.go()
        rospy.sleep(1)
        
        
        waypoints = []
        start_pose = right_arm.get_current_pose().pose
        wpose = deepcopy(start_pose)
        waypoints.append(deepcopy(wpose))

        #wpose.position.y -= 0.3
        #waypoints.append(deepcopy(wpose))
       
        wpose.position.z -= 0.25
        waypoints.append(deepcopy(wpose))

        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = right_arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    0.01,        # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions
            
            # Increment the number of attempts 
            attempts += 1
            
            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
    
            right_arm.execute(plan)
                        
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(2)

        waypoints = []
        start_pose = right_arm.get_current_pose().pose
        wpose = deepcopy(start_pose)
        waypoints.append(deepcopy(wpose))

        wpose.position.z += 0.15
        waypoints.append(deepcopy(wpose))

        #wpose.position.y -= 0.30
        #waypoints.append(deepcopy(wpose))
       

        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = right_arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    0.01,        # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions
            
            # Increment the number of attempts 
            attempts += 1
            
            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
    
            right_arm.execute(plan)
                        
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
        
        rospy.sleep(1)
        
        joint_positions = right_arm.get_current_joint_values()
        joint_positions[0] = 0
        joint_positions[4] = 0
        joint_positions[5] = 0
        right_arm.set_joint_value_target(joint_positions)
        right_arm.go()
        rospy.sleep(1)

        # Return the arm to the named "resting" pose stored in the SRDF file
        right_arm.set_named_target('right')
        right_arm.go()
        rospy.sleep(1)
        
        # Return the gripper target to neutral position
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
        rospy.sleep(2)
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
