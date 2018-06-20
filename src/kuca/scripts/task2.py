#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface 
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from copy import deepcopy

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
 
        # Construct the initial scene object
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        
        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('arm')
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        right_arm.set_goal_joint_tolerance(0.001)
        right_arm.set_goal_position_tolerance(0.0001)
       
        scene.remove_world_object('block')

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
        
        
        # Set the size of block
        block_size = [0.01, 0.05, 0.05]
        
        # Create a pose for the block relative to the end-effector
        p = PoseStamped()
        p.header.frame_id = end_effector_link
        
        
        # Place the end of the object within the grasp of the gripper
        p.pose.position.x = -0.015
        p.pose.position.y = 0.0
        p.pose.position.z = 0.0
        
        # Align the object with the gripper (straight out)
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        
        # Attach the tool to the end-effector
        scene.attach_box(end_effector_link, 'block', p, block_size)

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


        joint_positions = right_arm.get_current_joint_values()
        joint_positions[0] = -1.5707
        right_arm.set_joint_value_target(joint_positions)
        right_arm.go()
        rospy.sleep(1)
        
        
        waypoints = []
        start_pose = right_arm.get_current_pose().pose
        wpose = deepcopy(start_pose)
        waypoints.append(deepcopy(wpose))

       
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

        scene.remove_attached_object(end_effector_link, 'block')
        rospy.sleep(1)

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
        
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
