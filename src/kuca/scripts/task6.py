#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface 
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
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

        right_arm.allow_replanning(True)

        # Set a small tolerance on joint angles
        right_arm.set_goal_joint_tolerance(0.001)
        right_arm.set_goal_position_tolerance(0.0001)
       
        for i in range(5):
            scene.remove_attached_object('block' + str(i+1))
            scene.remove_world_object('block' + str(i+1))
            rospy.sleep(0.5)
            
        # Start the arm target in "right" pose stored in the SRDF file
        right_arm.set_named_target('right')
        
        # Plan a trajectory to the goal configuration
        traj = right_arm.plan()
         
        # Execute the planned trajectory
        right_arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)
       
        target_pose = right_arm.get_current_pose()
        target_pose.pose.position.x -= 0.2
        target_pose.pose.position.y += 0.4 
        target_pose.pose.position.z -= 0.5 
        target_pose.pose.orientation.w = 1

        object_pose = target_pose
        #object_pose.pose.position.x -= 0.08 
       
        q = quaternion_from_euler(0, 0, -1.5707)
        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_footprint'
        place_pose.pose.position.x = 0.3
        place_pose.pose.position.y = -0.3
        place_pose.pose.position.z = 0.25
        place_pose.pose.orientation.x = q[0]
        place_pose.pose.orientation.y = q[1]
        place_pose.pose.orientation.z = q[2]
        place_pose.pose.orientation.w = q[3]

        count = 0
        while count < 1:
            # Set the size of block
            block_size = [0.01, 0.05, 0.05]
            
            count += 1
           # scene.add_box('block'+str(count), object_pose, block_size)
           # rospy.sleep(1)

            #right_arm.set_joint_value_target(target_pose, end_effector_link, True)
            right_arm.set_pose_target(target_pose)
            right_arm.go()
            rospy.sleep(2) 
            
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
           # scene.attach_box(end_effector_link, 'block'+str(count), p, block_size)
            rospy.sleep(1)
        
            right_arm.set_pose_target(place_pose)
            right_arm.go()
            rospy.sleep(5)

            scene.remove_attached_object(end_effector_link, 'block'+str(count))
            rospy.sleep(2)
            

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
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)
