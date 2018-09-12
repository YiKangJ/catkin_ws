#!/usr/bin/env python

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

class MoveItDemo:
    def __init__(self):
         
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('arm')
        
                
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        
        # Set a small tolerance on joint angles
        right_arm.set_goal_tolerance(0.0001)
        
        right_arm.set_named_target('right')
        right_arm.go()
        rospy.sleep(1)
        
        '''
        start_pose = right_arm.get_current_pose().pose
        start_pose.position.x = 0.4
        start_pose.position.y = 0
        #start_pose.position.z = 0.4
        start_pose.orientation.x = 0.707106781186547
        start_pose.orientation.y = 0
        start_pose.orientation.z = 0
        start_pose.orientation.w = 0.707106781186547
         
        right_arm.set_pose_target(start_pose)

        right_arm.go()
        
        # Pause for a moment
        rospy.sleep(1)
        '''
        plan = RobotTrajectory()

        joint_trajectory = JointTrajectory()
        joint_trajectory.header.frame_id = 'base_link'
        #joint_trajectory.header.stamp = rospy.Time.now()
        joint_trajectory.joint_names = ['base_to_armA', 'armA_to_armB', 'armB_to_armC', 'armC_to_armD']

        # Output
        with open ("/home/jyk/Desktop/pvt.txt",'r') as f:
            for line in f.readlines():
                cont = line.rstrip('\r\n').replace(' ', '').split(',') 
                point = JointTrajectoryPoint()
                point.positions = map(float, cont[0:4])
                point.velocities = map(float, cont[4:8])
                point.time_from_start = rospy.Duration(float(cont[8])*6)
                joint_trajectory.points.append(deepcopy(point))

        
        plan.joint_trajectory = joint_trajectory

        #print type(plan)
        #print plan
        right_arm.set_start_state_to_current_state()
        right_arm.execute(plan)

        #rospy.sleep(3)

        '''
        # Return the arm to the named "resting" pose stored in the SRDF file
        right_arm.set_named_target('right')
        right_arm.go()
        rospy.sleep(1)
        '''

        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)
         

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
