#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface 
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from copy import deepcopy

class MoveItDemo:
    def move_arm(self, right_arm, num):

        waypoints = []
        start_pose = right_arm.get_current_pose().pose

        wpose = deepcopy(start_pose)
        waypoints.append(deepcopy(wpose))
        
        if num == 1:
            wpose.position.x -= 0.15
            waypoints.append(deepcopy(wpose))

            wpose.position.y += 0.5
            waypoints.append(deepcopy(wpose))

            wpose.position.z -= 0.5
            waypoints.append(deepcopy(wpose))
        
        elif num == 2:   
            wpose.position.x -= 0.1
            waypoints.append(deepcopy(wpose))
            wpose.position.z += 0.25
            waypoints.append(deepcopy(wpose))
            wpose.position.y -= 0.5
            waypoints.append(deepcopy(wpose))
            wpose.position.x += 0.1
            waypoints.append(deepcopy(wpose))

        elif num == 3:
            wpose.position.z -= 0.25
            waypoints.append(deepcopy(wpose))

        else:
            wpose.position.z += 0.15
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
       

        count = 1
        while count <= 2:
            self.move_arm(right_arm, 1)
            rospy.sleep(2) 
            
            self.move_arm(right_arm, 2)
            rospy.sleep(1)

            
            # Return the arm to the named "resting" pose stored in the SRDF file
            right_arm.set_named_target('right')
            
            plan_fast = right_arm.plan()
            print type(plan_fast)
            if count == 1:
                 right_arm.execute(plan_fast)
            else:
                plan_slow = deepcopy(plan_fast)
                for position in plan_slow.joint_trajectory.points.positions:
                    plan_slow.joint_trajectory.points.positions[position] /= 2.0
                for velocity in plan_slow.joint_trajectory.points.velocities:
                    plan_slow.joint_trajectory.points.velocities[velocity] /= 2.0
                for acc in plan_slow.joint_trajectory.points.accelerations:
                    plan_slow.joint_trajectory.points.positions[acc] /= 4.0
                for time in plan_slow.joint_trajectory.points.time_from_start:
                    plan_slow.joint_trajectory.points.positions[time] /= 2.0

                
                
                right_arm.execute(plan_slow)
            rospy.sleep(1)
            count += 1 
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
