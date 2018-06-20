#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface 
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_msgs.msg import PlanningScene, ObjectColor
from copy import deepcopy
from random import random

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        rospy.on_shutdown(self.shutdown)
 
        # Construct the initial scene object
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = {}

        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('arm')
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))

        right_arm.allow_replanning(True)
        
        # Set the solve method.
        #right_arm.set_planner_id('RRTConnectkConfigDefault')
    

        # Set a small tolerance on joint angles
        right_arm.set_goal_joint_tolerance(0.0001)
        right_arm.set_goal_position_tolerance(0.00001)
        right_arm.set_goal_orientation_tolerance(0.0001)
     
        # Set the position of the mesh file.
        filename = '/home/jyk/catkin_ws/src/kuca/meshes/block.STL'
        
        # Set the planning time.
        right_arm.set_planning_time(5)

        row = 5
        col = 5 
        
        total = row * col

        for i in range(total):
            #scene.remove_attached_object('block' + str(i))
            scene.remove_world_object('block' + str(i))
        
        rospy.sleep(1)    
        # Start the arm target in "right" pose stored in the SRDF file
        right_arm.set_named_target('right')
        
        # Plan a trajectory to the goal configuration
        traj = right_arm.plan()
         
        # Execute the planned trajectory
        right_arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)
        
        ###########
        print right_arm.get_current_joint_values()
        print '#'*10
        ###########

        # Make a block in fixed pose
        q = quaternion_from_euler(0, 0, 1.5707)
        target_pose = right_arm.get_current_pose()
        target_pose.pose.position.x -= 0.8
        target_pose.pose.position.y += 0.9
        target_pose.pose.position.z -= 0.5
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]


        object_pose = deepcopy(target_pose)
        
        #q = quaternion_from_euler(0, 0, 1.5707)
        object_pose.pose.orientation.x = 0
        object_pose.pose.orientation.y = 0
        object_pose.pose.orientation.z = 0
        object_pose.pose.orientation.w = 1

        refer_place_pose = PoseStamped()
        refer_place_pose.header.frame_id = 'base_footprint'
        refer_place_pose.pose.position.x = 0.8
        refer_place_pose.pose.position.y = 0.1
        refer_place_pose.pose.position.z = 0.6
        refer_place_pose.pose.orientation.x = 0
        refer_place_pose.pose.orientation.y = 0
        refer_place_pose.pose.orientation.z = 0
        refer_place_pose.pose.orientation.w = 1

        count = 0

        while (count < total):
            # Set the size of block
            block_size = [0.01, 0.05, 0.05]
            
            place_pose = deepcopy(refer_place_pose)
            
            #if (count % 5 == 0):
            #    place_pose.pose.position.z += 0.51*(count)

            place_pose.pose.position.y += 0.076*(count % col)
            place_pose.pose.position.z -= 0.043315*(count % col)
            place_pose.pose.position.z += 0.087*(count // col)

            block_id = 'block' + str(count)

            scene.add_mesh(block_id, object_pose, filename)
            rospy.sleep(1)


            #right_arm.set_joint_value_target(target_pose, end_effector_link, True)
            target_pose.header.stamp = rospy.Time()
            right_arm.set_pose_target(target_pose)
            right_arm.go()
            rospy.sleep(0.5) 
            
            # Create a pose for the block relative to the end-effector
            p = PoseStamped()
            p.header.frame_id = end_effector_link
            
            
            # Place the end of the object within the grasp of the gripper
            p.pose.position.x = 0.01
            p.pose.position.y = 0.0
            p.pose.position.z = 0.0
            
            # Align the object with the gripper (straight out)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            
            # Attach the tool to the end-effector
            scene.attach_mesh(end_effector_link, block_id, p, filename)
            rospy.sleep(1)
        
            #right_arm.set_start_state_to_current_state()
            place_pose.header.stamp = rospy.Time()
            right_arm.set_pose_target(place_pose)
            right_arm.go()
            rospy.sleep(0.5)
            
            right_arm.detach_object(end_effector_link)
            #scene.remove_attached_object(end_effector_link, 'block'+str(count))
            #scene.remove_world_object('block'+str(count))
            rospy.sleep(1)
           
            # Set the color to the block.
            rgb = self.makeColor()
            self.setColor(block_id, rgb)

            count += 1

        # Return the arm to the named "resting" pose stored in the SRDF file
        right_arm.set_named_target('right')
        right_arm.go()
        rospy.sleep(1)
        
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

    def makeColor(self):
        rgb = []
        rgb.append(round(random(), 2))
        rgb.append(round(random(), 2))
        rgb.append(round(random(), 2))
        return rgb

    # Set the color of an object
    def setColor(self, name, rgb, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = rgb[0]
        color.color.g = rgb[1]
        color.color.b = rgb[2]
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color
        
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = False
        
        # Append the colors from the global color dictionary 
        #for color in self.colors.values():
        #    p.object_colors.append(color)
        p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

    def shutdown(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)
