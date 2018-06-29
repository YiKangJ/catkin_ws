#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  arm = moveit_commander.MoveGroupCommander("arm")

  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % arm.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  end_effector_link = arm.get_end_effector_link()
  print "============ End effector: %s" % end_effector_link
    
  arm.set_goal_tolerance(0.00001);
  #arm.set_planner_id("BKPIECEkConfigDefault") 2
  #arm.set_planner_id("BFMTkConfigDefault") 2
  #arm.set_planner_id("BiESTkConfigDefault")
  #arm.set_planner_id("BiTRRTkConfigDefault")
  #arm.set_planner_id("BKPIECEkConfigDefault")
  #arm.set_planner_id("BKPIECEkConfigDefault")
    
  arm.set_named_target("right")
  arm.go()
  rospy.sleep(1)

  pose_target = geometry_msgs.msg.Pose()
  '''  
  pose_target.position.x = 0.4
  pose_target.position.y = -0.3
  pose_target.position.z = 0.4

  pose_target.orientation.x = 0.670820393249937
  pose_target.orientation.y = -0.223606797749979
  pose_target.orientation.z = -0.223606797749979
  pose_target.orientation.w = 0.670820393249937
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  rospy.sleep(3)
  print arm.get_current_pose()
  print '-'*30
  
  pose_target.position.x = 0.4
  pose_target.position.y = -0.2
  pose_target.position.z = 0.4

  pose_target.orientation.x = 0.688190960235602
  pose_target.orientation.y = -0.162459848116465
  pose_target.orientation.z = -0.162459848116465
  pose_target.orientation.w = 0.688190960235602
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  #rospy.sleep(1)
  print arm.get_current_pose()
  print '-'*30
  '''
  pose_target.position.x = 0.312
  pose_target.position.y = 0.247
  pose_target.position.z = 0.364
  pose_target.orientation.x = 0.667840891317311
  pose_target.orientation.y = 0.232354349829369
  pose_target.orientation.z = 0.232354349830866
  pose_target.orientation.w = 0.667840891317311
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  '''
  pose_target.position.x = 0.4
  pose_target.position.y = -0.1
  pose_target.position.z = 0.4

  pose_target.orientation.x = 0.701808823710204
  pose_target.orientation.y = -0.0863966142936918
  pose_target.orientation.z = -0.0863966142936918
  pose_target.orientation.w = 0.701808823710204
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  #rospy.sleep(1)
  print arm.get_current_pose()
  print '-'*30
  
  
  pose_target.position.x = 0.4
  pose_target.position.y = 0
  pose_target.position.z = 0.4

  pose_target.orientation.x = 0.707106781186547
  pose_target.orientation.y = 0
  pose_target.orientation.z = 0
  pose_target.orientation.w = 0.707106781186548
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  #rospy.sleep(1)
  print arm.get_current_pose()
  print '-'*30
  

  pose_target.position.x = 0.4
  pose_target.position.y = 0.1
  pose_target.position.z = 0.4

  pose_target.orientation.x = 0.701808823710204
  pose_target.orientation.y = 0.0863966142936918
  pose_target.orientation.z = 0.0863966142936918
  pose_target.orientation.w = 0.701808823710204
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  #rospy.sleep(1)
  print arm.get_current_pose()
  print '-'*30
  
  pose_target.position.x = 0.4
  pose_target.position.y = 0.2
  pose_target.position.z = 0.4

  pose_target.orientation.x = 0.688190960235602
  pose_target.orientation.y = 0.162459848116465
  pose_target.orientation.z = 0.162459848116465
  pose_target.orientation.w = 0.688190960235602
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  #rospy.sleep(1)
  print arm.get_current_pose()
  print '-'*30
  
  
  pose_target.position.x = 0.4
  pose_target.position.y = 0.3
  pose_target.position.z = 0.4

  pose_target.orientation.x = 0.670820393249937
  pose_target.orientation.y = 0.223606797749979
  pose_target.orientation.z = 0.223606797749979
  pose_target.orientation.w = 0.670820393249937
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)

  arm.go()
  #rospy.sleep(1)
  print arm.get_current_pose()
  print '-'*30
  '''
  arm.set_named_target("up")
  arm.go()
  rospy.sleep(1)
  
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
