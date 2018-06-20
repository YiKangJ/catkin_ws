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


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)


  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % arm.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  end_effector_link = arm.get_end_effector_link()
  print "============ End effector: %s" % end_effector_link
    
  arm.set_goal_tolerance(0.0001);
  #arm.set_planner_id("BKPIECEkConfigDefault") 2
  #arm.set_planner_id("BFMTkConfigDefault") 2
  #arm.set_planner_id("BiESTkConfigDefault")
  #arm.set_planner_id("BiTRRTkConfigDefault")
  #arm.set_planner_id("BKPIECEkConfigDefault")
  #arm.set_planner_id("BKPIECEkConfigDefault")
    
  arm.set_named_target("right")
  arm.go()
  rospy.sleep(1)

  joint_positions = [0, 0.909948481767, -1.59570870752, 0.68576022575]
  pose_target = geometry_msgs.msg.Pose()
  '''
  pose_target.position.x = 0.132938660994
  pose_target.position.y = -0.0955394659395
  pose_target.position.z = 0.818050660634

  pose_target.orientation.x = 0.568943188752
  pose_target.orientation.y = -0.419889964574
  pose_target.orientation.z =  0.00894212272965
  pose_target.orientation.w = 0.707047455312
  '''
  pose_target.position.x = 0.5
  pose_target.position.y = 0.3
  pose_target.position.z = 0.3

  pose_target.orientation.x = 0.707106781186547
  pose_target.orientation.y = 0.0
  pose_target.orientation.z = 0.0
  pose_target.orientation.w = 0.707106781186547
  
  arm.set_start_state_to_current_state()
  #arm.set_pose_target(pose_target)
  arm.set_joint_value_target(joint_positions)

  arm.go()
  rospy.sleep(3)
  '''  
  pose_target.position.x = 0.0151494
  pose_target.position.y = -0.00695572
  pose_target.position.z = 0.715798

  pose_target.orientation.x = 0
  pose_target.orientation.y = 0
  pose_target.orientation.z = 0
  pose_target.orientation.w = 1
  
  pose_target.position.x = 0.00663386513978
  pose_target.position.y = 0.00628765005211
  pose_target.position.z = 0.86011464552

  pose_target.orientation.x = -0.470078484577
  pose_target.orientation.y = -0.528233012237
  pose_target.orientation.z = -0.0603648311788
  pose_target.orientation.w = 0.704522668391
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)
  #arm.set_joint_value_target(joint_positions)

  arm.go()
  rospy.sleep(3)

  pose_target.position.x = 0.209690939587
  pose_target.position.y = -0.211907873744
  pose_target.position.z = 0.235437784057

  pose_target.orientation.x = 0.575017427409
  pose_target.orientation.y = -0.411526317144
  pose_target.orientation.z = -0.119272325492
  pose_target.orientation.w = 0.696975724718
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)
  #arm.set_joint_value_target(joint_positions)

  arm.go()
  rospy.sleep(3)

  pose_target.position.x = -0.504417518865
  pose_target.position.y = -0.0699290237893
  pose_target.position.z = 0.474962674352

  pose_target.orientation.x = 0.0019269065365
  pose_target.orientation.y = 0.707103498593
  pose_target.orientation.z = -0.700141412041
  pose_target.orientation.w = 0.099009759386
  
  arm.set_start_state_to_current_state()
  arm.set_pose_target(pose_target)
  #arm.set_joint_value_target(joint_positions)

  arm.go()
  rospy.sleep(3)
  
  arm.set_named_target("up")
  arm.go()
  rospy.sleep(1)
  '''
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()



if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
