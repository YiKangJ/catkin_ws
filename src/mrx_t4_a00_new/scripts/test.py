#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
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
    
  arm.set_goal_position_tolerance(0.00001);
  #arm.set_planner_id("BKPIECEkConfigDefault") 2
  #arm.set_planner_id("BFMTkConfigDefault") 2
  #arm.set_planner_id("BiESTkConfigDefault")
  #arm.set_planner_id("BiTRRTkConfigDefault")
  #arm.set_planner_id("BKPIECEkConfigDefault")
  #arm.set_planner_id("BKPIECEkConfigDefault")
    
  arm.set_named_target("right")
  arm.go()
  rospy.sleep(1)
  '''
  pose_target = geometry_msgs.msg.Pose()
  
  pose_target.position.x = 0.45
  pose_target.position.y = -0.234
  pose_target.position.z = 0.42

  pose_target.orientation.x = 0.707106781186547
  pose_target.orientation.y = 0
  pose_target.orientation.z = 0
  pose_target.orientation.w = 0.707106781186547
  '''
  for i in range(50):
    arm.set_start_state_to_current_state()
    #arm.set_pose_target(pose_target)
    x = random.uniform(0.0, 0.35)
    y = random.uniform(0, 0.38)
    if random.randint(1, 2) == 2:
        y = -y
    z = random.uniform(0.2, 0.35)
    arm.set_position_target([x, y, z])
    print [x, y, z]
    arm.go()
    rospy.sleep(2)

  arm.set_named_target("up")
  arm.go()
  #rospy.sleep(1)
  
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
