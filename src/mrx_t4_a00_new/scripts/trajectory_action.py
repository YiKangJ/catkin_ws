#!/usr/bin/env python
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')

        reset = rospy.get_param('~reset', False)

        arm_joints = ['base_to_armA', 'armA_to_armB', 'armB_to_armC', 'armC_to_armD']

        if reset:
            arm_goal = [0, 0 , 0, 0]
        else:
            arm_goal = [-0.3, -1.0, 0.5, 0.8]
        
        rospy.loginfo("Waiting for arm trajectory controller...")
        arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        arm_client.wait_for_server()
        rospy.loginfo('...connected.')
        
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3)
    
        rospy.loginfo("Moving the arm to goal position..") 
        
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory = arm_trajectory
        arm_goal.goal_time_tolerance = rospy.Duration(0)
        arm_client.send_goal(arm_goal)

        arm_client.wait_for_result(rospy.Duration(5))

        rospy.loginfo('Done.')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
