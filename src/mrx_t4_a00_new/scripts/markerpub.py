#!/usr/bin/env python
import rospy, sys
import moveit_commander
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

def markerPub():
	
	# Create a marker publisher.
	marker_puber = rospy.Publisher('end_effector_trail', Marker, queue_size=10)
	rospy.init_node('markerPub', anonymous=True)
	rate = rospy.Rate(10)
	
	# Get the trails of the end effector from moveit API.
	moveit_commander.roscpp_initialize(sys.argv)
	right_arm = MoveGroupCommander('arm')
	right_arm.set_pose_reference_frame('base_link')
	
	# Init the Marker and clear the trails created before.
	marker = Marker()
	marker.points = []
	marker.ns = "my_namespace"
	marker.header.frame_id = 'base_link'
	marker.id = 0
	marker.type = 4
	marker.action = Marker.ADD
	marker.scale.x = 0.005
	marker.scale.y = 0
	marker.scale.z = 0
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	marker_puber.publish(marker)

	# Publish trails contantly.
	while not rospy.is_shutdown():
		
		marker.header.stamp = rospy.Time().now()
		
		# points and line type use marker.point and arrow et. use pose
		marker.points.append(right_arm.get_current_pose().pose.position)
		#marker.pose = right_arm.get_current_pose().pose.position
		#marker.pose.orientation.x = 0
		#marker.pose.orientation.y = 0
		#marker.pose.orientation.z = 0
		#marker.pose.orientation.w = 1
		 
		marker_puber.publish(marker)
		rate.sleep()
	
    # Exit the relevant process.
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)


if __name__ == "__main__":
	try:
		markerPub()
	except rospy.ROSInterruptException:	
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
