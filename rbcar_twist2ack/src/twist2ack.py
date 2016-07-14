#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def cmd_vel_callback(msg):
	
	spin = 1.0
	ack_msg.header.stamp = rospy.Time.now()
	ack_msg.header.frame_id = "odom"
	# to keep angular speed sign moving backwards, the sign needs to be inverted (for teb). 
	if msg.linear.x >0.0:
		 spin = 1.0
	else:
		 spin = -1.0
	ack_msg.drive.steering_angle = msg.angular.z * spin * 5.0;		
	ack_msg.drive.steering_angle_velocity = 0.0
	ack_msg.drive.speed = msg.linear.x
	ack_msg.drive.acceleration = 0.0
	ack_msg.drive.jerk = 0.0
	#rospy.loginfo ("angular.z=%s", msg.data.angular.z)


ack_msg = AckermannDriveStamped()
cmd_sub = rospy.Subscriber('/cmd_vel_out', Twist, cmd_vel_callback)	
ack_pub = rospy.Publisher('/rbcar_robot_control/command', AckermannDriveStamped, latch=True)
rospy.init_node('twist2ack')
	
r = rospy.Rate(10)
while not rospy.is_shutdown():
	ack_pub.publish(ack_msg)
	r.sleep()
rospy.loginfo("node has shutdown!")

    
 

