#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import sys, getopt, math
from sensor_msgs.msg import JointState
import os

class cmdvel2gazebo:

    def __init__(self,car_wheel_base, car_wheel_threat, max_abs_steer, wheel_radius, max_wheel_turn_speed):
        
        rospy.Subscriber('/cmd_vel', Twist, self.callback)
        
        self.pub_steerL = rospy.Publisher('/rbcar/front_left_steer_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/rbcar/front_right_steer_position_controller/command', Float64, queue_size=1)
        
        
        self.pub_rearL = rospy.Publisher('/rbcar/back_joint_left_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/rbcar/back_joint_right_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontL = rospy.Publisher('/rbcar/front_joint_left_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontR = rospy.Publisher('/rbcar/front_joint_right_wheel_velocity_controller/command', Float64, queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0
        
        self.L = car_wheel_base
        self.T=car_wheel_threat
        self.wheel_radius = wheel_radius
        self._max_wheel_turn_speed = max_wheel_turn_speed

        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()

        # we want maxsteer to be that of the "inside" tire, and since it is 0.6 in gazebo, we
        # set our ideal steering angle max to be less than that, based on geometry
        self.maxsteerInside=max_abs_steer
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        R_Min_interior = self.L/math.tan(self.maxsteerInside)
        self.R_Min_baselink = R_Min_interior + (self.T / 2.0)
        # radius of inside tire is rMax, so radius of the ideal middle tire (R_MIN) is rMax+treadwidth/2
        # self.rMin = R_MIN+(self.T/2.0)
        rospy.logwarn("################ MINIMUM TURNING RADIUS ACKERMAN==="+str(self.R_Min_baselink))

        self._check_cmd_vel_ready()

        # self.joint_states_topic_name = "/rbcar/joint_states"
        # rbcar_joints_data = self._check_joint_states_ready()
        # if rbcar_joints_data is not None:
        #     self.rbcar_joint_dictionary = dict(zip(rbcar_joints_data.name, rbcar_joints_data.position))
        #     rospy.Subscriber(self.joint_states_topic_name, JointState, self.rbcar_joints_callback)
        # else:
        #     assert False, "error in Joint States"

        # rad / sec
        self.max_steering_speed = 2.0
        self.acceptable_steer_error = 0.1

    def _check_joint_states_ready(self):
        self.joint_states = None
        rospy.logdebug("Waiting for /joint_states to be READY...")
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(self.joint_states_topic_name, JointState, timeout=5.0)
                rospy.logdebug("Current "+str(self.joint_states_topic_name)+" READY=>")

            except:
                rospy.logerr("Current "+str(self.joint_states_topic_name)+" not ready yet, retrying")

        return self.joint_states
    
    def rbcar_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.rbcar_joint_dictionary = dict(zip(msg.name, msg.position))
        

    def _check_cmd_vel_ready(self):
        data = None
        while data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message('/cmd_vel', Twist, timeout=1.0)
                self.process_cmd_vel_data(data)
                rospy.logdebug("Current cmd_vel READY=>")

            except:
                rospy.logwarn("Current cmd_vel not ready yet, retrying...")

        
    def process_cmd_vel_data(self,data):
        self.linear_velocity = data.linear.x        
        # We limit the minimum value of the Steering Radius
        # Todo Process negatives

        rospy.loginfo("self.linear_velocity="+str(self.linear_velocity))
        rospy.loginfo("data.angular.z="+str(data.angular.z))

        if data.angular.z != 0.0:
            steering_radius_raw = abs(self.linear_velocity / data.angular.z)
            rospy.loginfo("steering_radius_raw="+str(steering_radius_raw))
            rospy.loginfo("R_Min_baselink="+str(self.R_Min_baselink))

            self.steering_radius = max(abs(steering_radius_raw), self.R_Min_baselink)
            # We consider that turning left should be positive
            self.turning_sign = -1 * math.copysign(1,data.angular.z)
            # Going Fowards is positive
            self.linear_sign = math.copysign(1,self.linear_velocity)
            self.omega_turning_speed = self.linear_velocity / self.steering_radius
        else:
            self.steering_radius = -1
            self.turning_sign = 0.0
            self.linear_sign = 0.0
            self.omega_turning_speed = 0.0

        self.lastMsg = rospy.Time.now()
    
    def callback(self,data):
        """
        We get the linear velocity and the desired Turning Angurlar velocity.
        We have to convert it to Turning Radius
        """
        self.process_cmd_vel_data(data)
        self.publish()
        

    def limit_wheel_speed(self, in_speed):

        if in_speed > self._max_wheel_turn_speed:
                rospy.logwarn("MAX Wheel Speed!")
                in_speed = self._max_wheel_turn_speed
        elif in_speed < -1.0 *self._max_wheel_turn_speed:
            rospy.logwarn("MAX Wheel Speed!")
            in_speed = -1.0* self._max_wheel_turn_speed
        
        return in_speed

    def publish(self):

        # Step 1 Calculate the Wheel Turning Speed for the Rear Wheels
        # For that we have the following input data of the base_link
        # Filtered if it was impossible for the system to do it
        # self.turning_sign states which side system want to turn
        # and who is the exterior interior wheel ( default interior = right wheel. exterior = left wheel )
        vel_base_link = self.linear_velocity
        omega_base_link = self.omega_turning_speed
        turning_radius_base_link = self.steering_radius

        turning_radius_right_rear_wheel = None
        turning_radius_left_rear_wheel = None

        if self.steering_radius >= 0:
            # Default Interior = Right WHeel
            # Make this sign multiplication because when going backwards angular is inverted, so it ha sto invert the sign
            turning_radius_right_rear_wheel = turning_radius_base_link + ( -1 * self.turning_sign * self.linear_sign) * (self.T / 2.0)
            vel_right_rear_wheel = omega_base_link * turning_radius_right_rear_wheel
            wheel_turnig_speed_right_rear_wheel = self.limit_wheel_speed(vel_right_rear_wheel / self.wheel_radius)

            # Default Interior = Left WHeel
            # Make this sign multiplication because when going backwards angular is inverted, so it ha sto invert the sign
            turning_radius_left_rear_wheel = turning_radius_base_link + ( 1 * self.turning_sign * self.linear_sign) * (self.T / 2.0)
            vel_left_rear_wheel = omega_base_link * turning_radius_left_rear_wheel
            wheel_turnig_speed_left_rear_wheel = self.limit_wheel_speed(vel_left_rear_wheel / self.wheel_radius)
        else:
            # Not turning , there fore they are all the same
            # Default Interior = Right WHeel
            wheel_turnig_speed_right_rear_wheel = self.limit_wheel_speed(vel_base_link / self.wheel_radius)
            # Default Interior = Left WHeel
            wheel_turnig_speed_left_rear_wheel = self.limit_wheel_speed(vel_base_link / self.wheel_radius)

        #### END REAR WHeel Calculations

        # Step 2: Calculate the Wheel Turning Speed for the Front wheels and the STeering angle
        if self.steering_radius >= 0:
            turning_radius_right_front_wheel = turning_radius_right_rear_wheel
            distance_to_turning_point_right_front_wheel = math.sqrt(pow(self.L,2)+pow(turning_radius_right_front_wheel,2))
            vel_right_front_wheel = omega_base_link * distance_to_turning_point_right_front_wheel
            
            wheel_turnig_speed_right_front_wheel = self.limit_wheel_speed(vel_right_front_wheel / self.wheel_radius)
            alfa_right_front_wheel = math.atan(self.L / turning_radius_right_front_wheel)


            turning_radius_left_front_wheel = turning_radius_left_rear_wheel
            distance_to_turning_point_left_front_wheel = math.sqrt(pow(self.L,2)+pow(turning_radius_left_front_wheel,2))
            vel_left_front_wheel = omega_base_link * distance_to_turning_point_left_front_wheel
            wheel_turnig_speed_left_front_wheel = self.limit_wheel_speed(vel_left_front_wheel / self.wheel_radius)
            alfa_left_front_wheel = math.atan(self.L / turning_radius_left_front_wheel)
        else:
            wheel_turnig_speed_right_front_wheel = self.limit_wheel_speed(vel_base_link / self.wheel_radius)
            alfa_right_front_wheel = 0.0

            wheel_turnig_speed_left_front_wheel = self.limit_wheel_speed(vel_base_link / self.wheel_radius)
            alfa_left_front_wheel = 0.0
        #### END FRONT WHeel Calculations

        #os.system('clear') 
        rospy.loginfo("#####################")
        rospy.loginfo("@ INPUT VALUES @")
        rospy.loginfo("vel_base_link="+str(vel_base_link))
        rospy.loginfo("omega_base_link="+str(omega_base_link))
        rospy.loginfo("turning_radius_base_link="+str(turning_radius_base_link))
        rospy.loginfo("@ TURNING SPEEDS @")
        rospy.loginfo("wheel_turnig_speed_right_rear_wheel="+str(wheel_turnig_speed_right_rear_wheel))
        rospy.loginfo("wheel_turnig_speed_left_rear_wheel="+str(wheel_turnig_speed_left_rear_wheel))
        rospy.loginfo("wheel_turnig_speed_right_front_wheel="+str(wheel_turnig_speed_right_front_wheel))
        rospy.loginfo("wheel_turnig_speed_left_front_wheel="+str(wheel_turnig_speed_left_front_wheel))
        rospy.loginfo("@ ANGLES @")
        rospy.loginfo("alfa_right_front_wheel="+str(alfa_right_front_wheel))
        rospy.loginfo("alfa_left_front_wheel="+str(alfa_left_front_wheel))
        rospy.loginfo("####### END #########")
        

        # Step 3 Publish all teh data in the correponding topics

        msgRearR = Float64()
        msgRearL = Float64()
        msgFrontR = Float64()
        msgFrontL = Float64()
        msgFrontSteerR = Float64()
        msgFrontSteerL = Float64()

        
        msgRearR.data = wheel_turnig_speed_right_rear_wheel
        msgRearL.data = wheel_turnig_speed_left_rear_wheel
        msgFrontR.data = wheel_turnig_speed_right_front_wheel
        msgFrontL.data = wheel_turnig_speed_left_front_wheel

        msgFrontSteerR.data = -1 * self.turning_sign * self.linear_sign * alfa_right_front_wheel
        msgFrontSteerL.data = -1 * self.turning_sign * self.linear_sign * alfa_left_front_wheel



        #### We publish all the messages

        self.pub_rearL.publish(msgRearL)
        self.pub_rearR.publish(msgRearR)
        
        self.pub_frontR.publish(msgFrontR)
        self.pub_frontL.publish(msgFrontL)

        self.pub_steerR.publish(msgFrontSteerR)
        self.pub_steerL.publish(msgFrontSteerL)

    

def main():
    # we eventually get the ns (namespace) from the ROS parameter server for this node
    rospy.init_node('cmdvel2gazebo', anonymous=True, log_level=rospy.INFO)

    # Distance from Front to Rear axel
    car_wheel_base = 1.82278

    # Distance from left to right wheels
    car_wheel_threat = 1.02996

    # Calculated as the  maximum steering angle the inner wheel can do
    max_abs_steer = 0.6108

    wheel_radius = 0.634 / 2.0

    # Radiand per second, that with the current wheel radius would make 44 Km/h max linear vel
    max_wheel_turn_speed = 39.0

    node = cmdvel2gazebo(car_wheel_base, car_wheel_threat, max_abs_steer, wheel_radius, max_wheel_turn_speed)
    rate = rospy.Rate(10) # run at 10Hz
    while not rospy.is_shutdown():
        # node.publish()
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn("Reseted Time...")
            pass

if __name__ == '__main__':
    main()
    
