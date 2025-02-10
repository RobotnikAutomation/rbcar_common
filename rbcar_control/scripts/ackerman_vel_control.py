#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import sys, getopt, math
from sensor_msgs.msg import JointState

class cmdvel2gazebo:

    def __init__(self,car_wheel_base, car_wheel_threat, max_abs_steer, wheel_radius):
        
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

        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()

        # we want maxsteer to be that of the "inside" tire, and since it is 0.6 in gazebo, we
        # set our ideal steering angle max to be less than that, based on geometry
        self.maxsteerInside=max_abs_steer
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rIdeal = self.L/math.tan(self.maxsteerInside)
        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        self.rMin = rIdeal+(self.T/2.0)

        self._check_cmd_vel_ready()

        self.joint_states_topic_name = "/rbcar/joint_states"
        rbcar_joints_data = self._check_joint_states_ready()
        if rbcar_joints_data is not None:
            self.rbcar_joint_dictionary = dict(zip(rbcar_joints_data.name, rbcar_joints_data.position))
            rospy.Subscriber(self.joint_states_topic_name, JointState, self.rbcar_joints_callback)
        else:
            assert False, "error in Joint States"

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
        self.publish_steer()
        

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
        if data.angular.z != 0.0:
            steering_radius_raw = abs(self.linear_velocity / data.angular.z)
            self.steering_radius = max(abs(steering_radius_raw), self.rMin)
            # We consider that turning left should be positive
            self.turning_sign = -1 * math.copysign(1,data.angular.z)
            self.omega_turning_speed = self.linear_velocity / self.steering_radius
        else:
            self.steering_radius = -1
            self.turning_sign = 0.0
            self.omega_turning_speed = 0.0
        
        self.lastMsg = rospy.Time.now()
    
    def callback(self,data):
        """
        We get the linear velocity and the desired Turning Angurlar velocity.
        We have to convert it to Turning Radius
        """
        self.process_cmd_vel_data(data)
        


    def publish(self):

        # delta_last_msg_time = rospy.Time.now() - self.lastMsg

        # msgs_too_old = delta_last_msg_time > self.timeout
        # if msgs_too_old:
        #     msgRearFront = Float64()
        #     msgFrontSteer = Float64()

        #     self.pub_rearL.publish(msgRearFront)
        #     self.pub_rearR.publish(msgRearFront)
            
        #     self.pub_steerR.publish(msgRearFront)
        #     self.pub_steerL.publish(msgRearFront)

        #     self.pub_steerR.publish(msgFrontSteer)
        #     self.pub_steerL.publish(msgFrontSteer)

        #     rospy.logerr("Message Too OLD")

        if self.steering_radius >= 0:

            rospy.logdebug("TURNING")

            msgRearR = Float64()
            msgRearL = Float64()

            auxi = (self.steering_radius +(  ( -1 * self.turning_sign)*(self.T / 2.0) )  ) 
            linear_vel_interior = self.omega_turning_speed * auxi

            auxe = (self.steering_radius +(  ( 1 * self.turning_sign)*(self.T / 2.0) )  ) 
            linear_vel_exterior = self.omega_turning_speed * auxe

            wheel_turnig_speed_interior = linear_vel_interior / self.wheel_radius
            wheel_turnig_speed_exterior = linear_vel_exterior / self.wheel_radius
            
            msgRearR.data = wheel_turnig_speed_interior
            msgRearL.data = wheel_turnig_speed_exterior            

            #### Front Steering Wheels

            msgFrontR = Float64()
            msgFrontL = Float64()

            
            distance_to_turning_point_interior = math.sqrt(pow(self.L,2)+pow(auxi,2))

            
            distance_to_turning_point_exterior = math.sqrt(pow(self.L,2)+pow(auxe,2))

            linear_vel_front_interior = self.omega_turning_speed * distance_to_turning_point_interior
            linear_vel_front_exterior = self.omega_turning_speed * distance_to_turning_point_exterior

            wheel_turnig_speed_front_interior = linear_vel_front_interior / self.wheel_radius
            wheel_turnig_speed_front_exterior = linear_vel_front_exterior / self.wheel_radius

            msgFrontL.data = wheel_turnig_speed_front_exterior
            msgFrontR.data = wheel_turnig_speed_front_interior


            #### Steering Position

            msgFrontSteerR = Float64()
            msgFrontSteerL = Float64()

            alfa_interior = math.atan(self.L / auxi)
            alfa_exterior = math.atan(self.L / auxe)

            msgFrontSteerR.data = -1 * self.turning_sign * alfa_interior
            msgFrontSteerL.data = -1 * self.turning_sign * alfa_exterior


            #### We publish all the messages

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)
            
            self.pub_frontR.publish(msgFrontR)
            self.pub_frontL.publish(msgFrontL)

            self.update_steer_values(msgFrontSteerR.data, msgFrontSteerL.data)
            # self.pub_steerR.publish(msgFrontSteerR)
            # self.pub_steerL.publish(msgFrontSteerL)

        else:
            rospy.logdebug("NOT TURNING")
            # if we aren't turning, everything is easy!
            msgRearFront = Float64()
            msgFrontSteer = Float64()
            msgRearFront.data = self.linear_velocity / self.wheel_radius
            msgFrontSteer.data = 0.0

            self.pub_rearL.publish(msgRearFront)
            self.pub_rearR.publish(msgRearFront)
            
            self.pub_frontR.publish(msgRearFront)
            self.pub_frontL.publish(msgRearFront)

            self.update_steer_values(msgFrontSteer.data, msgFrontSteer.data)
            # self.pub_steerR.publish(msgFrontSteer)
            # self.pub_steerL.publish(msgFrontSteer)
    
    def update_steer_values(self, SteerR, SteerL):

        self.steer_r = SteerR
        self.steer_l = SteerL

    def publish_steer(self):

        msgFrontSteerRVel = Float64()
        msgFrontSteerLVel = Float64()

        curent_steer_r = self.rbcar_joint_dictionary["front_right_steer"]
        delta_r = curent_steer_r - self.steer_r 
        curent_steer_l = self.rbcar_joint_dictionary["front_left_steer"]
        delta_l = curent_steer_l - self.steer_l

        rospy.logdebug("####################################")
        rospy.logdebug("curent_steer_r="+str(curent_steer_r))
        rospy.logdebug("curent_steer_l="+str(curent_steer_l))
        rospy.logdebug("self.steer_r ="+str(self.steer_r ))
        rospy.logdebug("self.steer_l ="+str(self.steer_l))
        rospy.logdebug("delta_r="+str(delta_r))
        rospy.logdebug("delta_l="+str(delta_l))
        rospy.logdebug("####################################")

        if delta_r > self.acceptable_steer_error:
            print("STEER Left")
            msgFrontSteerRVel.data = -1.0 * self.max_steering_speed
        elif delta_r < self.acceptable_steer_error:
            print("STEER Right")
            msgFrontSteerRVel.data = self.max_steering_speed
        else:
            print("STOP STEERING")
            msgFrontSteerRVel.data = 0.0

        if delta_l > self.acceptable_steer_error:
            print("STEER Left")
            msgFrontSteerLVel.data = -1.0 * self.max_steering_speed
        elif delta_l < self.acceptable_steer_error:
            print("STEER Right")
            msgFrontSteerLVel.data = self.max_steering_speed
        else:
            print("STOP STEERING")
            msgFrontSteerLVel.data = 0.0

        self.pub_steerR.publish(msgFrontSteerRVel)
        self.pub_steerL.publish(msgFrontSteerLVel)

def main():
    # we eventually get the ns (namespace) from the ROS parameter server for this node
    rospy.init_node('cmdvel2gazebo', anonymous=True, log_level=rospy.DEBUG)

    # Distance from Front to Rear axel
    car_wheel_base = 1.82278

    # Distance from left to right wheels
    car_wheel_threat = 1.02996

    # Calculated as the  maximum steering angle the inner wheel can do
    max_abs_steer = 0.7

    wheel_radius = 0.634 / 2.0
    node = cmdvel2gazebo(car_wheel_base, car_wheel_threat, max_abs_steer, wheel_radius)
    rate = rospy.Rate(50) # run at 10Hz
    while not rospy.is_shutdown():
        node.publish()
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn("Reseted Time...")
            pass

if __name__ == '__main__':
    main()
    
