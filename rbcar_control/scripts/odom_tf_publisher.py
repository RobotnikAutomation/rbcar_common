#! /usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class OdomToTF(object):

    def __init__(self, odom_topic_name="/odom", publish_rate=30.0):
        self._publish_rate = publish_rate
        # set so that whenever we receive on the odom topic, 
        # the callback method is called
        self.pose = PoseStamped()
        self.seq_num = 0
        self._odom_topic_name = odom_topic_name

        self._check_odom_ready()
        rospy.Subscriber(self._odom_topic_name, Odometry, self.callback)
        
        # initial values are not provided (set to None)
        self.x = None
        self.y = None
    
    def _check_odom_ready(self):
        odom_data = None
        while odom_data is None and not rospy.is_shutdown():
            try:
                odom_data = rospy.wait_for_message(self._odom_topic_name, Odometry, timeout=1.0)
                rospy.logdebug("Current "+str(self._odom_topic_name)+" READY=>" + str(odom_data))

            except:
                rospy.logerr("Current "+str(self._odom_topic_name)+" not ready yet, retrying for getting odom")

        self.process_data(odom_data)   


    def process_data(self,data):
        # Note that we append a new pose to the path ONLY if the position
        # has moved more than 1m from its previous spot (L1 norm)
        self.pose = PoseStamped()
        self.seq_num += 1

        # copy over the values individually
        self.pose.header.frame_id=data.child_frame_id
        self.pose.header.seq= self.seq_num
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = float(data.pose.pose.position.x)
        self.pose.pose.position.y = float(data.pose.pose.position.y)
        self.pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        self.pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        self.pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        self.pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    def callback(self,data):
        self.process_data(data)        


    def get_odom_pose_now(self):
        return self.pose


    def handle_pose(self,pose_msg):
        br = tf.TransformBroadcaster()
        
        br.sendTransform((pose_msg.pose.position.x,pose_msg.pose.position.y,pose_msg.pose.position.z),
                        (pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w),
                        rospy.Time.now(),
                        pose_msg.header.frame_id,
                        "odom")

    def publisher_of_tf(self):
        
        rospy.loginfo("Ready..Starting to Publish TF data now...")
        
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            pose_now = self.get_odom_pose_now()
            self.handle_pose(pose_now)
            rate.sleep()
    

if __name__ == '__main__':
    rospy.init_node('publisher_of_tf_node', anonymous=True)

    obj = OdomToTF()
    try:
        obj.publisher_of_tf()
    except rospy.ROSInterruptException:
        pass