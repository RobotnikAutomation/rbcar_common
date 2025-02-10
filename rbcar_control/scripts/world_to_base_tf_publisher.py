#! /usr/bin/env python
import rospy
import time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class WorldBaseTFPublisher(object):

    def __init__(self):
        rospy.loginfo("Start init WorldBaseTFPublisher Class")
        self._br = tf.TransformBroadcaster()
        rospy.loginfo("set up tf.TransformBroadcaster DONE")
        self._current_pose = Pose()
        rospy.loginfo("_current_pose DONE")
        self.get_init_position()
        rospy.loginfo("get_init_position DONE")
        self._sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.loginfo("self._sub DONE")

    def get_init_position(self):
        data_odom = None
        r = rospy.Rate(2)
        while data_odom is None and not rospy.is_shutdown():
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    # This is to avoid error when world is rested, time when backwards.
                    pass

        self._current_pose = data_odom.pose.pose

    def odom_callback(self, msg):
        self._current_pose = msg.pose.pose

    def get_current_pose(self):
        return self._current_pose

    def handle_tf_pose_rbcar(self, pose_msg, link_name, world_name = "odom"):

        self._br.sendTransform(
                                (pose_msg.position.x,
                                 pose_msg.position.y,
                                 pose_msg.position.z),
                                (pose_msg.orientation.x,
                                 pose_msg.orientation.y,
                                 pose_msg.orientation.z,
                                 pose_msg.orientation.w),
                                rospy.Time.now(),
                                link_name,
                                world_name)

    def publisher_of_tf(self):

        frame_link_name = "/base_footprint"
        time.sleep(1)
        rospy.loginfo("Ready..Starting to Publish TF data now...")

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pose_now = self.get_current_pose()
            if not pose_now:
                rospy.logerr("The Pose is not yet available...Please try again later")
            else:
                self.handle_tf_pose_rbcar(pose_now, frame_link_name)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass


if __name__ == '__main__':
    rospy.init_node('publisher_of_world_base_tf_node', anonymous=True)
    rospy.loginfo("STARTING WORLS TO BASE TF PUBLISHER...")
    world_base_tf_pub = WorldBaseTFPublisher()
    world_base_tf_pub.publisher_of_tf()