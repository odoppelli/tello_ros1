#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def handle_black_pearls_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # weird odom shit // splitting in position and orientation
    msg_pos = msg.pose.pose.position
    msg_ori = msg.pose.pose.orientation

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "tello_base_link"
    t.transform.translation.x = msg_pos.x
    t.transform.translation.y = msg_pos.y
    t.transform.translation.z = msg_pos.z
    
    #q = tf_conversions.transformations.quaternion_from_euler(0)

    t.transform.rotation.x = msg_ori.x
    t.transform.rotation.y = msg_ori.y
    t.transform.rotation.z = msg_ori.z
    t.transform.rotation.w = msg_ori.w

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node('tf2_broadcaster')
    rospy.Subscriber('/tello/odom', Odometry, handle_black_pearls_pose)

rospy.spin()