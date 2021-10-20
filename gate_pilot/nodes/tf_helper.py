#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import geometry_msgs.msg


if __name__ == '__main__':

    rospy.init_node('tf_zero_helper')

    tfBuffer1 = tf2_ros.Buffer()
    listener1 = tf2_ros.TransformListener(tfBuffer1)
    tfBuffer2 = tf2_ros.Buffer()
    listener2 = tf2_ros.TransformListener(tfBuffer2)
    broadcaster = tf2_ros.TransformBroadcaster()
    transformStamped = geometry_msgs.msg.TransformStamped()

    rate = rospy.Rate(30.0)
    transformStamped.header.frame_id = 'my_zero'
    transformStamped.child_frame_id = 'flying_dutchman'
    quat = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
    while not rospy.is_shutdown():
        try:
            transformObj1 = tfBuffer1.lookup_transform('world','my_zero', rospy.Time())
            transformObj2 = tfBuffer2.lookup_transform('world','tello_base_link', rospy.Time())
            t_x = transformObj1.transform.translation.x - transformObj2.transform.translation.x
            t_y = transformObj1.transform.translation.y - transformObj2.transform.translation.y
            t_z = transformObj1.transform.translation.z - transformObj2.transform.translation.z
            #quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            
            transformStamped.header.stamp = rospy.Time.now()
            transformStamped.transform.translation.x = t_x
            transformStamped.transform.translation.y = t_y
            transformStamped.transform.translation.z = t_z
            transformStamped.transform.rotation.x = transformObj2.transform.rotation.x
            transformStamped.transform.rotation.y = transformObj2.transform.rotation.y
            transformStamped.transform.rotation.z = transformObj2.transform.rotation.z
            transformStamped.transform.rotation.w = transformObj2.transform.rotation.w
            broadcaster.sendTransform(transformStamped)
            rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()
            continue
    