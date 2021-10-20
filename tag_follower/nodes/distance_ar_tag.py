#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node('distance')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    distance = rospy.Publisher('commands/ar_distance', geometry_msgs.msg.Pose, queue_size=1)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            transformObject = tfBuffer.lookup_transform('tello_camera_link', 'ar_marker_3', rospy.Time())
            detected = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()
            continue
        msg = geometry_msgs.msg.Pose()

        msg.position.x = transformObject.transform.translation.x
        msg.position.y = transformObject.transform.translation.y
        msg.position.z = transformObject.transform.translation.z

        msg.orientation.x = transformObject.transform.rotation.x
        msg.orientation.y = transformObject.transform.rotation.y
        msg.orientation.z = transformObject.transform.rotation.z
        msg.orientation.w = transformObject.transform.rotation.w
        if detected:
            distance.publish(msg)
            detected = False
        
        else:
            msg.position.x = 0
            msg.position.y = 0
            msg.position.z = 0

            msg.orientation.x = 0
            msg.orientation.y = 0
            msg.orientation.z = 0
            msg.orientation.w = 0
        
        rate.sleep()