#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import std_msgs.msg

#   --- node name ---
rospy.init_node('commander')

#   --- Publishers ---
# control effort
command_ship = rospy.Publisher('/tello/cmd_vel',geometry_msgs.msg.Twist, queue_size=1)
# states
state_reader_x = rospy.Publisher('/x_dir/state',std_msgs.msg.Float64, queue_size=1)
state_reader_y = rospy.Publisher('/y_dir/state',std_msgs.msg.Float64, queue_size=1)
state_reader_z = rospy.Publisher('/z_dir/state',std_msgs.msg.Float64, queue_size=1)
# setpoints
setpoint_x = rospy.Publisher('/x_dir/setpoint', std_msgs.msg.Float64, queue_size=1)
setpoint_y = rospy.Publisher('/y_dir/setpoint', std_msgs.msg.Float64, queue_size=1)
setpoint_z = rospy.Publisher('/z_dir/setpoint', std_msgs.msg.Float64, queue_size=1)


vel_msg = geometry_msgs.msg.Twist()

vel_x = 0
vel_y = 0
vel_z = 0
setpoint_set = False

def send_information(msg):
    distance_x = msg.position.x 
    distance_y = msg.position.y
    distance_z = msg.position.z
    
    # setpoints
    global setpoint_set
    if (not setpoint_set):
        diff_x = 0.0
        diff_y = 1.0
        diff_z = -0.2
        setpoint_x.publish(diff_x)
        setpoint_y.publish(diff_y)
        setpoint_z.publish(diff_z)
        setpoint_set = True

    # states
    # cmd x --> ar -x
    state_reader_x.publish(distance_x)
    # cmd y --> ar -z
    state_reader_y.publish(distance_z)
     # cmd z --> ar y
    state_reader_z.publish(distance_y)

    
def send_commands_x(msg):
    # cmd x --> ar -x
    global vel_x
    vel_x = msg.data * -1
    send_commands()

def send_commands_y(msg):
    # cmd y --> ar -z
    global vel_y
    vel_y = msg.data *-1
    send_commands()

def send_commands_z(msg):
    # cmd z --> ar y
    global vel_z
    vel_z = msg.data
    send_commands()

def send_commands():
    global vel_x
    global vel_y
    global vel_z
    vel_msg.linear.x = vel_x
    vel_msg.linear.y = vel_y
    vel_msg.linear.z = vel_z
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    command_ship.publish(vel_msg)

if __name__ == "__main__":
    # Subscribers
    rospy.Subscriber('/commands/ar_distance',geometry_msgs.msg.Pose,send_information)
    rospy.Subscriber('/x_dir/control_effort',std_msgs.msg.Float64,send_commands_x)
    rospy.Subscriber('/y_dir/control_effort',std_msgs.msg.Float64,send_commands_y)
    rospy.Subscriber('/z_dir/control_effort',std_msgs.msg.Float64,send_commands_z)
    

rospy.spin()