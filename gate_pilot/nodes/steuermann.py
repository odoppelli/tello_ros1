#!/usr/bin/env python 
import rospy
import geometry_msgs.msg
import std_msgs.msg
import tf2_ros

#   --- node name ---
rospy.init_node('commander')

#   --- Publishers ---
# control effort
command_ship = rospy.Publisher('/tello/cmd_vel',geometry_msgs.msg.Twist, queue_size=1)
# states
state_reader_x = rospy.Publisher('/x_dir/state',std_msgs.msg.Float64, queue_size=1)
state_reader_y = rospy.Publisher('/y_dir/state',std_msgs.msg.Float64, queue_size=1)
state_reader_z = rospy.Publisher('/z_dir/state',std_msgs.msg.Float64, queue_size=1)
is_at_setpoint = rospy.Publisher('/commands/is_at_setpoint', std_msgs.msg.Bool, queue_size=1)

vel_msg = geometry_msgs.msg.Twist()
vel_x = 0
vel_y = 0
vel_z = 0
set_x = None
set_y = None
set_z = None
at_x = None
at_y = None
at_z = None


def setpoint_x(msg):
    global set_x
    set_x = msg.data

def setpoint_y(msg):
    global set_y
    set_y = msg.data

def setpoint_z(msg):
    global set_z
    set_z = msg.data

def check_arrival():
    global set_x
    global set_y
    global set_z
    global at_x
    global at_y
    global at_z
    bounds = 0.05
    if not (set_x is None) and not (at_x is None):
        if (set_x - at_x <= bounds) or (set_x - at_x >= bounds):
            if (set_y - at_y <= bounds) or (set_y - at_y >= bounds):
                if (set_z - at_z <= bounds) or (set_z - at_z >= bounds):
                    is_at_setpoint.publish(True)
                else:
                    is_at_setpoint.publish(False)
            else:
                is_at_setpoint.publish(False)
        else:
            is_at_setpoint.publish(False)
    else:
        is_at_setpoint.publish(False)
    

def send_information(msg):
    global at_x
    global at_y
    global at_z
    at_x = msg.position.x
    at_y = msg.position.y
    at_z = msg.position.z
    state_reader_x.publish(at_x)
    state_reader_y.publish(at_y)
    state_reader_z.publish(at_z)
    #check_arrival()
    

def send_commands_x(msg):
    # x --> y
    global vel_y
    vel_y = msg.data 
    send_commands()

def send_commands_y(msg):
    # y --> z
    global vel_x
    vel_x = msg.data
    send_commands()

def send_commands_z(msg):
    # z --> y
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
    rospy.Subscriber('commands/dutchman_position', geometry_msgs.msg.Pose, send_information)
    rospy.Subscriber('/x_dir/control_effort',std_msgs.msg.Float64,send_commands_x)
    rospy.Subscriber('/y_dir/control_effort',std_msgs.msg.Float64,send_commands_y)
    rospy.Subscriber('/z_dir/control_effort',std_msgs.msg.Float64,send_commands_z)
    rospy.Subscriber('/x_dir/setpoint',std_msgs.msg.Float64,setpoint_x)
    rospy.Subscriber('/y_dir/setpoint',std_msgs.msg.Float64,setpoint_y)
    rospy.Subscriber('/z_dir/setpoint',std_msgs.msg.Float64,setpoint_z)
    

rospy.spin()