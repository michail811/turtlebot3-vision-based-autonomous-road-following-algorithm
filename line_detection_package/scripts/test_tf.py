#!/usr/bin/env python
from __future__ import division
import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, copysign, sqrt, pow, pi, sin, cos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from transform_utils import quat_to_angle, normalize_angle
from line_detection_package.msg import line_follower

import matplotlib.pyplot as plt


def get_odom(odom_frame, base_frame):
    # returns the x,y,z position of the robot (meters) and the yaw (rads)
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
    return (Point(*trans), quat_to_angle(Quaternion(*rot)))


def get_rotation(orientation_q):
    (roll, pitch, yaw) = euler_from_quaternion(orientation_q)
    return (roll, pitch, yaw)


def get_line_info(msg):
    global line_isDetected, line_orientation, line_error_rad, timee
    line_isDetected = msg.line_detected
    line_orientation = msg.orientation
    line_error_rad = msg.error_rad
    




def g_angle():
    if line_isDetected is True:
        if line_orientation == "Left":
            goal_angle = line_error_rad
        if line_orientation == "Right":
            goal_angle = line_error_rad * (-1)
    else:
        goal_angle = 0.0
    return goal_angle


def vel_Kp_control(goal_position, current_position):
    #Kp_angular = 7.3
    #Kp_angular = 0.8
    ###1.9
    #Kp_linear = 0.5
    dl=0.035
    #0.042
    #0.0422222
    #0.042
    V=0.08
    #V=0.27
    #0.22
    #0.35
    #0.27
    error = goal_position - current_position
    linear_vel=V*cos(error)
    angular_vel=(V/dl)*sin(error)
    #linear_vel = Kp_linear * abs(error)
    #linear_vel=0.03
    #linear_vel=0.5
    print("goal_position",goal_position)
    print("current_position",current_position)
    #angular_vel = Kp_angular * (error)
    
    #linear_vel=0.27
    ###0.08

   
    

    return (linear_vel, angular_vel)

###def printttt(msg):
    #msg.time_det = rospy.Time.now()
    ###publish(msg.time_det)
    #print("rwgwgewvwevweergrae",msg.time_det)
######


rospy.init_node('line_controller', anonymous=False)
# sub = rospy.Subscriber('/odom', Odometry, get_rotation)

cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


move_cmd = Twist()

r = rospy.Rate(20)

odom_frame = '/odom'

tf_listener = tf.TransformListener()

line_detector = line_follower()

rospy.sleep(2)

angular_tolerance = radians(1.0)

line_isDetected = False
line_orientation = "none"
line_error_rad = 0.0
goal_angle = 0.0

# Find out if the robot uses /base_link or /base_footprint
try:
    tf_listener.waitForTransform(odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = '/base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = '/base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
        rospy.signal_shutdown("tf Exception")

position = Point()


(last_position, last_rotation) = get_odom(odom_frame, base_frame)

x_start = position.x
y_start = position.y
# (roll_rad, pitch_rad, yaw_rad) = get_rotation(rotation)
yaw_rad = last_rotation
yaw_deg = (yaw_rad * 180) / pi

print " "
print "the position of the robot is:"
print position
print " "
print "the orientation of the robot is:"
print "yaw in rads: %f , yaw in deg: %f" % (yaw_rad, yaw_deg)
line_detector.error_rad
line_detector.time_det


try:
    while not rospy.is_shutdown():
        (current_position, current_rotation) = get_odom(odom_frame, base_frame)

        sub = rospy.Subscriber('/line_follower/detector', line_follower, get_line_info)

        goal_angle = g_angle()
        print goal_angle
        # print sub_line.orientation
        goal_rotation = current_rotation + goal_angle
        rotation_left = goal_rotation - current_rotation
        (lin_vel, ang_vel) = vel_Kp_control(goal_rotation, current_rotation)
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        print (move_cmd)
        cmd_vel.publish(move_cmd)
        line_detector.time_det=rospy.get_time()
        #rospy.loginfo(line_detector)
        print("DNVO WEV WOES",line_detector.time_det)
        
        r.sleep()

except KeyboardInterrupt:
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)
    print "node terminated"
