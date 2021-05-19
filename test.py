#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import math
from math import sin, cos
import numpy as np
from numpy import arccos
from numpy.linalg import norm
import time

robot_callback_init = False
path_callback_init = False


# Robot parameters
b1 = b3 = math.pi/2
b2 = b4 = math.pi/2
y1 = y4 = -math.pi/4
y2 = y3 = math.pi/4
l = 0.27921
a1 = 0.567141
a2=-a1
a3=math.pi-a1
a4=-a3
r = 1

# Inverse kinematics matriz
inv_knmtc = np.array([[math.cos(b1-y1)/math.sin(y1), math.sin(b1-y1)/math.sin(y1), -l*math.sin(b1-y1-a1)/math.sin(y1)],
                      [math.cos(b2-y2)/math.sin(y2), math.sin(b2-y2)/math.sin(y2), -l*math.sin(b2-y2-a2)/math.sin(y2)],
                      [math.cos(b3-y3)/math.sin(y3), math.sin(b3-y3)/math.sin(y3), -l*math.sin(b3-y3-a3)/math.sin(y3)],
                      [math.cos(b4-y4)/math.sin(y4), math.sin(b4-y4)/math.sin(y4), -l*math.sin(b4-y4-a4)/math.sin(y4)]])

# Desired pose
qsi_desired = np.array([[0], [0], [0]])
qsi_desired_d = np.array([[0],[0],[0]])

# Control matrix
A = np.array([[-5, 0, 0],
              [0, -10, 0],
              [0, 0, -15]])
qsi_time = 0


# Callback function of desired path parameters
def callback_path(msg):
    global path_callback_init
    global robot_callback_init

    path_callback_init = True

    while (robot_callback_init and path_callback_init) is not True:
        time.sleep(0.01)

    global qsi_desired
    global qsi_desired_d
    global qsi_time

    # Callback measures
    x = msg.pose.position.x
    y = msg.pose.position.y

    a = msg.pose.orientation.x
    b = msg.pose.orientation.y
    c = msg.pose.orientation.z

    # Find desired angle 
    R = np.array([[cos(b)*cos(c), -cos(b)*sin(c) , sin(b) ],
                  [cos(a)*sin(c)+cos(c)*sin(a)*sin(b), cos(a)*cos(c)-sin(a)*sin(b)*sin(c), -cos(b)*sin(a)],
                  [sin(a)*sin(c)-cos(a)*cos(c)*sin(b), cos(c)*sin(a)+cos(a)*sin(b)*sin(c), cos(a)*cos(b)]])
    z = R.dot(np.array([0,1,0]))
    t = math.atan2(z[1],z[0])

    # Calculate time
    old_qsi_time = qsi_time
    qsi_time = float((msg.header.stamp.secs)+float(msg.header.stamp.nsecs)*10**(-9))


    old_qsi_desired = qsi_desired
    qsi_desired = np.array([[x], [y], [t]])
    qsi_desired_d = (qsi_desired-old_qsi_desired)/(qsi_time-old_qsi_time)
    qsi_desired_d[2][0] = math.atan2(math.sin(qsi_desired_d[2][0]),math.cos(qsi_desired_d[2][0])) # Fix angle between pi and -pi


    

def callback(msg):
    global path_callback_init
    global robot_callback_init

    robot_callback_init = True

    while (robot_callback_init and path_callback_init) is not True:
        time.sleep(0.01)

    # Callback measures
    x = msg.pose.position.x
    y = msg.pose.position.y
    theta = math.atan2(math.sin(msg.pose.orientation.z-math.pi/2),math.cos(msg.pose.orientation.z-math.pi/2))
    qsi_measured = np.array([[x], [y], [theta]])

    # Rotation fixed frame to robot frame
    R = np.array([[math.cos(theta), math.sin(theta), 0],
                  [-math.sin(theta), math.cos(theta), 0],
                  [0,0,1]])

    # Control action
    error = qsi_desired-qsi_measured
    error[2][0] = math.atan2(math.sin(error[2][0]),math.cos(error[2][0])) # Fix angle between pi and -pi
    q = inv_knmtc.dot(R.dot(qsi_desired_d-A.dot(error)))

    # Command action
    vel = Float32MultiArray()
    vel.data = [q[0][0],q[1][0],q[2][0],q[3][0]]
    pub.publish(vel)




if __name__ == '__main__':
    # Init ROS node
    rospy.init_node('test', anonymous=True)

    qsi_time = rospy.get_time()

    # Create publisher to send wheels velocity
    pub = rospy.Publisher('/sim_ros_interface/YouBot_wheels', Float32MultiArray, queue_size=10)
    # Create subscriber to listen to robot location
    sub = rospy.Subscriber("/sim_ros_interface/YouBot_position", PoseStamped, callback)
    # Create subscriber to listen to path location
    sub_path = rospy.Subscriber("/sim_ros_interface/Path_position", PoseStamped, callback_path)

    rospy.spin()
