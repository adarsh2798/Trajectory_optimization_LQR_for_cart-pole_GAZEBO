#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
import numpy as np

from scipy import linalg
cart_pose = Pose()
pole_pose = Pose()
pole_twist = Twist()
y_angular = 0
cart_pose_x = 0
state = np.zeros(4)

sentry_robot_vel = 0
yaw_angle_target = 0
pitch_angle_target = 0
num = 0

class InvertedPendulum:

    def __init__(self, m1, m2, J, l):
        self.m1 = m1
        self.m2 = m2
        self.J = J
        self.l = l
        self.g = 9.80665

    def state_equation(self, x, F):

        p = x[0]
        theta = x[1]
        dp = x[2]
        dtheta = x[3]

        dx = np.zeros(4)

        dx[0] = dp
        dx[1] = dtheta
        dx[2] = (-self.l*self.m2*np.sin(theta)*dtheta**2 + self.g*self.m2*np.sin(2*theta)/2 + F)/(self.m1 + self.m2*np.sin(theta)**2)
        dx[3] = (self.g*(self.m1 + self.m2)*np.sin(theta) - (self.l*self.m2*np.sin(theta)*dtheta**2 - F)*np.cos(theta))/(self.l*(self.m1 + self.m2*np.sin(theta)**2))

        return dx

    def model_matrix(self):

        alpha = (self.m1 + self.m2)*(self.J + self.m2*self.l**2)- self.m2**2*self.l**2
        A = np.array([ 
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [0, -self.g*self.m2**2*self.l**2/alpha, 0, 0],
                [0, self.m2*self.g*self.l*(self.m1 + self.m2)/alpha, 0, 0]
            ])

        B = np.array([
                [0],
                [0],
                [(self.J + self.m2*self.l**2)/alpha],
                [-self.m2*self.l/alpha]
            ])

        return A, B

def lqr(A, B, Q, R):

    P = linalg.solve_continuous_are(A, B, Q, R)
    K = linalg.inv(R).dot(B.T).dot(P)
    E = linalg.eigvals(A - B.dot(K))

    return P, K, E

def get_cart_pose(data):
    global cart_pose, pole_twist, state
    ind = data.name.index('sentry_robot::yaw_link')
    cart_pose = data.pose[ind]
    state[0] = cart_pose.position.x
    print(f"cart_pose_x : {state[0]}")
    ind_pitch = data.name.index('sentry_robot::pitch_link')
    pole_twist = data.twist[ind_pitch]
    state[2] = pole_twist.linear.x
    state[3] = pole_twist.angular.y
    #print(f"pole twist : \n{pole_twist}")

def commander():
    global sentry_robot_vel, cart_pose, y_angular, cart_pose_x, state

    ip = InvertedPendulum(m1 = 2.7, m2 = 0.86, J = 0.076, l = 0.5)
    A, B = ip.model_matrix()
    Q = np.diag([903, 913, 1880000, 6047000]) 
    R = 1*np.eye(1)
    P, K, E = lqr(A, B, Q, R)
    time_interval = 0.005
    publish_rate = int(1/time_interval)
    pub_yaw = rospy.Publisher('/yaw_controller/command', Float64, queue_size = 10)

    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        state[1] += y_angular*time_interval
        print(f"yaw_angle : {state[1]}")
        u = -np.dot(K, state)
        pub_yaw.publish(u)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)
    commander()
    rospy.spin()
