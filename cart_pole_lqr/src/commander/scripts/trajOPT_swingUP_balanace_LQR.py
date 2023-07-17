#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
import numpy as np
from tf.transformations import euler_from_quaternion
from scipy import linalg
import time
from swingUP_optimal_input_trajOPT import *
from balance_optimal_input_LQR import *
from std_srvs.srv import Empty

def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_sim()
    except rospy.ServiceException as e:
        print("Service call failed:", str(e))

def pause_simulation():
    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_physics()
    except rospy.ServiceException as e:
        print("Service call failed:", str(e))

def resume_simulation():
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()
    except rospy.ServiceException as e:
        print("Service call failed:", str(e))
cart_pose = Pose()
pole_pose = Pose()
pole_twist = Twist()

reset_simulation()

states = np.zeros(4)

TOP_CHECK=0
TOP_cnt=0
entering_TOP=0
u_opt=0


u_d_swingUP=compute_input_swingUP()

K=compute_K()


print("DONE!!")
start=time.time()
def get_states(data):
    global cart_pose, pole_twist, states,TOP_CHECK,TOP_cnt
    ind_cart = data.name.index('sentry_robot::yaw_link')
    cart_pose = data.pose[ind_cart]
    states[0] = cart_pose.position.x
   
    ind_pole = data.name.index('sentry_robot::pitch_link')
    pole_pose=data.pose[ind_pole]
    ori=pole_pose.orientation
    oril=[ori.x,ori.y,ori.z,ori.w]
    ori_eul=euler_from_quaternion(oril)
    pole_twist = data.twist[ind_pole]
    states[1]=ori_eul[1]
    states[2] = pole_twist.linear.x
    states[3] = pole_twist.angular.y
    #print(ori_eul[0]*180/np.pi,"       ", ori_eul[1]*180/np.pi,"            ",ori_eul[2]*180/np.pi)
    
    if(abs(ori_eul[1]*180/np.pi)<20 and ori_eul[0]*180/np.pi<-100): 
         TOP_CHECK=1
         TOP_cnt+=1
     
    #else:
    #   TOP_CHECK=0
    #print(f"pole twist : \n{pole_twist}")

def controller():
    global states,start,TOP_CHECK,u_opt,u_d_balance,TOP_cnt,entering_TOP,u_d_swingUP

    
    time_interval = 0.0001
    publish_rate = int(1/time_interval)
    pub_yaw = rospy.Publisher('/yaw_controller/command', Float64, queue_size = 10)

    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        
        t_curr=time.time()-start
        if TOP_CHECK==0:
                u_opt=get_interpolated_u_swingUP(t_curr,u_d_swingUP)
        else:
            u_opt=-np.dot(K,states)
                
            
             
        pub_yaw.publish(u_opt)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_states)
    controller()
    rospy.spin()
