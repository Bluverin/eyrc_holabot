#!/usr/bin/env python3

from cmath import pi
from selectors import PollSelector
from turtle import position
import rospy


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion

hola_x = 0
hola_y = 0
hola_theta = 0

x_goals = []
y_goals = []
theta_goals = []

def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def odometryCB(msg):
    global hola_x,hola_y,hola_theta
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y
    hola_theta  = euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))[2]

def main():
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
    rospy.Subscriber("/odom",Odometry,odometryCB)
    rospy.Subscriber("/task1_goals",PoseArray,task1_goals_Cb)
    vel = Twist()
    vel.linear.x, vel.linear.y, vel.angular.z=0, 0, 0
    while len(x_goals)==0:
        publisher.publish(vel)
        rate.sleep()
    i = 0
    operate = 1
    while operate:
        at_goal = 0
        kpx, kpy, kt, kit = 5, 5, 10, 5
        pt, tht = 0.03, 0.017
        while not at_goal:
            err_gf = [x_goals[i]-hola_x, y_goals[i]-hola_y, theta_goals[i]-hola_theta]
            
            a11 = math.cos(hola_theta)
            a12 = math.sin(hola_theta)

            
            err_rf = [0,0,err_gf[2]] 
            err_rf[0] = a11*err_gf[0] + a12*err_gf[1]
            err_rf[1] = -a12*err_gf[0] + a11*err_gf[1]
            err_old = err_rf[2]
            if abs(err_gf[0])<pt:
                kpx = 0
            else:
                kpx = 5    
            if abs(err_rf[1])<pt:
                kpy = 0
            else:
                kpy = 5    
            if abs(err_rf[2])<tht:
                kt = 0 
            else:
                kt = 10     
            if err_rf[2]-err_old < 0.005:
                kit = 0
            vel.linear.x = kpx*err_rf[0]
            vel.linear.y = kpy*err_rf[1]
            vel.angular.z = kt*err_rf[2] + kit*(err_rf[2]-err_old)/100
            publisher.publish(vel)
            
            if kpx==kpy==kt==0:
                at_goal = 1
                i += 1
                begin = rospy.get_time()
                while rospy.get_time() - begin<1:
                    publisher.publish(vel)
                    rate.sleep()    
            
            if i == len(x_goals):
                begin = rospy.get_time()
                while rospy.get_time() - begin<0.5:
                    publisher.publish(vel)
                    rate.sleep()
                if i == len(x_goals):
                    operate = 0
            rate.sleep()        
    
    while not rospy.is_shutdown():
        publisher.publish(vel)
        rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass    

