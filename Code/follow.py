#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, asin
from turtle import position
import numpy as np
import math as m
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
    def __init__(self):
    # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size = 10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback,queue_size = 1)
        
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)
        self.x_req = [0,0,0,0]
        self.y_req = [0,0,0,0]
        self.z_req = [0,0,0]
        self.omega = 0
  
    def traj_eval(self):
        t = self.t
        t2 = t ** 2
        t3 = t ** 3
        t4 = t ** 4
        t5 = t ** 5

        if (t<5):
            xd = 0
            yd = 0
            zd = (6 * t5)/3125 - (3 * t4)/125 + (2 * t3)/25
            xd_dot = 0
            yd_dot = 0
            zd_dot = (6 * t4)/625 - (12 * t3)/125 + (6 * t2)/25
            xd_ddot = 0
            yd_ddot = 0
            zd_ddot = (24 * t3)/625 - (36 * t2)/125 + (12 * t)/25

        elif (t<20):
            xd = (2 * t5)/253125 - t4/2025 + (22 * t3)/2025 - (8 * t2)/81 + (32 * t)/81 - 47/81
            yd = 0
            zd = 1
            xd_dot = (2 * t4)/50625 - (4 * t3)/2025 + (22 * t2)/675 - (16 * t)/81 + 32/81
            yd_dot = 0
            zd_dot = 0
            xd_ddot = (8 * t3)/50625 - (4 * t2)/675 + (44 * t)/675 - 16/81
            yd_ddot = 0
            zd_ddot = 0

        elif (t<35):
            xd = 1
            yd = (2 * t5)/253125 - (11 * t4)/10125 + (118 * t3)/2025 - (616 * t2)/405 + (1568 * t)/81 - 7808/81
            zd = 1
            xd_dot = 0
            yd_dot = (2 * t4)/50625 - (44 * t3)/10125 + (118 * t2)/675 - (1232 * t)/405 + 1568/81
            zd_dot = 0
            xd_ddot = 0
            yd_ddot = (8 * t3)/50625 - (44 * t2)/3375 + (236 * t)/675 - 1232/405
            zd_ddot = 0

        elif (t<50):
            xd = - (2 * t5)/253125 + (17 * t4)/10125 - (286 * t3)/2025 + (476 * t2)/81 - (9800 * t)/81 + 80000/81
            yd = 1
            zd = 1
            xd_dot = - (2 * t4)/50625 + (68 * t3)/10125 - (286 * t2)/675 + (952 * t)/81 - 9800/81
            yd_dot = 0
            zd_dot = 0
            xd_ddot = - (8 * t3)/50625 + (68 * t2)/3375 - (572 * t)/675 + 952/81
            yd_ddot = 0
            zd_ddot = 0

        elif (t<= 65):
            xd = 0
            yd = -(2 * t5)/253125 + (23 * t4)/10125 - (526 * t3)/2025 + (1196 * t2)/81 - (33800 * t)/81 + 380081/81
            zd = 1
            xd_dot = 0
            yd_dot = -(2 * t4)/50625 + (92 * t3)/10125 - (526 * t2)/675 + (2392 * t)/81 - 33800/81
            zd_dot = 0
            xd_ddot = 0
            yd_ddot = -(8 * t3)/50625 + (92 * t2)/3375 - (1052 * t)/675 + 2392/81
            zd_ddot = 0
        
        else:
            xd = 0
            yd = 0
            zd = 1
            xd_dot = 0
            yd_dot = 0
            zd_dot = 0
            xd_ddot = 0
            yd_ddot = 0
            zd_ddot = 0

        return xd,yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot
    
    def signum(self,value):
        phi = 0.9
        if value < 0:
            sgn = -1
        elif value > 0:
            sgn = 1
        if abs(value) > phi:
            return sgn 
        return value/phi 
    
    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        m = 27 * 1e-3
        l = 46 * 1e-3
        Ix = 16.571710 * 1e-6
        Iy = 16.571710 * 1e-6
        Iz = 29.261652 * 1e-6
        Ip = 12.65625 * 1e-8
        KF = 1.28192 * 1e-8
        KM = 5.964552 * 1e-3
        g = 9.81
        k = [12,380,400,4]
        K = [50,10,30,10] # PD parametrs
        rho = 0
        lamda = [7,9,9,7]

        xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate()
        
        while(rpy[0,0]<-pi):
            rpy[0,0] = rpy[0,0]+2 * pi
        while(rpy[0,0]>pi):
            rpy[0,0] = rpy[0,0]-2 * pi
        while(rpy[1,0]<-pi):
            rpy[1,0] = rpy[1,0]+2 * pi
        while(rpy[1,0]>pi):
            rpy[1,0] = rpy[1,0]-2 * pi
        while(rpy[2,0]<-pi):
            rpy[2,0] = rpy[2,0]+2 * pi
        while(rpy[2,0]>pi):
            rpy[2,0] = rpy[2,0]-2 * pi
        
        x = xyz[0,0]
        y = xyz[1,0]
        z = xyz[2,0]
        dx = xyz_dot[0,0]
        dy = xyz_dot[1,0]
        dz = xyz_dot[2,0]
        phi = rpy[0,0]
        psi = rpy[2,0]
        theta = rpy[1,0]
        phi_dot = rpy_dot[0,0]
        psi_dot = rpy_dot[2,0]
        theta_dot = rpy_dot[1,0]

        e1 = zd-z
        e1_dot = zd_dot-dz
        s1 = e1_dot + lamda[0] * e1
   
        Fx = m * (-K[0] * (x - xd) - K[1] * (dx-xd_dot) + xd_ddot)
        Fy = m * (-K[2] * (y - yd) - K[3] * (dy-yd_dot) + yd_ddot)
        
        u1 = (m * (zd_ddot+g+k[0] * self.signum(s1)+lamda[0] * e1_dot))/(cos(phi) * cos(theta))

        Fp = Fx/u1
        Ft = Fy/u1

        if abs(Fp)>0.9:
            Fp = 0.9* Fp/abs(Fp)
        if abs(Ft)>0.9:
            Ft = 0.9* Ft/abs(Ft)

        theta_desired = asin(Fp)   
        phi_desired = asin(-Ft)

        s2 = lamda[1] * (phi-phi_desired)+(phi_dot)
        s3 = lamda[2] * (theta-theta_desired)+(theta_dot)
        s4 = psi_dot+lamda[3] * psi

        u2 = -theta_dot * psi_dot * (Iy-Iz)+Ip * self.omega * theta_dot-(Ip * abs(theta_dot) * rho+k[1] * Ix) * self.signum(s2)-Ix * lamda[1] * phi_dot
        u3 = -phi_dot * psi_dot * (Iz-Ix)-Ip * self.omega * phi_dot-(Ip * abs(phi_dot) * rho+k[2] * Iy) * self.signum(s3)-Iy * lamda[2] * theta_dot
        u4 = -phi_dot * theta_dot * (Ix-Iy)-k[3] * Iz * self.signum(s4)-lamda[3] * psi_dot * Iz
                
        m1 = 1/(4 * KF)
        m2 = sqrt(2) * m1/l
        m3 = m1/KM

        u = np.array([[u1],[u2],[u3],[u4]])
        mat = np.array([[m1,-m2, -m2,-m3],
                        [m1,-m2,m2,m3],
                        [m1,m2,m2,-m3],
                        [m1,m2,-m2,m3]])
        
        w = np.dot(mat,u)

        motor_vel = np.zeros([4,1])
        motor_vel[0,0] = sqrt(w[0,0])
        motor_vel[1,0] = sqrt(w[1,0])
        motor_vel[2,0] = sqrt(w[2,0])
        motor_vel[3,0] = sqrt(w[3,0])
        self.omega = motor_vel[0,0]-motor_vel[1,0]+ motor_vel[2,0]- motor_vel[3,0]


        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
        self.motor_speed_pub.publish(motor_speed)

    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0]) * np.tan(rpy[1]), np.cos(rpy[0]) * np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])],[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
        rpy = np.expand_dims(rpy, axis = 1)
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
    def save_data(self):
        with open("/home/blacksnow/rbe502_project/src/project/log.pkl","wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)
if __name__  == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")

        

