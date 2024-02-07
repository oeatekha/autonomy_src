#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from scipy import linalg
import scipy as sp
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import math
import matplotlib.pyplot as plt
import numpy as np
import tf_transformations

class sendvelocity(Node):
    def __init__(self,name):
		# initialize base class (must happen before everything else)
        super().__init__(name)  # initialize base class

        # physic constrain for the locobot
        self.L = 0.1

        # different Kp values to test
        self.kp_values = [0.5, 10, 1.5]  
        self.current_kp = self.kp_values[1] #initialize the first one

        # prepare for the plots
        self.data_collections = {kp: {'time': [], 'x': [], 'y': [], 'rx': [], 'ry': []} for kp in self.kp_values}
        
        # initialize time
        # self.start_time = self.get_clock().now().nanoseconds * 1e-9   
        self.start_time = self.get_clock().now()

        # create subscription with
		# self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.subscription = self.create_subscription(Odometry,'/odom', self.mobile_control_callback,10)

		# create publisher with
		# self.create_publisher(<msg type>, <topic>, <qos>)
        self.mobile_base_vel_publisher = self.create_publisher(Twist, "/locobot/diffdrive_controller/cmd_vel_unstamped", 10)
    
    def mobile_control_callback(self,data):
        control_in, t = self.compute_error_vector(data)
        v, w = self.control_input_calculation(control_in)
        self.send_control_command(v, w, t)
        # Update data collection for the current Kp value
        #seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        # t = seconds + nanoseconds * 1e-9 - self.start_time
        
        # get the current position of robot
        # point_x = data.pose.pose.position.x
        # point_y = data.pose.pose.position.y
        
        # # # rx, ry = self.target_position(t)
        # # # self.data_collections[self.current_kp]['time'].append(t)

        # # # prepare for plots
        # self.data_collections[self.current_kp]['x'].append(point_x)
        # self.data_collections[self.current_kp]['y'].append(point_y)
        # self.data_collections[self.current_kp]['rx'].append(rx)
        # self.data_collections[self.current_kp]['ry'].append(ry)

    def compute_error_vector(self, data):
        # fint t
        t_now = self.get_clock().now()
        t = (t_now - self.start_time).nanoseconds / 1e9

        # qw = data.pose.pose.orientation.w
        # qx = data.pose.pose.orientation.x
        # qy = data.pose.pose.orientation.y
        # qz = data.pose.pose.orientation.z
        # R11 = qw**2 + qx**2 - qy**2 -qz**2
        # R12 = 2*qx*qz - 2*qw*qz
        # R21 = 2*qx*qz + 2*qw*qz
        # R22 = qw**2 - qx**2 + qy**2 -qz**2
                        
        # Rotation_mat = np.matrix([[R11,R12],[R21,R22]])
        # axis_angle_mat = sp.linalg.logm(Rotation_mat)
        # theta = axis_angle_mat[1,0]

        # find point P
        q = data.pose.pose.orientation
        _,_,theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        point_x = data.pose.pose.position.x 
        point_y = data.pose.pose.position.y 
        point_P = Point()
        point_P.x = point_x + self.L * math.cos(theta)
        point_P.y = point_y + self.L * math.sin(theta)

        self.data_collections[self.current_kp]['x'].append(point_P.x)
        self.data_collections[self.current_kp]['y'].append(point_P.y)
        #seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        # Convert nanoseconds to seconds and add to the seconds
        #t = seconds + nanoseconds * 1e-9 - self.start_time

        # find error_vect
        rx, ry = self.target_position(t)
        self.data_collections[self.current_kp]['time'].append(t)
        self.data_collections[self.current_kp]['rx'].append(rx)
        self.data_collections[self.current_kp]['ry'].append(ry)
        err_x = rx - point_P.x 
        err_y = ry - point_P.y
        error_vect = np.matrix([[err_x],[err_y]])
        M = np.matrix([[np.cos(theta), -np.sin(theta) * self.L],
                            [np.sin(theta), np.cos(theta)  * self.L]])
        M_inv = np.linalg.inv(M)
        k_mat = self.current_kp * np.eye(2)
        control_input = M_inv @ k_mat @ error_vect
        #error_pos = np.linalg.norm(error_vect)
        # rtheta = np.arctan2(err_y,err_x)
        # error_angle = rtheta - theta
        # error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi #normalize
        return control_input,t
        
    def control_input_calculation(self,control_input):
        # Kp_mat = self.current_kp*np.eye(2)
        # point_p_error_signal = Kp_mat*error_vect
        # non_holonomic_mat = np.matrix([[np.cos(theta), -np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        # control_input = np.linalg.inv(non_holonomic_mat)*point_p_error_signal
        # v = float(control_input.item(0))
        # w = float(control_input.item(1))
         # Control inputs
        v = float(control_input.item(0))  # linear velocity
        w = float(control_input.item(1)) # angular velocity
        return v,w

    def send_control_command(self, v, w, t):
        if t <= 20:
            control_msg = Twist()
            control_msg.linear.x = v
            control_msg.angular.z = w
            self.mobile_base_vel_publisher.publish(control_msg)
        else:
            control_msg = Twist()
            control_msg.linear.x = 0.0
            control_msg.angular.z = 0.0
            self.mobile_base_vel_publisher.publish(control_msg)
            print("Finished")

    def target_position(self,t):
        # if t <= 20:  # Duration of two revolutions
        #     rx = 0.5 * math.cos(t * 2 * math.pi / 10)
        #     ry = 0.5 * math.sin(t * 2 * math.pi / 10)
        # else:
        #     rx, ry = 0, 0  # Stop motion after 20 seconds
        
        rx = 0.5 * math.cos(t * 2 * math.pi / 10)
        ry = 0.5 * math.sin(t * 2 * math.pi / 10)
        return rx, ry

def plot_data(data_collections):
    colors = ['r', 'g', 'b']  # Different colors for each Kp plot

    plt.figure()
    for kp, color in zip(data_collections.keys(), colors):
        plt.plot(data_collections[kp]['time'], data_collections[kp]['x'], label=f'Robot X (Kp={kp})', color=color)
        plt.plot(data_collections[kp]['time'], data_collections[kp]['rx'], label=f'Reference X (Kp={kp})', color=color, linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position')
    plt.title('X Position over Time for Different Kp Values')
    plt.legend()
    plt.show()

    plt.figure()
    for kp, color in zip(data_collections.keys(), colors):
        plt.plot(data_collections[kp]['time'], data_collections[kp]['y'], label=f'Robot Y (Kp={kp})', color=color)
        plt.plot(data_collections[kp]['time'], data_collections[kp]['ry'], label=f'Reference Y (Kp={kp})', color=color, linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position')
    plt.title('Y Position over Time for Different Kp Values')
    plt.legend()
    plt.show()

def main():
    rclpy.init()
    node = sendvelocity('send_velocity_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        plot_data(node.data_collections)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
