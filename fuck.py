#! /usr/bin/env python
import numpy as np
import rospy
from rospy.names import valid_name_validator_resolved
from std_msgs.msg import Float64MultiArray 
from sympy import *
import math
import Optimization
import pdb

class shit():
    def __init__(self):
        self.pos_pub = rospy.Publisher('/probot_anno/arm_vel_controller/command', Float64MultiArray, queue_size = 1)
        self.maxomega = np.array([[0.78],[0.69],[0.78],[0.95],[0.95],[0.95]])
        self.maxacc = np.array([[0.34], [0.25], [0.34], [0.43], [0.43], [0.43]])
        self.t_f = 7
        self.t1 = 2
        self.t2 = 4
        self.t_temp = 3
        self.ratio = np.array([[30],[205 / 3],[50],[125 / 2],[125 / 2],[200 / 9]])
        self.start_ratio = 0.2
        self.acc_start_ratio = np.array([[-0.4], [0.8], [0.4], [0], [0], [0]])
        self.vel_start_ratio = np.array([[0], [0.1], [0], [0], [0], [0]])
        self.zero_point = np.array([[0], [0], [0], [0], [0], [0]])
        self.A_point = np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]])
        self.B_point = np.array([[0], [-0.4722], [-0.4642], [0], [-0.9365 + 0.9365 + 1.47], [0]])
        self.C_point = np.array([[-0.7086], [-1.4651], [-0.0049], [0], [1.47], [0]])
        
    def trajectory_planning(self, theta0, theta1):
        velocity = []
        t_gap = np.arange(0, self.t_temp, 0.01)
        for i in range(6):
            result = self.PolySolve(theta1[i, 0], theta0[i, 0], i)
            velocity.append(self.Single_Cal(t_gap, result))
            # pdb.set_trace()
            pass
        velocity = np.array(velocity)
        return velocity
    
    def Single_Cal(self, t_gap, coeff):
        result = []
        for i in t_gap:
            T = np.array([[0, 1, i , i ** 2, i ** 3, i ** 4]])
            result.append(np.dot(T, coeff)[0, 0])
        return result

    def PolySolve(self, theta2, theta1, i):
        a0 = symbols('a0')
        a1 = symbols('a1')
        a2 = symbols('a2')
        a3 = symbols('a3')
        a4 = symbols('a4')
        a5 = symbols('a5')
        t_f = self.t_temp
        result = solve([a0 - theta1, 
                        a1 - 0, 
                        2 * a2 - 0, 
                        a0 + a1 * t_f + a2 * t_f ** 2 + a3 * t_f ** 3 + a4 * t_f ** 4 + a5 * t_f ** 5 - theta2,
                        a1 + 2 * t_f * a2 + 3 * t_f ** 2 * a3 + 4 * t_f ** 3 * a4 + 5 * t_f ** 4 * a5 - self.maxomega[i, 0] * self.vel_start_ratio[i, 0],
                        2 * a2 + 6 * t_f * a3 + 12 * t_f ** 2 * a4 + 20 * t_f ** 3 * a5 - self.maxacc[i, 0] * self.acc_start_ratio[i, 0]],
                       [a0, a1, a2, a3, a4, a5])
        result = np.double(np.array([[result[a0]], [result[a1]], [result[a2]], [result[a3]], [result[a4]], [result[a5]]]))
        result = np.array([[0], [1], [2], [3], [4], [5]]) * result
        # print(result)
        return result
    
    def Controller(self, theta0, theta1):
        velocity = self.trajectory_planning(theta0, theta1)
        size = velocity.shape[1]
        return velocity, size
    
    def Publish(self, velocity, size, rate):
        for i in range(size):
            b = np.array([[velocity[0, i], velocity[1, i], velocity[2, i], velocity[3, i], velocity[4, i], velocity[5, i]]])
            # c = self.ratio.reshape([1, 6])
            d = []
            for i in range(6):
                d.append(b[0, i])
            self.publishResult(d, rate)
    
    def publishResult(self, data, rate):
        a = Float64MultiArray()
        #a.data = np.float64(np.array([np.pi / 6, np.pi / 6, np.pi / 6, np.pi / 6, np.pi / 2 + np.pi / 6, np.pi / 6]))
        a.data = np.float64(data)
	    #a.data = np.float64(np.array([-np.pi / 4, np.pi / 3, -np.pi / 4, np.pi / 3, np.pi / 2 - np.pi/4, np.pi / 3]))
        # rate = rospy.Rate(10) # 10hz
        count = 0
        self.pos_pub.publish(a)
        rate.sleep()
    
    def Trible_trajectory_planning(self, theta0, theta1, theta2):
        velocity = []
        t_gap = np.arange(0, self.t_f, 0.01)
        for i in range(6):
            result = self.Trible_PolySolve(theta2[i, 0] ,theta1[i, 0], theta0[i, 0], i)
            velocity.append(self.Trible_Single_Cal(t_gap, result))
            # pdb.set_trace()
            pass
        velocity = np.array(velocity)
        return velocity
    
    def Trible_Single_Cal(self, t_gap, coeff):
        result = []
        for i in t_gap:
            T = np.array([[0, 1, i , i ** 2, i ** 3, i ** 4, i ** 5, i ** 6]])
            result.append(np.dot(T, coeff)[0, 0])
        return result
    
    def Trible_PolySolve(self, theta3, theta2, theta1, i):
        a0 = symbols('a0')
        a1 = symbols('a1')
        a2 = symbols('a2')
        a3 = symbols('a3')
        a4 = symbols('a4')
        a5 = symbols('a5')
        a6 = symbols('a6')
        a7 = symbols('a7')
        t_1 = self.t1
        t_2 = self.t2
        t_f = self.t_f
        result = solve([a0 - theta1, 
                        a1 - self.maxomega[i, 0] * self.vel_start_ratio[i, 0], 
                        2 * a2 - self.maxacc[i, 0] * self.acc_start_ratio[i, 0], 
                        a0 + a1 * t_1 + a2 * t_1 ** 2 + a3 * t_1 ** 3 + a4 * t_1 ** 4 + a5 * t_1 ** 5 + a6 * t_1 ** 6 + a7 * t_1 ** 7 - theta2,
                        a0 + a1 * t_2 + a2 * t_2 ** 2 + a3 * t_2 ** 3 + a4 * t_2 ** 4 + a5 * t_2 ** 5 + a6 * t_2 ** 6 + a7 * t_2 ** 7 - theta3,
                        a0 + a1 * t_f + a2 * t_f ** 2 + a3 * t_f ** 3 + a4 * t_f ** 4 + a5 * t_f ** 5 + a6 * t_f ** 6 + a7 * t_f ** 7 - theta1,
                        a1 + 2 * t_f * a2 + 3 * t_f ** 2 * a3 + 4 * t_f ** 3 * a4 + 5 * t_f ** 4 * a5 + 6 * t_f ** 5 * a6 + 7 * t_f ** 6 * a7 - self.maxomega[i, 0] * self.vel_start_ratio[i, 0],
                        2 * a2 + 6 * t_f * a3 + 12 * t_f ** 2 * a4 + 20 * t_f ** 3 * a5 + 30 * t_f ** 4 * a6 + 42 * t_f ** 5 * a7 - self.maxacc[i, 0] * self.acc_start_ratio[i, 0]],
                       [a0, a1, a2, a3, a4, a5, a6, a7])
        result = np.double(np.array([[result[a0]], [result[a1]], [result[a2]], [result[a3]], [result[a4]], [result[a5]], [result[a6]], [result[a7]]]))
        result = np.array([[0], [1], [2], [3], [4], [5], [6], [7]]) * result
        # print(result)
        return result

    def Trible_Controller(self, theta0, theta1, theta2):
        velocity = self.Trible_trajectory_planning(theta0, theta1, theta2)
        size = velocity.shape[1]
        return velocity, size

def main():
    rospy.init_node('fuck')
    a = shit()
    b = Optimization.Optimize(20)
    # a.trajectory_planning([])
    rate = rospy.Rate(100)
    # t = []
    # t.append(b.time_optimization(np.array([[0], [0], [0], [0], [0], [0]]), np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]])))
    # t.append(b.time_optimization(np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]]), np.array([[0], [-0.4722], [-0.4642], [0], [-0.9365], [0]])))
    # t.append(b.time_optimization(np.array([[0], [-0.4722], [-0.4642], [0], [-0.9365], [0]]), np.array([[-0.7086], [-1.4651], [-0.0049], [0], [1.47], [0]])))
    # pdb.set_trace()
    # a.Controller(np.array([[0], [0], [0], [0], [0], [0]]), np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]]), rate)
    # while true:
    #     a.t_temp = b.time_optimization(np.array([[0], [0], [0], [0], [0], [0]]), np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]]))
    #     pdb.set_trace()
    #     a.Controller(np.array([[0], [0], [0], [0], [0], [0]]), np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]]), rate)
    #     a.t_temp = b.time_optimization(np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]]), np.array([[0], [0], [0], [0], [0], [0]]))
    #     pdb.set_trace()
    #     a.Controller(np.array([[0.5233], [-0.8715], [-0.6732], [0], [1.5447], [0]]), np.array([[0], [0], [0], [0], [0], [0]]), rate)
    velocity, size = a.Controller(a.zero_point, a.A_point)
    Trible_velocity, Trible_size = a.Trible_Controller(a.A_point, a.B_point, a.C_point)
    # print (Trible_velocity)
    # print ('hhh')
    # print (velocity)
    # pdb.set_trace()
    a.Publish(velocity, size, rate)
    while True:
        a.Publish(Trible_velocity, Trible_size, rate)
    rospy.spin()

if __name__ == '__main__':
    main()
    # a = []
    # a.append(10)
    # a.append(10)
    # a.append(10)
    # a = np.array([a])
    # b = np.array([[1, 2, 3, 4], [5, 6, 7, 8]])
    # print(b.T)
