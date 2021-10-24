#! /usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray 
from sympy import *
import math

class shit():
    def __init__(self):
        self.pos_pub = rospy.Publisher('/probot_anno/arm_vel_controller/command', Float64MultiArray, queue_size = 1)
        self.maxomega = np.array([[0.6],[0.6],[0.6],[0.6],[0.6],[0.6]])
        self.maxacc = np.array([[0.25], [0.25], [0.25], [0.25], [0.25], [0.25]])
        self.t_temp = 10
        self.ratio = np.array([[30],[205 / 3],[50],[125 / 2],[125 / 2],[200 / 9]])
        
    def trajectory_planning(self, theta0, theta1):
        velocity = []
        t_gap = np.arange(0, self.t_temp, 0.02)
        for i in range(6):
            result = self.PolySolve(theta1[i, 0], theta0[i, 0])
            velocity.append(self.Single_Cal(t_gap, result))
            pass
        velocity = np.array(velocity)
        return velocity
    
    def Single_Cal(self, t_gap, coeff):
        result = []
        for i in t_gap:
            T = np.array([[1, i, i ** 2, i ** 3, i ** 4, i ** 5]])
            result.append(np.dot(T, coeff)[0, 0])
        return result

    def PolySolve(self, theta2, theta1):
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
                        a1 + 2 * t_f * a2 + 3 * t_f ** 2 * a3 + 4 * t_f ** 3 * a4 + 5 * t_f ** 4 * a5,
                        2 * a2 + 6 * t_f * a3 + 12 * t_f ** 2 * a4 + 20 * t_f ** 3 * a5],
                       [a0, a1, a2, a3, a4, a5])
        result = np.double(np.array([[result[a0]], [result[a1]], [result[a2]], [result[a3]], [result[a4]], [result[a5]]]))
        result = np.array([[0], [1], [2], [3], [4], [5]]) * result
        return result
    
    def Controller(self, theta0, theta1, rate):
        velocity = self.trajectory_planning(theta0, theta1)
        size = velocity.shape[1]
        for i in range(size):
            b = np.array([[velocity[0, i], velocity[1, i], velocity[2, i], velocity[3, i], velocity[4, i], velocity[5, i]]])
            c = self.ratio.reshape([1, 6])
            # b = b * c
            d = []
            for i in range(6):
                d.append(b[0, i] / 5)
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
        print('hah')

def main():
    rospy.init_node('fuck')
    a = shit()
    # a.trajectory_planning([])
    rate = rospy.Rate(50)
    while true:
        a.Controller(np.array([[0], [0], [0], [0], [0], [0]]), np.array([[-0.7086], [-1.4651], [-0.0049], [0], [1.47], [0]]), rate)
        a.Controller(np.array([[-0.7086], [-1.4651], [-0.0049], [0], [1.47], [0]]), np.array([[0], [0], [0], [0], [0], [0]]), rate)
    # while true:
    #     for i in np.arange(0, 0.1, 0.01):
    #         a.publishResult([-i, 0, 0, 0, 0, 0], rate)
    #     for i in np.arange(0, 0.1, 0.01):
    #         a.publishResult([-0.1 + i, 0, 0, 0, 0, 0], rate)
    #     for i in np.arange(0, 0.1, 0.01):
    #         a.publishResult([i, 0, 0, 0, 0, 0], rate)
    #     for i in np.arange(0, 0.1, 0.01):
    #         a.publishResult([0.1 - i, 0, 0, 0, 0, 0], rate)
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
    




