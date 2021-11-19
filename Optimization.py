#! /usr/bin/env python
from sympy import *
import numpy as np
import math
import pdb

class Optimize():
    def __init__(self, frequency):
        self.maxomega = np.array([[0.75],[0.65],[0.75],[0.92],[0.92],[0.92]])
        self.maxacc = np.array([[0.34], [0.25], [0.34], [0.43], [0.43], [0.43]])
        self.frequency = frequency
    def time_optimization(self, theta0, theta1):
        t_temp = 4
        flag = 1
        while flag: 
            velocity = []
            t_gap = np.arange(0, t_temp, 1.0 / self.frequency)
            for i in range(6):
                result = self.PolySolve(theta1[i, 0], theta0[i, 0], t_temp)
                velocity, accelerate = self.Single_Cal(t_gap, result)
                vel_max = max(velocity)
                acc_max = max(accelerate)
                if vel_max > self.maxomega[i, 0] or acc_max > self.maxacc[i, 0]:
                    flag = 0
                    break
            # pdb.set_trace()
            if flag:
                t_temp = t_temp - 0.1
        return t_temp + 0.1
    
    def Single_Cal(self, t_gap, coeff):
        vel_result = []
        acc_result = []
        for i in t_gap:
            T = np.array([[0, 1, 2 * i , 3 * i ** 2, 4 * i ** 3, 5 * i ** 4]])
            vel_result.append(np.dot(T, coeff)[0, 0])
            T = np.array([[0, 0, 2 , 6 * i, 12 * i ** 2, 20 * i ** 3]])
            acc_result.append(np.dot(T, coeff)[0, 0])
        return vel_result, acc_result

    def PolySolve(self, theta2, theta1, t_temp):
        a0 = symbols('a0')
        a1 = symbols('a1')
        a2 = symbols('a2')
        a3 = symbols('a3')
        a4 = symbols('a4')
        a5 = symbols('a5')
        t_f = t_temp
        result = solve([a0 - theta1, 
                        a1 - 0, 
                        2 * a2 - 0, 
                        a0 + a1 * t_f + a2 * t_f ** 2 + a3 * t_f ** 3 + a4 * t_f ** 4 + a5 * t_f ** 5 - theta2,
                        a1 + 2 * t_f * a2 + 3 * t_f ** 2 * a3 + 4 * t_f ** 3 * a4 + 5 * t_f ** 4 * a5,
                        2 * a2 + 6 * t_f * a3 + 12 * t_f ** 2 * a4 + 20 * t_f ** 3 * a5],
                       [a0, a1, a2, a3, a4, a5])
        result = np.double(np.array([[result[a0]], [result[a1]], [result[a2]], [result[a3]], [result[a4]], [result[a5]]]))
        # result = np.array([[0], [1], [2], [3], [4], [5]]) * result
        # print(result)
        return result    