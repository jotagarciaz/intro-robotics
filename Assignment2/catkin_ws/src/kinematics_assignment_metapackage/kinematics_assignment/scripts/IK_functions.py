#! /usr/bin/env python3

"""
    # {student full name}
    # {student email}
"""

import math as m
import numpy as np
from functools import reduce


def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    l0=0.07
    l1=0.3
    l2=0.35    

    a=((x-l0)**2+y**2-l1**2-l2**2)/(2*l1*l2)
    b=m.sqrt(1-a**2)
    
    q1=m.atan2(y,x-l0)-m.atan2(l2*b,l1+l2*a)
    q2=m.atan2(b,a) 
    q3=z
    
    q=[q1,q2,q3]

    """
    Fill in your IK solution here and return the three joint values in q
    """

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
