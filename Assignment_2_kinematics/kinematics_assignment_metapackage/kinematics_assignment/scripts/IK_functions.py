#! /usr/bin/env python3

"""
    # {Joaquin Garcia Benitez}
    # {jogb@kth.se}
"""

import math

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """
    """
    Using J.J. Craig notation from page 110
    """
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

   

    c2 = ((x-l0)**2+y**2-l1**2-l2**2)/(2*l1*l2)
    s2 = math.sqrt(1-c2**2)
    
    k1 = l1+l2*c2
    k2 = l2*s2

    q[0] = math.atan2(y, x-l0) - math.atan2(k2, k1)
    q[1] = math.atan2(s2, c2)
    q[2] = z

    print(q)
    return q


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions  # it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
