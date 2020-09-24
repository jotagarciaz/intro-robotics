#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {student full name}
# {student id}
# {student email}

from dubins import Car
import math as m
import numpy as np

def solution(car):

    ''' <<< write your code below >>> '''
    controls=[0]
    times=[0,1]

    print(car.xt)
    print(car.x0)

    for i in range(780):
        controls.append(i/1000)
        times.append(i+2)

    ''' <<< write your code above >>> '''

    return controls, times
