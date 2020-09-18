#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Joaquin Garcia Benitez}
# {19950502-T759}
# {jogb@kth.se}

from dubins import *
import math
from numpy import inf

def solution(car):

  '''
  Your code below
  '''

  # initial state
  x, y = car.x0, car.y0
  theta = 0

  # arbitrary control
  phi = 0.2

  # compute next state after 0.01 seconds
  xn, yn, thetan = step(car, x, y, theta, phi)

  # assemble path
  controls, times = [phi], [0, 0.01]

  controls, times = Breadth_First_Search(car, [], [])

  '''
  Your code above
  '''

  return controls, times

def Breadth_First_Search(car, path, visited):
    threshold = 0.2
    queue = [[car.x0,car.y0,0,[],[0],math.sqrt((car.x0 - car.xt)**2 + (car.y0 - car.yt)**2)]]
    queue1 = []
    while len(queue) > 0:
        x,y,theta,controls,times,_ = queue.pop(0)
        if math.sqrt((x - car.xt)**2 + (y - car.yt)**2) <= threshold:
            return controls, times
        visited.append([round(x,1), round(y,1)])
        for phi in [-math.pi/4, 0, math.pi/4]:
            useable, x1, y1, theta1, controls1, times1, cost = positions(x,y,phi,theta,car,replace_array(controls),replace_array(times),threshold)
            if useable and not [round(x1,1), round(y1,1), round(theta1,1)] in queue1:
                path.append(phi)
                queue1.append([round(x1,1), round(y1,1), round(theta1,1)])
                queue.append([x1, y1, theta1, controls1, times1, cost])
            queue.sort(key=lambda x: x[5])
    return [],[0]

def replace_array(arr):
    new_arr = []
    for x in arr:
        new_arr.append(x)
    return new_arr



def collisions(x,y,car):
    for obs in car.obs:
        d = math.sqrt((x - obs[0])**2 + (y - obs[1])**2)
        if d <= obs[2] + 0.1:
            return True
    return False

def outbounds(x,y,car):
    if (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub ):
        return False
    return True
   
def positions(x,y,phi,theta,car,controls,times,threshold):
    cost = 0
    for i in range(100 if phi == 0 else 157):
        x, y, theta = step(car, x, y, theta, phi)
        while theta >= math.pi:
            theta -= 2*math.pi
        while theta <= -2*math.pi:
            theta += math.pi  
        controls.append(phi)
        times.append(times[-1] + cos(theta))
        if collisions(x,y,car) or outbounds(x,y,car):
            return False, 0, 0, 0, controls, times, inf
        if math.sqrt((x - car.xt)**2 + (y - car.yt)**2) <= threshold:
            return True, x, y, theta, controls, times, 0
    cost = math.sqrt((x - car.xt)**2 + (y - car.yt)**2)
    return True, x, y, theta, controls, times, cost
