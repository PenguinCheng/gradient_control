#!/usr/bin/python
"""
Adaptive control algorithm class for leaders, to keep two leaders matianing a constant distance 
@author: Peng Cheng
@date: 2015/9/3 18:52 
		
"""
import time
import numpy as np
#import matplotlib.pyplot as plt

current_milli_time = lambda: int(time.time() * 1000)

class leaderControl(object):

	def __init__(self, vx0, vy0, initial_d12, initial_x, initial_y):
		self.vx0 = vx0
		self.vy0 = vy0
		self.vx = 0.0
		self.vy = 0.0
		self.x = initial_x
		self.y = initial_y
		self.d12 = initial_d12  

	# @param: x1-the x position of the object itself
	#         x2-the x position of another leader
	#         y1-the y position of the object itself
	#         y2-the y position of another leader
	def controller(self, x1, x2, y1, y2):
		currentDisPow = pow(x1-x2, 2) + pow(y1-y2, 2)
		self.vx = self.vx0 + (x2 - x1) * (currentDisPow - pow(self.d12, 2)) * 0.01  #scale factor 0.01 need to be considered carefully
		self.vy = self.vy0 + (y2 - y1) * (currentDisPow - pow(self.d12, 2)) * 0.01

	# It's better to output control in main function
	'''
	def run(self, vx, vy):
		send_ned_velocity(vx, vy, 0)  # vz = 0.0
	'''
'''
if __name__ == "__main__":
	leader1 = leaderControl(0.0, 0.0, 2.0, 0.0, 8.0)
	leader2 = leaderControl(0.0, 0.0, 2.0, 10.0, 0.0)
	delt_T = 0.5  #s
	lastRecord = current_milli_time()
	cnt = 0
	leader1X=[]
	leader1Y=[]
	leader2X=[]
	leader2Y=[]
	while cnt<50:
		if current_milli_time() - lastRecord >= 500:  #ms
			print "[%d] current time is: " % cnt + str(current_milli_time())
			lastRecord = current_milli_time()
			leader1.controller(leader1.x, leader2.x, leader1.y, leader2.y)
			leader2.controller(leader2.x, leader1.x, leader2.y, leader1.y)

			leader1.x += leader1.vx * delt_T
			leader1.y += leader1.vy * delt_T
			leader2.x += leader2.vx * delt_T
			leader2.y += leader2.vy * delt_T

			cnt += 1
			leader1X.append(leader1.x)
			leader1Y.append(leader1.y)
			leader2X.append(leader2.x)
			leader2Y.append(leader2.y)
			print "x1:" + str(leader1.x)
			print "y1:" + str(leader1.y)
			print "x2:" + str(leader2.x)
			print "y2:" + str(leader2.y)
		else:
			pass
		
	plt.figure(1)
	plt.plot(leader1X,leader1Y)
	plt.plot(leader2X,leader2Y)
	plt.show()
'''
