import math
import time
import tf
import numpy as np
import csv
from protocol import Protocol 

global_height = 0
target = 30

# class heightcontroller:

#     def __init__(self, kp, kd, ki):
#         self.talker = Protocol( IP, PORT)
#         self.kp = kp
#         self.kd = kd
#         self.ki = ki
#         self.vel = 0
#         self.breaker = True
#         self.prev_time = 0
#         self.prev_error = 0
#         self.e_i = 0
#         self.speed = 0.0001
#         self.error_tol = 0.01
#         self.height = 0

#     def calc_error(self,error): # Calculates the error, its `derivative' and `integral' and their sum along with constants multiplied
#         curr_time = time.time()
#         dt = 0.0
#         if curr_time != 0.0:
#             dt = curr_time - self.prev_time
#         de = error - self.prev_error
        
#         e_p = self.kp * error
#         self.e_i += error * dt
#         e_d = 0
#         if dt > 0:
#             e_d = de/dt
#         self.prev_time = curr_time
#         self.prev_error = error
#         print("time", dt, de)
#         return e_p + (self.ki*self.e_i) + (self.kd*e_d)

#     def pos_change(self,targ_pos): # Corrects only position
#         # if not self.breaker:
#         errors = targ_pos - self.height
#         self.vel = self.calc_error(errors)
#         self.height = self.height + self.vel*self.speed
#         print("height:", self.height)

#         # if abs(errors) > abs(self.error_tol) :
#         #     #here we need to tell it to inc or dec its height
#         #     self.height =self.height + self.vel*self.speed #how does this sound for testing?
#         #     self.reach_pose = False
#         #     print("height:", self.height)
#         # else:
#         #     self.reach_pose = True
#         #     print ("reached destination coordinates")
#         #     print ("error:",np.linalg.norm(errors))

#     def autopilot(self, targ_pos):
#         # if self.height != targ_pos:
#         #     self.breaker = False
#         # while not self.breaker:
#         while 1:
#             self.pos_change(targ_pos)
#             # if self.reach_pose:
#             #     break
# start =time.time()
# hello = heightcontroller(0.5, 0.0005, 0.000000000000005)
# hello.autopilot(target)
# end=time.time()
# print(end-start)
class pidcontroller:
    # Defining the P,I,D control parameters
    def __init__(self, IP, PORT, targ, kp=[0.05,0.05,3], kd=[0.2,0.2,0.0],ki=[0.000,0.000,0.0]):
        self.talker =  Protocol(IP, PORT)
        self.kp = kp
        self.kd = kd
        self.ki = ki
        # self.breaker = True 
        self.prev_time = np.array([0.0,0.0,0.0])
        self.prev_error = np.array([0.0,0.0,0.0])
        self.e_i = np.array([0.0,0.0,0.0])
        self.vel = np.array([0.0,0.0,0.0])
        self.speed = 1 
        self.repeat_pilot = False
        self.single_point_error = 0.001
        self.multiple_point_error = 0.2 #0.1
        self.ang_error = 0.05 #0.02
        self.length = 1
        self.curr_pos=np.array([0.0,0.0,0.0])
        self.curr_ori=np.array([1500,1500,1500,900])
        self.equilibrium_pose = np.array([self.talker.EQUIILIBRIUM_ROLL, self.talker.EQUIILIBRIUM_PITCH, self.talker.EQUIILIBRIUM_YAW, 900]) #need to do something about thrust
            
    def calc_error(self,error,i): # Calculates the error, its `derivative' and `integral'
        curr_time = time.time() 
        dt = 0.0
        if self.curr_time != 0.0:
            dt = curr_time - self.prev_time
        de = error[i] - self.prev_error[i]
        e_p = self.kp[i] * error[i]
        self.e_i[i] += error[i] * dt
        e_d = 0
        if dt > 0:
            e_d = de/dt
        self.prev_time[i] = curr_time
        self.prev_error[i] = error[i]
        return e_p + (self.ki[i]*self.e_i[i]) + (self.kd[i]*e_d)

    def pos_change(self,targ_pos=np.array([0,0,0]),curr_pos = np.array([0,0,0]), curr_ori = np.array([1500, 1500, 1500, 900])):# Corrects only position
        # if not self.breaker:
        errors = targ_pos-curr_pos
        for i in range(len(errors)):
            self.vel[i] = self.calc_error(errors[i],i)
        if max(errors) > self.error_tol or min(errors) < -self.error_tol:
            for(i) in range(2): 
                curr_temp=self.equilibrium_pose[i]+self.speed*self.vel[i]
                if((curr_temp<self.talker.MAX_THRUST) and (curr_temp>self.talker.MIN_THRUST)):
                    curr_ori[i]=curr_temp
                else:
                    if(curr_temp>self.talker.MAX_THRUST):
                        curr_ori[i]=self.talker.MAX_THRUST
                    else:
                        curr_ori[i]=self.talker.MIN_THRUST
            
            self.talker.set_RPY_THR(curr_ori[0], curr_ori[1], curr_ori[2], curr_ori[3]+self.speed*self.vel[2])
            # Thrust will be changed by self.speed*self.vel[2]
            # Roll will be changed by self.speed*self.vel[0]
            # Pitch will be changed by self.speed*self.vel[1]
            self.reach_pose = False
            print()
        else:
            self.reach_pose = True
            print ("reached destination coordinates")
            print ("error:",np.linalg.norm(errors)) 

    def autopilot(self,targ_pos):
        # self.curr_pos and orientation needs to be read
        if  self.curr_pos != targ_pos:
            self.reach_pose = False
        while not self.reach_pose:
            self.pos_change(targ_pos,self.curr_pos,self.curr_ori)
            if self.reach_pose:
                break
            # self.curr_pos and orientation needs to be read from IMU          
