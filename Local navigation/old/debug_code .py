# -*- coding: utf-8 -*-
"""
Created on Mon Nov 29 01:09:47 2021

@author: eliot
"""

%%run_python #%%transpile_to_aseba # Change to show the code generated in Aseba

import utility as util
import constant
# This program implements obstacle avoidance using an ANN
# The inputs x1-7 are taken from the proximity sensors and scaled
# The inputs are multiplied by the weights 
#   and used to set the motor powers
# The center button starts and stops the robot

#sates:
#0 the robot stops 
#1 the robot arrives at the obstacle and choses which side to go around
#2 the robot goes forward until the sensor chose (1 left or 5 right) doesnt see anythin
#3 the robot goes forward 1 thymio length
#4 the robot rotates until its sensor (1 or 5) sees the obstacle
#5 the robot follows the obstaclke on its length until its sees nothin
#6 the robot goes forward to completely avoid the obstacle 

# State for start and stop, why is this instruction here? (hint: look at the aseba transpile code)
adjust = 2000
close_thresh = 20
time = 0
state = 1
y = [0,0] 
timer_period[0] = 10
time_one_thym_dist = 500
side = constant.NONE
counter_state = constant.NONE
    
@onevent
def button_center():
    global state
    if button_center == 1:
        state = 1 if state==0 else 0

@onevent
def prox():
    global prox_horizontal, motor_left_target, motor_right_target, button_center, state, y 
    #asymetric w_l[2] != w_r[2] so the robot choses one side if it arrives straight on the obstacle
    w_l = [20,  10, -15, -10, -20, 6, 0]
    w_r = [-20, -10, -10,  10,  20, 0, 6]

    # Scale factors for sensors and constant factor
    sensor_scale = 400
    constant_scale = 20
    
    x = [0,0,0,0,0,0,0]
    
### STATE 0 the robot stops###    
    if state == 0:
        y = [0,0]
### STATE 1  the robot arrives at the obstacle and choses which side to go around###         
    if state == 1:
        # Memory
        x[5] = y[0]//10
        x[6] = y[1]//10
        
        x[0]=0
        x[4]=0
        
        for i in range(1,4):
            # Get and scale inputs
            x[i] = prox_horizontal[i] // sensor_scale
            
        y = [80,80]    
        
        for i in range(len(x)):    
            # Compute outputs of neurons and set motor powers
            y[0] = y[0] + x[i] * w_l[i]
            y[1] = y[1] + x[i] * w_r[i]
            
        if util.see_nothing(x[1:4],close_thresh) == True :
            state = 2
### STATE 2 the robot goes forward until the sensor chose (1 left or 5 right) doesnt see anythin###             
    if state == 2:
        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale
        if left_prox > close_thresh: 
            y = [80,80]
            side = constant.RIGHT
        if right_prox > close_thresh:
            y = [80,80]
            side = constant.LEFT
        if left_prox < close_thresh and right_prox < close_thresh:
            state = 3
            
### STATE 4 the robot rotates until its sensor (1 or 5) sees the obstacle###             
    if state == 4: 
        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale
        if side == constant.LEFT:
            while right_prox == 0 :
                y = [80,0]
            if right_prox > 0:
                state = 5
        if side == constant.RIGHT:
            while left_prox == 0 :
                y = [0,80]
            if left_prox > 0:
                state = 5
                
### STATE 5 the robot follows the obstaclke on its length until its sees nothin###             
    if state == 5:

        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale
        left_prox_adjust = left_prox -adjust//sensor_scale
        right_prox_adjust = right_prox -adjust//sensor_scale
        if side == constant.LEFT:
            y = [80,80+right_prox_adjust]
        if side == constant.RIGHT:
            y = [80+left_prox_adjust,80]
        if left_prox < close_thresh and right_prox < close_thresh:
            state = 6

   
    # Set motor powers
    motor_left_target = y[0]
    motor_right_target = y[1]
    

#will be used to go forward from 1 thymio length
@onevent
def timer0():
    global state,counter_state,motor_left_target, motor_right_target
    if counter_state == constant.RESET:
        time = 0
### STATE 3 the robot goes forward 1 thymio length and 6 the robot goes forward to completely avoid the obstacle###             
    if state == 3 or state == 6:
        time += 1         
        if time <= time_one_thym_dist:
            motor_left_target = 80
            motor_right_target = 80
        else:
            counter_state == constant.RESET
            if state == 3:
                state = 4
            if state == 6: 
                state = 0 #stops the robot and terminates the avoidance 
            