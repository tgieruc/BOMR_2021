# -*- coding: utf-8 -*-
"""
Created on Mon Nov 29 09:02:12 2021

@author: eliot
"""

### STATE 2 the robot goes forward until the sensor chose (1 left or 5 right) doesnt see anythin###             
    if state == 2:
        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale
        if left_prox > close_thresh: 
            y = [80,80]
            side = RIGHT
        if right_prox > close_thresh:
            y = [80,80]
            side = LEFT
        if left_prox < close_thresh and right_prox < close_thresh:
            state = 3
            
### STATE 4 the robot rotates until its sensor (1 or 5) sees the obstacle###             
    if state == 4: 
        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale
        if side == LEFT:
            while right_prox == 0 :
                y = [80,0]
            if right_prox > 0:
                state = 5
        if side == RIGHT:
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
        if side == LEFT:
            y = [80,80+right_prox_adjust]
        if side == RIGHT:
            y = [80+left_prox_adjust,80]
        if left_prox < close_thresh and right_prox < close_thresh:
            state = 6

   
    # Set motor powers
    motor_left_target = y[0]
    motor_right_target = y[1]
    

#will be used to go forward from 1 thymio length
@onevent
def timer0():
    global state,counter_state,motor_left_target, motor_right_target, NONE
    #counter constants
    RESET = 1
    
    if counter_state == RESET:
        time = 0
        counter_state = NONE
### STATE 3 the robot goes forward 1 thymio length and 6 the robot goes forward to completely avoid the obstacle###             
    if state == 3 or state == 6:
        time += 1         
        if time <= time_one_thym_dist:
            motor_left_target = 80
            motor_right_target = 80
        else:
            counter_state == RESET
            if state == 3:
                state = 4
            if state == 6: 
                state = 0 #stops the robot and terminates the avoidance 