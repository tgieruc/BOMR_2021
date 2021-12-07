# -*- coding: utf-8 -*-
"""
Created on Tue Dec  7 14:57:23 2021

@author: guill
"""

def motors(l_speed=500, r_speed=500, verbose=False):
    """
    Sets the motor speeds of the Thymio 
    param l_speed: left motor speed
    param r_speed: right motor speed
    param verbose: whether to print status messages or not
    """
    # Printing the speeds if requested
    if verbose:
        print("\t\t Setting speed : ", l_speed, r_speed)
    return {
        "motor.left.target": [l_speed],
        "motor.right.target": [r_speed],
    }

def test_see_nothing_center(close_threshold=1, verbose=False):
    """
    tests if the three middle sensors see nothing
    """
    if any([x>close_threshold for x in node['prox.horizontal'][1:-3]]):
        return False
    if verbose: print("\t\t degagé de l'obstacle")
    return True

async def go_straight(motor_speed=100, duration=5, verbose=False):
    """
    goes straight for a certain duration
    """
    if verbose: print("Starting go straight behaviour")
        
    await node.set_variables(motors(motor_speed, motor_speed)) #test with lower speed value
    await client.sleep(duration)
    await node.set_variables(motors(0, 0))
    
async def NN_avoid(motor_speed=100, sensor_scale=400, verbose=False):
    global side
    """
    avoids obstacle with neural network
    """
    see_nothing = False
    y = [0,0] 
    
    while not see_nothing:
        #asymetric w_l[1] != w_r[1] so the robot choses one side if it arrives straight on the obstacle
        w_l = [5, -10, -5, 4, 0]
        w_r = [-5, -5, 5, 0, 4]

        x = [0,0,0,0,0]

        if verbose: print("Starting avoidance with neural network")

        prox_horizontal = node['prox.horizontal'][:-2]
        print(prox_horizontal)
        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale

        # Memory
        x[3] = y[0]//10
        x[4] = y[1]//10        

        for i in range(1,4): #we only take the 3 sensors of the middle
            # Get and scale inputs
            x[i-1] = prox_horizontal[i] // sensor_scale
            
        print(x)
        
        y = [80,80] 

        for i in range(len(x)):    
            # Compute outputs of neurons and set motor powers
            y[0] = y[0] + x[i] * w_l[i]
            y[1] = y[1] + x[i] * w_r[i]
        
        print(y)
        
        if test_see_nothing_center(1,True): 
            y = [0,0]
            if left_prox > 1:
                side = 'left'
            if right_prox > 1:
                side = 'right'
                
        if test_see_nothing_center(1,True) and left_prox < 1 and right_prox < 1:
            see_nothing = True    
                
            print(side)
            
        await node.set_variables(motors(y[0],y[1]))
        await client.sleep(0.1)#let the variables be updated
        
async def turn(motor_speed=100,sensor_scale=400,  verbose=False):
    global side
    """
    turn until obstcale is detected
    """
    adjust = 3
    doneTurn = False
    y = [0,0] 
    
    
    if side == 'none':
        print("side not set")
        doneTurn = True
        
    while not doneTurn:
        
        if verbose: print("turning",side)
            
        prox_horizontal = node['prox.horizontal'][:-2]
        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale
        
        print(left_prox)
        print(right_prox)
        
        if side == 'right':
            if right_prox <= 1 + adjust:
                y = [80,0]
            if right_prox > 1 + adjust:
                doneTurn = True
                print("done turn right")
                
        if side == 'left':
            if left_prox <= 1 + adjust:
                y = [0,80]
            if left_prox > 1 + adjust:
                doneTurn = True
                print("done turn left")
        await node.set_variables(motors(y[0],y[1]))        
        await client.sleep(0.1)#let the variables be updated
        
async def follow_obstacle(motor_speed=100,sensor_scale=400, verbose=False):
    global side
    """
    follows the obstacle side until its end
    """
    adjust2 = 1000
    constant_scale = 10
    doneFollow = False
    y = [0,0] 
    
    if side == 'none':
        print("side not set")
        doneFollow = True
        
    while not doneFollow:
        
        if verbose: print("following wall detected by",side, 'sensor')
            
        prox_horizontal = node['prox.horizontal'][:5]
        left_prox = prox_horizontal[0]// sensor_scale
        right_prox = prox_horizontal[4]// sensor_scale
        
        print(left_prox)
        print(right_prox)
        
        left_prox_adjust = left_prox -adjust2//sensor_scale
        right_prox_adjust = right_prox -adjust2//sensor_scale
        if side == 'right':
            y = [80,80+right_prox_adjust*constant_scale]
        if side == 'left':
            y = [80+left_prox_adjust*constant_scale,80]
            
        await node.set_variables(motors(y[0],y[1]))    
        await client.sleep(0.1)#let the variables be updated
       
        if left_prox < 1 and right_prox < 1:
            doneFollow = True
            
async def obstacle_avoidance(verbose=True):
    done = False
    side = 'none'
    while not done:
        side = 0;
        await NN_avoid(verbose=verbose)
        
        await go_straight(duration =3,verbose=verbose)
        
        await turn(verbose=verbose)
        
        await follow_obstacle(verbose=verbose)
        
        await go_straight(duration=5,verbose=verbose)
        
        done = True