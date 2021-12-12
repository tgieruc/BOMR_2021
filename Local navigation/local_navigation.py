# -*- coding: utf-8 -*-
"""
Created on Tue Dec  7 14:57:23 2021

@author: guill
"""
from tdmclient import aw

global side


def set_speed(speed_left, speed_right, node):
    v = {
        "motor.left.target": [speed_left],
        "motor.right.target": [speed_right],
    }
    aw(node.set_variables(v))


def get_speed(node):
    right_speed = node['motor.right.speed']
    left_speed = node['motor.left.speed']
    return [left_speed, right_speed]


async def go_straight(motor_speed, duration, node, client):
    """
    goes straight for a certain duration
    """
    set_speed(motor_speed, motor_speed, node)  # test with lower speed value
    await client.sleep(duration)
    set_speed(0, 0, node)


async def NN_avoid(sensor_scale, node, client, side):
    """
    avoids obstacle with neural network
    """
    y = [0, 0]

    while all([x != 0 for x in node['prox.horizontal'][0:5]]):
        print("fdp")
        # asymetric w_l[1] != w_r[1] so the robot choses one side if it arrives straight on the obstacle
        w_l = [5, -10, -5, 4, 0]
        w_r = [-5, -5, 5, 0, 4]

        x = [0, 0, 0, 0, 0]

        prox_horizontal = node['prox.horizontal'][:-2]
        print(prox_horizontal)
        left_prox = prox_horizontal[0] // sensor_scale
        right_prox = prox_horizontal[4] // sensor_scale

        # Memory
        x[3] = y[0] // 10
        x[4] = y[1] // 10

        for i in range(1, 4):  # we only take the 3 sensors of the middle
            # Get and scale inputs
            x[i - 1] = prox_horizontal[i] // sensor_scale

        print(x)

        y = [80, 80]

        for i in range(len(x)):
            # Compute outputs of neurons and set motor powers
            y[0] = y[0] + x[i] * w_l[i]
            y[1] = y[1] + x[i] * w_r[i]

        print(y)

        if left_prox > 1:
            side = 'left'
        if right_prox > 1:
            side = 'right'

        await client.sleep(0.1)

        set_speed(y[0], y[1], node)
        await client.sleep(0.1)  # let the variables be updated
        return side, 1


async def turn(sensor_scale, node, client, side):
    """
    turn until obstcale is detected
    """
    adjust = 3
    doneTurn = False
    y = [0, 0]

    if side == 'none':
        print("side not set")
        doneTurn = True

    while not doneTurn:

        prox_horizontal = node['prox.horizontal'][:-2]
        left_prox = prox_horizontal[0] // sensor_scale
        right_prox = prox_horizontal[4] // sensor_scale

        print(left_prox)
        print(right_prox)

        if side == 'right':
            if right_prox <= 1 + adjust:
                y = [80, 0]
            if right_prox > 1 + adjust:
                doneTurn = True
                print("done turn right")

        if side == 'left':
            if left_prox <= 1 + adjust:
                y = [0, 80]
            if left_prox > 1 + adjust:
                doneTurn = True
                print("done turn left")
        set_speed(y[0], y[1], node)
        await client.sleep(0.1)  # let the variables be updated


async def follow_obstacle(sensor_scale, node, client, side):
    """
    follows the obstacle side until its end
    """
    adjust2 = 1000
    constant_scale = 10
    doneFollow = False
    y = [0, 0]

    if side == 'none':
        print("side not set")
        doneFollow = True

    while not doneFollow:
        prox_horizontal = node['prox.horizontal'][:5]
        left_prox = prox_horizontal[0] // sensor_scale
        right_prox = prox_horizontal[4] // sensor_scale

        left_prox_adjust = left_prox - adjust2 // sensor_scale
        right_prox_adjust = right_prox - adjust2 // sensor_scale
        if side == 'right':
            y = [80, 80 + right_prox_adjust * constant_scale]
        if side == 'left':
            y = [80 + left_prox_adjust * constant_scale, 80]

        set_speed(y[0], y[1], node)
        await client.sleep(0.1)  # let the variables be updated

        if left_prox < 1 and right_prox < 1:
            doneFollow = True


async def obstacle_avoidance(node, client):
    side = 'none'
    done = 0
    while done == 0:
        side, done = await NN_avoid(400, node, client, side)

    await go_straight(100, 3, node, client)
    print("went away")
    await turn(400, node, client, side)
    print("turned away")

    await follow_obstacle(400, node, client, side)
    print("followed away")

    await go_straight(100, 5, node, client)
    print("went away")

