from tdmclient import ClientAsync
import numpy as np
import time


# State for start and stop
state = 0


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
    #Values of speed should be int
    return {
        "motor.left.target": [int(l_speed)],
        "motor.right.target": [int(r_speed)],
    }

def run_ann_without_memory(node, variables):
    global state
    # Weights of neuron inputs
    w_l = np.array([40,  20, -20, -20, -40,  30, -10])
    w_r = np.array([-40, -20, -20,  20,  40, -10,  30])

    # Scale factors for sensors and constant factor
    sensor_scale = 200
    constant_scale = 20

    x = np.zeros(shape=(7,))
    y = np.zeros(shape=(2,))


    if node["button.center"] == 1 and state == 0:
        state = 1
        node.send_set_variables(motors(500, 500, True))
        print("moving!")
        time.sleep(0.1)
    elif node["button.center"] == 1 and state == 1:
       state = 0
       node.send_set_variables(motors(0,0))
       print("Stopping!")
       time.sleep(0.1)

    try:
        if state != 0:
            # Get and scale inputs
            x = np.array(variables["prox.horizontal"]) / sensor_scale

            # Compute outputs of neurons and set motor powers
            y[0] = np.sum(x * w_l)
            y[1] = np.sum(x * w_r)

            print(int(y[0]), int(y[1]), variables["prox.horizontal"])
            node.send_set_variables(motors(y[0], y[1], True))

    except KeyError:
                pass


with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(run_ann_without_memory)
            await client.sleep()
    client.run_async_program(prog)
