%%run_python

@onevent 
def button_forward(): # stop counting
    global  motor_left_target, motor_right_target
    motor_left_target = 100
    motor_right_target = 100