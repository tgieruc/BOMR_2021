# -*- coding: utf-8 -*-
"""
Created on Sun Nov 28 22:11:57 2021

@author: eliot
"""

def see_nothing(sensor_array, threshold):
    temp = 0
    for i in range (len(sensor_array)):
        temp = temp + sensor_array[i]
    if temp <= threshold:
        return True
    else:
        return False
    
a = [1, 2, 3, 4, 5, 6]
b = a[4]
    
# print("The Array a is: ", a) #printing the array
# print("The Array b is: ", b) #printing the 2D-Array
   

