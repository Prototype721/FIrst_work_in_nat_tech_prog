import pymurapi as mur
import cv2 as cv
import time

prev_time = 0 
prev_error = 0.0

auv = mur.mur_init()


def clamp(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v

def keep_depth(value):
    global prev_time
    global prev_error
    cur_time = int(round(time.time() * 1000))
    
    error = auv.get_depth() - value
    
    power_2 = 0
    power3 = 0
    power_value = error * 70
    dif_value = 5 / (cur_time - prev_time) * (error - prev_error)
    power_2 = clamp(power_value + dif_value, 100, -100)
    power_3 = clamp(power_value + dif_value, 100, -100)
        
    auv.set_motor_power(2, power_2)
    auv.set_motor_power(3, power_3)
    prev_error = error
    prev_time = cur_time
    
while True:
    #depth = auv.get_depth()
    #print(depth)
    #yaw = auv.get_yaw()
    #print(yaw)
    
    #image = auv.get_image_front()
    
    #cv.imshow("HI", image)
    #cv.waitKey(10)
    #-100 lj 100
    keep_depth(2)
    time.sleep(0.03)
