#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from lane_detection import LaneDetection
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import can

#void setup
image_pub = rospy.Publisher("opencv_image", Image, queue_size = 10)
camera = cv2.VideoCapture(-1)
start = time.time()
alinan_mesafe = 0
tabelalar = 20
motion_mem = 0

#float the angles
def mapfloat(x,  in_min,  in_max,  out_min,  out_max):

    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;

#seriously?
def rad_to_deg(rad):
    angle = rad*180/np.pi
    return round(angle,2)

def callback_tabela(data):
    global tabela 
    tabela = data.data

# Engel Callback fonksiyonu efenim
def callback_engel(data):
    global alinan_mesafe
    alinan_mesafe = data.data

def cizgi_takip():
    rospy.init_node('cizgi_takip_kodu', anonymous=True)
    kp = 0.8 # P parametresi
    ki = 0.0001  # I parametresi
    kd = 0.0001  # D parametresi
    integral = 0.0
    last_error = 0.0			
    xpoints = []
    ypoints = []
    y2points= []
    while not rospy.is_shutdown():
        print("cizgi takip calisiyor")
        #ana dugum
        sec = 0
        #kamera sub   
        initial_time = rospy.get_rostime().secs
        #cv2 ros bridge
        ret, img = camera.read()
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        message = CvBridge().cv2_to_imgmsg(img, "rgb8")
        image_pub.publish(message)
        img = img[0:img.shape[0], 0:int(img.shape[1]/2)] 
        img = img[200:img.shape[0], 0:img.shape[1]]
        #roi test
        warped,_ = LaneDetection().region_of_interest(img)   
        end = time.time()  
        sec  = end - start
        out,refangle = LaneDetection().main(img)
            
        #pid ayarlari ok this is not turkish
        error = 0 + refangle
        integral += error * (1.0 / rospy.get_time())
        derivative = (error - last_error) / rospy.get_time()
        output = kp * error + ki * integral + kd * derivative
        last_error = error
        
        #visualize
        cv2.imshow("original", img)
        can_out = mapfloat(-rad_to_deg(output),-90,90,0,255)
        print(can_out)
        basla_flag = True
        
        #mesaj olusturma, ok ok last turkish
        bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
        msg = can.Message(arbitration_id=0x560, data = [1, int(can_out), 135, 0, 0], is_extended_id = False)
        
        #get other data
        rospy.Subscriber('engel_mesafe', Float32MultiArray, callback_engel)
        rospy.Subscriber('tabela', Int32MultiArray, callback_tabela)
        
        print(alinan_mesafe)
        #escape obstacle
        if alinan_mesafe == 0:
            print("lidaar hatasi")
        
        elif alinan_mesafe != 0:
            if alinan_mesafe[0] < 3 and alinan_mesafe[1] < 4 and alinan_mesafe[2] > 3:
                #sola don
                msg = can.Message(arbitration_id=0x560, data = [1, 45, 130, 0 , 4], is_extended_id = False)
            elif alinan_mesafe[0] < 4 and alinan_mesafe[1] > 4 and alinan_mesafe[2] < 3:
                #saga don
                msg = can.Message(arbitration_id=0x560, data = [1, 210, 130, 0, 1], is_extended_id = False)
            elif alinan_mesafe[0] < 4 and alinan_mesafe[1] < 3 and alinan_mesafe[2] < 3:
                #dur 
                msg = can.Message(arbitration_id=0x560, data = [0, 128, 128, 255, 5], is_extended_id = False)
            elif alinan_mesafe[0] < 4 and alinan_mesafe[1] > 4 and alinan_mesafe[2] > 4:
                #saga don
                msg = can.Message(arbitration_id=0x560, data = [1, 210, 135, 0, 0], is_extended_id = False)

        #send message
        while basla_flag:
            try:
                bus.send(msg)
                print(f"Message sent on {bus.channel_info}")
                time.sleep(0.05)
                basla_flag = False
            except can.CanError:
                print("Message NOT sent") 
        cv2.imshow("original",img)  
        print("calisti")
        cv2.waitKey(1)

while 1:
    cizgi_takip()
