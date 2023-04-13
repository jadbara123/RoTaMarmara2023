#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float32MultiArray

mesafe_pub = rospy.Publisher('engel_mesafe', Float32MultiArray, queue_size=10)

def callback_laser(data):
        global mesafe
        adet = len(data.ranges)
        bolgeler = {
        'on' : min(min(data.ranges[430:470]),8),
        'sol': min(min(data.ranges[410:429]),8),
        'sag': min(min(data.ranges[471:490]),8)
    }
        my_msg = Float32MultiArray()
        my_msg.data = [bolgeler['on'], bolgeler['sag'], bolgeler['sol']]
        mesafe_pub.publish(my_msg)

def main():
    print("engel algilama calisiyor")
    rospy.init_node('engel_algila', anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber("/scan", LaserScan, callback_laser)
        rospy.spin()
while 1:
    main()
