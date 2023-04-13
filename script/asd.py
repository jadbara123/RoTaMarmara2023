import rospy
from std_msgs.msg import Float32MultiArray
engel = 0

def engel_callback(data):
    global engel 
    engel = data.data

def main():
    rospy.init_node('subscriber')
    rospy.Subscriber('engel_mesafe', Float32MultiArray, engel_callback)
    print(engel)

while 1:
    main() 
