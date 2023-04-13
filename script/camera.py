import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

image_pub = rospy.Publisher("opencv_image", Image, queue_size = 10)
camera = cv2.VideoCapture(0)

def main():
    rospy.init_node("kamera_verisi")
    ret, image = camera.read()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    message = CvBridge().cv2_to_imgmsg(image, "rgb8")
    image_pub.publish(message)
    
while 1:
    main()
    
