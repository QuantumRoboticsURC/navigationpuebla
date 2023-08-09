#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class ImagePublisher():
    def __init__(self):
        rospy.init_node("image_publisher")
        self.cap = cv2.VideoCapture("/dev/video2")
        self.img_1_pub = rospy.Publisher("/image_raw", Image, queue_size=1)
        self.rate = rospy.Rate(10)
        self.image_msg = Image()

    def cv2_to_imgmsg(self, image, encoding = "bgr8"):
        if encoding == "bgr8":
            self.image_msg.header = Header()
            self.image_msg.height = image.shape[0]
            self.image_msg.width = image.shape[1]
            self.image_msg.encoding = encoding
            self.image_msg.is_bigendian = 0
            self.image_msg.step = image.shape[1]*image.shape[2]

            data = np.reshape(image, (self.image_msg.height, self.image_msg.step) )
            data = np.reshape(image, (self.image_msg.height*self.image_msg.step) )
            data = list(data)
            self.image_msg.data = data
            return self.image_msg
        else:            
            raise Exception("Error while convering cv image to ros message") 

    def main(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                self.cv2_to_imgmsg(frame)
                self.img_1_pub.publish(self.image_msg)
                self.rate.sleep()
        

if __name__ == "__main__":
    img_publisher = ImagePublisher()
    img_publisher.main()

    