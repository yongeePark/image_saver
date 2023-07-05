#!/usr/bin/env python2
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import time

# Instantiate CvBridge
bridge = CvBridge()

# path = 'images/'
# name = 'marker'
cnt_start = 0
class SaveImage:
    # Constructor
    def __init__(self):
        rospy.init_node('save_image_node')
        # Get Parameter
        topic_name = rospy.get_param('/image_saver/topic_name')
        self.path = rospy.get_param('/image_saver/save_path')
        self.image_name = rospy.get_param('/image_saver/image_name')
        


        self.sub_image1 = rospy.Subscriber(topic_name, Image, self.image_callback,  queue_size=10000)
        
        self.time_interval = 1.0/10
        self.prev_time = rospy.get_time()
        self.count = cnt_start+0

        print("============================")
        print("Topic         : ",topic_name)
        print("Path          : ",self.path)
        print("image name    : ",self.image_name)
        print("Time interval : ",1 / self.time_interval)
        
        
    def image_callback(self, msg):
        # start_time = time.time()

        if (rospy.get_time() - self.prev_time) < self.time_interval:
            return
        print("Received an image "+str(self.count).zfill(4))
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(self.path+self.image_name+'_'+str(self.count).zfill(4)+'.jpg', cv2_img)
            self.count += 1
            self.prev_time = rospy.get_time()
        # elapsed_time = time.time() - start_time
        # print("time : ",elapsed_time)
        # print("fps : ",1 / elapsed_time)

def main():
    save_image = SaveImage()
    print("Start saving image")
    rospy.spin()


if __name__ == '__main__':
    main()

