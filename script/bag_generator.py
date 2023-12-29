#!/usr/bin/env python3
import rospy

import rosbag
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

if __name__ == '__main__':
    # bag = rosbag.Bag('test.bag', 'w')
    rospy.init_node('save_image_node')


    color_count = 0
    depth_count = 0
    start_time = rospy.Time.now()
    with rosbag.Bag('output.bag', 'w') as outbag:
        for topic, msg, t in rosbag.Bag('/home/usrg/nn_image_conversion/bagfile/bag_busan_cave/2023-08-31-10-41-25.bag').read_messages():
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if topic == "/camera/color/image_raw/compressed":
                print("write image",t)
                msg.header.stamp = rospy.Duration(color_count * 0.1)
                outbag.write(topic, msg, start_time + rospy.Duration(color_count * 0.1))
                color_count +=1
            elif topic == "/camera/aligned_depth_to_color/image_raw":
                msg.header.stamp = rospy.Duration(depth_count * 0.1)
                outbag.write(topic, msg, start_time + rospy.Duration(depth_count * 0.1))
                depth_count +=1
            elif topic == "/os_cloud_node/points":
                msg.header.stamp = rospy.Duration(depth_count * 0.1)
                outbag.write(topic, msg, start_time + rospy.Duration(depth_count * 0.1))
                depth_count +=1
            
        

    # num_image = 6719

    # for i in range(num_image+1):
    #     image_name = "/home/usrg/catkin_ws/src/image_saver/images/data_busan_cave_"+str(i).zfill(4)+".jpg"
    #     print(image_name)
    #     image = cv2.imread(image_name, cv2.IMREAD_COLOR)
    #     cv2.imshow("image",image)
    #     ros_image_msg = bridge.cv2_to_imgmsg(image,"bgr8")


    #     bag.write('/camera/color/image_raw', ros_image_msg)
    

    # bag.close()