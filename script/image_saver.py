# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

path = 'images/'
name = 'marker'
cnt_start = 0
class SaveImage:
    def __init__(self):
        rospy.init_node('save_image_node')
        self.sub_image1 = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback,  queue_size=10000)
        
        self.time_interval = 0.5
        self.prev_time = rospy.get_time()
        self.count = cnt_start+0
        
        
    def image_callback(self, msg):
        if (rospy.get_time() - self.prev_time) < self.time_interval:
            return
        print("Received an image "+str(self.count).zfill(4))
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(path+name+'_'+str(self.count).zfill(4)+'.jpg', cv2_img)
            self.count += 1
            self.prev_time = rospy.get_time()

def main():
    save_image = SaveImage()
    rospy.spin()


if __name__ == '__main__':
    main()
