import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

lastShowTime = 0.0

br = CvBridge()
img_rgb, img_depth = [],[]

def bound(img):
    minVal = 300.0
    maxVal = 5000.0
    thresh = ((img.astype(float)-minVal)*(img > minVal))
    scaled = (thresh * (255. / (maxVal- minVal)))
    scaled[scaled > 255] = 255
    return scaled.astype(np.uint8)

def callback_depth(data):
    global img_depth
    temp_img = br.imgmsg_to_cv2(data)
    temp_img = bound(temp_img)
    img_depth = cv2.cvtColor(temp_img, cv2.COLOR_GRAY2RGB)

def callback_rgb(data):
    global img_rgb
    temp_img = br.imgmsg_to_cv2(data)
    img_rgb = cv2.cvtColor(temp_img, cv2.COLOR_BGR2RGB)
        

def main():
    global lastShowTime
    rospy.init_node("depthToImage")

    lastShowTime = 0.0
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback_depth)
    w,h = 640, 480

    while True:
        if type(img_rgb) != list and type(img_depth) != list:
            img_comb = np.zeros((h,2*w,3), np.uint8)
            img_comb[:h,:w] = img_rgb
            img_comb[:h,w:w+w] = img_depth

            cv2.imshow("test",img_comb)

        if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
            break

    print("Exiting")
    # rospy.spin()


if __name__ == '__main__':
    main()