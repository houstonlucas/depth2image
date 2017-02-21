#!/usr/bin/python
import rospy
import cv2
import numpy as np
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

br = CvBridge()
img_rgb, img_depth = [],[]

def bound(img):
    #Bounds the uint16 depth data to uint8 [0,255]
    minVal = 300.0
    maxVal = 5000.0
    thresh = ((img.astype(float)-minVal)*(img > minVal))
    scaled = (thresh * (255. / (maxVal- minVal)))
    scaled[scaled > 255] = 255
    return scaled.astype(np.uint8)

def callback_depth(data):
    #saves the converted depth image to img_depth
    global img_depth
    temp_img = br.imgmsg_to_cv2(data)
    temp_img = bound(temp_img)
    img_depth = cv2.cvtColor(temp_img, cv2.COLOR_GRAY2RGB)

def callback_rgb(data):
    #saves the converted rgb image to img_rgb
    global img_rgb
    temp_img = br.imgmsg_to_cv2(data)
    img_rgb = cv2.cvtColor(temp_img, cv2.COLOR_BGR2RGB)
        

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        '--rate', '-r', required=False, type = float,
        default = 10.0, 
        help='Specifies the collection rate in Hz. (Defaults to 10Hz)'
    )
    parser.add_argument(
        '--outDir', '-o', required=False, type = str,
        default = 'out/img',
        help="Specifies the output location. (Defaults to 'out/img')"
    )
    args = parser.parse_args(rospy.myargv()[1:])


    rospy.init_node("depth2Image")

    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback_depth)
    w,h = 640, 480

    batchLocation = args.outDir
    saveDelay = 1.0 / args.rate

    lastSave = 0.0
    imgNumber = 0
    while True:
        # Only run if the images have been changed from the inital state.
        if type(img_rgb) != list and type(img_depth) != list:
            #Combine Images
            img_comb = np.zeros((h,2*w,3), np.uint8)
            img_comb[:h,:w] = img_depth
            img_comb[:h,w:w+w] = img_rgb

            cv2.imshow("test",img_comb)

            if time.time() - lastSave > saveDelay:
                #Save Image
                lastSave = time.time()
                fname = batchLocation + str(imgNumber).zfill(4) + ".png"
                cv2.imwrite(fname , img_comb)
                imgNumber += 1

        #Exit on 'q' or Esc
        if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
            break

    print("Exiting")

if __name__ == '__main__':
    main()