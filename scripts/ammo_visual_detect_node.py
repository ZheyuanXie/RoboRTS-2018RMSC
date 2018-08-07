#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class AmmoDetect(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub_ = rospy.Subscriber("camera_0", Image, callback=self.ImageCB)
        self.pose_pub_ = rospy.Publisher("ammo_pose", PoseStamped, queue_size=1)
    
    def ImageCB(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        mtx = np.array(
            [[527.917811, 0.0, 326.678226],
            [0.0,494.424655,217.938651],
            [0.0,0.0,1.0]])
        dist_coeffs = np.array([0.005612,-0.048042,0.004065,0.005149,0.0])
        undistort_img = cv2.undistort(cv_image, mtx, dist_coeffs)

        preprocessed_img = self.Preprocess(undistort_img)
        edge_detect_img = self.EdgeDetection(preprocessed_img)
        self.FindContour(edge_detect_img)
        cv2.imshow('w', edge_detect_img)
        cv2.waitKey(3)
        
    
    def Preprocess(self, img):
        yuv_img = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)
        thres_low = np.array([15,102,141])
        thres_high = np.array([111,135,165])
        red_img = cv2.inRange(yuv_img, thres_low, thres_high)
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7), (4,4))
        dilate_img = cv2.dilate(red_img, element)
        erode_img = cv2.erode(dilate_img, element)
        return erode_img
    
    def EdgeDetection(self, img):
        smallElement = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3), (1,1))
        dilate_img = cv2.dilate(img, smallElement)
        return dilate_img - img

    def FindContour(self, img):
        contours = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            cv2.minAreaRect(contour)
    
    def CalcTransform(self, contour):
        pass


if __name__ == "__main__":
    rospy.init_node("ammo_detect_node")
    ad = AmmoDetect()
    rospy.spin()