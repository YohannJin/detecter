#!/bin/python3
from operator import is_not
import rospy
import numpy as np
import cv2
# from utils import ARUCO_DICT
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
#from std_msgs.msg import IntList
from geometry_msgs.msg import Pose
import sys

from redmarkerdetection import *    # image processing by cython

#transfer pos info to ros msg
#only position no orientation
def pose_aruco_2_ros(rvec, tvec):
    aruco_pose_msg = Pose()
    aruco_pose_msg.position.x = tvec[0]
    aruco_pose_msg.position.y = tvec[1]
    aruco_pose_msg.position.z = tvec[2]
    aruco_pose_msg.orientation.x = 0
    aruco_pose_msg.orientation.y = 0
    aruco_pose_msg.orientation.z = 0
    aruco_pose_msg.orientation.w = 0
    return aruco_pose_msg

class arucoPose:
    def __init__(self):
        #publish the cube pose
        self.aruco_pose_pub = rospy.Publisher("aruco_pose", Pose, queue_size= 10)

        #publish the sind pose
        #3 posese at same time
        self.aruco_sink1_pub = rospy.Publisher("aruco_sink1", Pose, queue_size=1)
        self.aruco_sink2_pub = rospy.Publisher("aruco_sink2", Pose, queue_size=1)
        self.aruco_sink3_pub = rospy.Publisher("aruco_sink3", Pose, queue_size=1)

        # self.tag1_pub = rospy.Publisher("tag1", Int32, queue_size=1)
        # self.tag2_pub = rospy.Publisher("tag2", Int32, queue_size=1)
        # self.tag3_pub = rospy.Publisher("tag3", Int32, queue_size=1)
        self.tag_pub = rospy.Publisher("tags", Point, queue_size=1)

        self.tag_succeed = False



        #screen output
        self.aruco_pose_image_pub = rospy.Publisher("aruco_pose_image", Image, 10)
        self.bridge = CvBridge()

        #the image source
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageColorCallback)

        #the known datas
        self.MARKER_SIZE = 0.045 # [m]
        #tpl
        self.ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

        self.MTX = np.array([[617.3054000792732, 0.0, 424.0], 
                        [0.0, 617.3054000792732, 240.0], 
                        [0.0, 0.0, 1.0]])
        self.DIST = np.array([0., 0., 0., 0., 0.])

        #to store the ids
        self.id_list = []
        #linear
        self.tvec_list = []
        #rotation
        self.rvec_list = []
        #from redmarker_detector
        load_template()

    #run while receiving image_color
    def imageColorCallback(self,data):
        #from rosmsg to image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)
        
        #shape info of image
        (h, w, channel) = cv_image.shape

        #TODO can be tuned
        seg_papram = np.array([0,15,125,180,46,80],dtype="uint8")

        #red marker detection
        #output: id of cubes, pos, orientation
        id_list,tvec_list,rvec_list = marker_detection(cv_image,seg_papram)

        #screen output
        # cv2.imshow('frame', cv_image)
        # cv2.waitKey(1)

        # flags, default false
        target_detected = False
        sink1_detected = False
        tab_detected = False
        tag1 = False
        tag2 = False
        tag3 = False
        sink2_detected = False
        sink3_detected = False
        field_top = -0.15
    
        tag = Point()
        #tab deteciton
        if(len(id_list)>4):
            for i in range(len(id_list)):
                aruco_pose_msg = pose_aruco_2_ros(rvec_list[i],tvec_list[i])
                msg = Int32()
                msg = id_list[i]
                if id_list[i] == 6:
                    id_list[i] = 2
                if (id_list[i] >= 0 and id_list[i] <= 4) and aruco_pose_msg.position.y < -0.2 and self.tag_succeed == False: 
                    if(tag1==False):
                        # print("t1 ", id_list[i])
                        tag.x = id_list[i]
                        #self.tag1_pub.publish(msg)
                        print(tag.x)
                        tag1 = True
                    elif(tag2 == False):
                        # print("t2 ", id_list[i])
                        tag.y = id_list[i]
                        print(tag.y)
                        #self.tag2_pub.publish(msg)
                        tag2 = True
                    elif(tag3 == False):
                        # print("t3 ", id_list[i])
                        tag.z = id_list[i]
                        #self.tag3_pub.publish(msg)
                        print(tag.z)
                        print(tag)
                        tag3 = True
                        self.tag_succeed = True
                        self.tag_pub.publish(tag)
        else:
            for i in range(len(id_list)):
            # transfer to rosmsg
                aruco_pose_msg = pose_aruco_2_ros(rvec_list[i],tvec_list[i])
                #sink detection             
                #if sink1_detected == False: #(id_list[i] == 2 or id_list[i] == 4 or id_list[i] == 5) and
                   # if aruco_pose_msg.position.y > -10.012 and aruco_pose_msg.position.y < 10.02: #0.0075 0.0085
                # if id_list[i] >= 0:
                #     self.aruco_sink1_pub.publish(aruco_pose_msg)
                #     sink1_detected = True
                #     print("Sink_Id : ",id_list[i]," : ",aruco_pose_msg)
                #cube detection
                #(id_list[i] >= 0 and id_list[i] <= 4)
                if id_list[i] >= 0 and id_list[i] <= 4:#and target_detected == False:                
                    #check height, only choose the bottom one
                    if aruco_pose_msg.position.y < 0.025 and aruco_pose_msg.position.y > 0.0:
                        self.aruco_pose_pub.publish(aruco_pose_msg)
                        target_detected = True
                        print("Id : ",id_list[i]," : ",aruco_pose_msg)
                elif id_list[i] == 7 and sink1_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink1_pub.publish(aruco_pose_msg)
                        print("sink1 !!!")
                        sink1_detected = True
                elif id_list[i] == 6 and sink2_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink2_pub.publish(aruco_pose_msg)
                        sink2_detected = True
                        print("sink2 !!!")
                elif id_list[i] == 5 and sink3_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink3_pub.publish(aruco_pose_msg)
                        sink3_detected = True
                        print("sink3 !!!")



def main():
    #ap = arucoPose()
    rospy.init_node('aruco_pose_node', anonymous=True)
    ap = arucoPose()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # cv2.destoryAllWindows()


if __name__ == '__main__':
    main()
