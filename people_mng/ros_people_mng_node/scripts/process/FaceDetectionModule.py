__author__ ='Jacques Saraydaryan'


import rospy 
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from sensor_msgs.msg import Image
from people_face_identification.srv import LearnFaceFromImg,GetImgFromId



class FaceDetectionModule():

    def __init__(self):
         ## wait for face detection learning face
        try:
            rospy.wait_for_service('/learn_face_from_img',5)
            rospy.wait_for_service('/get_img_from_id',5)

            rospy.loginfo("service learn_face_from_img,get_img_from_id READY")
            self._faceLearnSrv = rospy.ServiceProxy('learn_face_from_img', LearnFaceFromImg)
            self._getImgFromIdSrv = rospy.ServiceProxy('get_img_from_id', GetImgFromId)
        except Exception as e:
            rospy.logwarn("Service learn_face_from_img,get_img_from_id call failed: %s" % e)

    def processFaceOnImg(self,img,label):
        try:
            resp1=self._faceLearnSrv(label,img)
            resp2 = self._getImgFromIdSrv(label)
            rospy.loginfo("Img of learnt label:"+str(label))
            rospy.loginfo( "IMG:"+str(resp2))
            return resp2
        except rospy.ServiceException, e:
             rospy.logwarn("Service call failed: %s"+str(e))
             return None
