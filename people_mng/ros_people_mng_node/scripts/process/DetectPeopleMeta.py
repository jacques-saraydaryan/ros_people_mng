__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy 
import actionlib
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from sensor_msgs.msg import Image
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from ros_color_detection_srvs.srv import DetectColorFromImg

from ros_people_mng_msgs.msg import PeopleMetaInfo,PeopleMetaInfoList
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction,ProcessPeopleFromImgResult
from ros_people_mng_srvs.srv import ProcessPeopleFromImg
from ros_openpose_gossip_srvs.srv import OpenPoseGossip

from FaceDetectionModule import FaceDetectionModule

class DetectPeopleMeta():

    def __init__(self):
        self.configure()
        
    def configure(self):
        ## wait for openpose joints detection
        try:
            rospy.wait_for_service('/people_pose_from_img',5)
            rospy.loginfo("service people_pose_from_img READY")
            self._openPoseSrv = rospy.ServiceProxy('people_pose_from_img', DetectPeoplePoseFromImg)
        except Exception as e:
            rospy.logwarn("Service people_pose_from_img call failed: %s" % e)
        
        
        ## wait for gossip pose detection
        try:
            rospy.wait_for_service('/openpose_gossip_srv',5)
            rospy.loginfo("service openpose_gossip_srv READY")
            self._gossipPoseSrv = rospy.ServiceProxy('openpose_gossip_srv', OpenPoseGossip)
        except Exception as e:
            rospy.logwarn("Service openpose_gossip_srv call failed: %s" % e)

        ## wait for color detection service
        try:
            rospy.wait_for_service('/detect_color_srv',5)
            rospy.loginfo("service detect_color_srv READY")
            self._colorDetectionSrv = rospy.ServiceProxy('detect_color_srv', DetectColorFromImg)
        except Exception as e:
            rospy.logwarn("Service detect_color_srv call failed: %s" % e)

       
        self._faceProcess=FaceDetectionModule()

        ## wait for gossip pose detection
        #try:
        #    rospy.wait_for_service('/<TODO>',5)
        #    rospy.loginfo("service <TODO> READY")
        #    self._openPoseSrv = rospy.ServiceProxy('<TODO>', DetectColorFromImg)
        #except Exception as e:
        #    rospy.logwarn("Service <TODO> call failed: %s" % e)

    def callOpenpose(self,img):
        try:
            resp1 = self._openPoseSrv(img)
            rospy.loginfo("nb people")
            rospy.loginfo( "service:"+str(resp1.personList))
            return resp1.personList
        except rospy.ServiceException, e:
             rospy.logwarn("Service call failed: %s"+str(e))
             return None

    def callGossipPose(self,persons):
        try:
            resp1 = self._gossipPoseSrv(persons)
            rospy.loginfo("Pose Gossip Of Service")
            rospy.loginfo( "service:"+str(resp1))
            return resp1
        except rospy.ServiceException, e:
             rospy.logwarn("Service call failed: %s"+str(e))
             return None

    def callColorDetection(self,img):
        try:
            resp1 = self._colorDetectionSrv(img)
            rospy.loginfo("Colors")
            rospy.loginfo( "main:"+str(resp1.main_color))
            rospy.loginfo( "colors:"+str(resp1.main_colors))
            return resp1
        except rospy.ServiceException, e:
             rospy.logwarn("Service call failed: %s"+str(e))
             return None

    def processImg(self,img):
        rospy.loginfo("------- Process Data: OPENPOSE -------")
        persons=self.callOpenpose(img)
        persons.image_w=img.width
        persons.image_h=img.height
        rospy.loginfo(persons)
        if persons == None:
             return
        rospy.loginfo(persons)


        rospy.loginfo("------- Process Data: GOSSIP POSE -------")
        gossip_pose=self.callGossipPose(persons)
        if gossip_pose == None:
            #TODO
            pass
        rospy.loginfo(gossip_pose)

        rospy.loginfo("------- Process Data: COLOR DETECTION -------")
        #CAUTION On execution per person per body focus
        dominant_color=self.callColorDetection(img)
        if dominant_color == None:
            #TODO
            pass
        rospy.loginfo(dominant_color.main_color)

        rospy.loginfo("------- Process Data: FACE DETECTION -------")
        #CAUTION On execution per person
        self._faceProcess.processFaceOnImg(img,'JSA2')

        pass
