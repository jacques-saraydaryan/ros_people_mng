__author__ ='Jacques Saraydaryan'

import sys
import time
import rospy 
import actionlib
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import random
import copy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from ros_color_detection_srvs.srv import DetectColorFromImg

from ros_people_mng_msgs.msg import PeopleMetaInfo,PeopleMetaInfoList
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction,ProcessPeopleFromImgResult
from ros_people_mng_srvs.srv import ProcessPeopleFromImg
from ros_openpose_gossip_srvs.srv import OpenPoseGossip

from FaceDetectionModule import FaceDetectionModule
from PersonMetaInfo import PersonMetaInfo


class DetectPeopleMeta():
    

    def __init__(self,is_face_bounding_box_used):
        self.is_face_bounding_box_used=is_face_bounding_box_used
        self.configure()
        
    def configure(self):
        ## wait for openpose joints detection
        try:
            rospy.wait_for_service('/people_pose_from_img',50)
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

        self._bridge = CvBridge()

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
            rospy.loginfo("-------------------  NB PEOPLE: "+str(len(resp1.personList.persons))+"-----------------------")
            #rospy.loginfo( "service:"+str(len(resp1.personList))
            return resp1.personList
        except rospy.ServiceException, e:
             rospy.logwarn("Service call failed: %s"+str(e))
             return None

    def callGossipPose(self,persons):
        try:
            resp1 = self._gossipPoseSrv(persons)
            #rospy.loginfo("Pose Gossip Of Service")
            #rospy.loginfo( "service:"+str(resp1))
            return resp1
        except rospy.ServiceException, e:
             rospy.logwarn("Service call failed: %s"+str(e))
             return None

    def callColorDetection(self,img):
        try:
            resp1 = self._colorDetectionSrv(img)
            #rospy.loginfo("Colors")
            #rospy.loginfo( "main:"+str(resp1.main_color))
            #rospy.loginfo( "colors:"+str(resp1.main_colors))
            return resp1
        except rospy.ServiceException, e:
             rospy.logwarn("Service call failed: %s"+str(e))
             return None

    def processImg(self,img):
        start_time=time.time()
        personMetoInfoMap={}
        ################################
        ####  PROCESS OVERALL IMG   #### 
        ################################
        rospy.logwarn("---------------------------------------------------:NEW PROCESS-----:" + str(
            round(time.time() - start_time, 3)) + "s")
        start_time=time.time()
        rospy.loginfo("------- Process Data: OPENPOSE -------")
        persons=self.callOpenpose(img)
        persons.image_w=img.width
        persons.image_h=img.height
        #rospy.loginfo(persons)
        if persons == None:
             return
        #rospy.loginfo(persons)

        rospy.logwarn("---------------------------------------------------:OPENPOSE: timeElasped since last operation:" + str(
            round(time.time() - start_time, 3)) + "s")
        start_time = time.time()

        rospy.loginfo("------- Process Data: GOSSIP POSE -------")
        gossip_pose=self.callGossipPose(persons)
        if gossip_pose == None:
            return
        #rospy.loginfo(gossip_pose)
 
        for person in gossip_pose.personsGossip.personsGossip:
            ################################
            ####   PROCESS ONE PERSON   #### 
            ################################
            current_person=PersonMetaInfo(person.id)
            current_person.posture=person.posture
            current_person.handPosture=person.handPosture
            current_person.distanceEval=person.distanceEval
            current_person.setBoundingBox(PersonMetaInfo.PERSON_RECT,person.boundingBox.points)
            current_person.setPosition(person.Cam2MapXYPoint.x,person.Cam2MapXYPoint.y)

            ### FIXME TO REMOVE ONLY FOR TEST 
            #x0,y0=self.getRandomPt(img.height,img.width)
            #x1=x0+30
            #y1=y0+40
            #pt1=Point32()
            #pt1.x=x0
            #pt1.y=y0
#
            #pt2=Point32()
            #pt2.x=x1
            #pt2.y=y1
            #person.shirtRect.points.append(pt1)
            #person.shirtRect.points.append(pt2)
            ### END FIXME TO REMOVE ONLY FOR TEST 

            # convert image msg to cv img for crop purpose
            cv_image = self._bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

            rospy.logwarn(
                "---------------------------------------------------: GOSSIP: timeElasped since last operation:" + str(
                    round(time.time() - start_time, 3)) + "s")
            start_time = time.time()
            rospy.loginfo("------- Process Data: Main Color Detection -------")
            rospy.logdebug("COLOR DETECTION: PERSON:")
            rospy.logdebug(person)

            if len(person.shirtRect.points) ==2 :
                current_person.setBoundingBox(PersonMetaInfo.SHIRT_RECT,person.shirtRect.points)
                #Get main color
                ## crop image with given point the startY and endY coordinates, followed by the startX and endX
                #imCrop = im[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
                #cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
                imCrop = cv_image[int(person.shirtRect.points[0].y):int(person.shirtRect.points[1].y), int(person.shirtRect.points[0].x):int(person.shirtRect.points[1].x)]
                #rospy.loginfo(imCrop)

                if imCrop.size !=0:
                    #convert cv image to image msg to send to service
                    imCrop_msg = self._bridge.cv2_to_imgmsg(imCrop, encoding="bgr8") 
                    
                    dominant_color1=self.callColorDetection(imCrop_msg)

                    #current_person.colorNameMap[PersonMetaInfo.SHIRT_RECT]=dominant_color1.main_color.color_name
                    #current_person.colorRGBMap[PersonMetaInfo.SHIRT_RECT]=dominant_color1.main_color.rgb

                    current_person.setMainColor(PersonMetaInfo.SHIRT_RECT,dominant_color1.main_color.color_name,dominant_color1.main_color.rgb)
                    current_person.setColorList(PersonMetaInfo.SHIRT_RECT,dominant_color1.main_colors.colorList)
                    rospy.loginfo("id:"+str(current_person.id)+"-shirtRect-color:"+str(dominant_color1.main_color.color_name))
                else:
                    rospy.logwarn("Crop Img =[]")

            if len(person.trouserRect.points) ==2 :
                current_person.setBoundingBox(PersonMetaInfo.TROUSER_RECT,person.trouserRect.points)
                #Get main color
                ## crop image with given point the startY and endY coordinates, followed by the startX and endX
                #imCrop = im[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
                imCrop = cv_image[int(person.trouserRect.points[0].y):int(person.trouserRect.points[1].y), int(person.trouserRect.points[0].x):int(person.trouserRect.points[1].x)]
                #convert cv image to image msg to send to service
                #rospy.loginfo("PT1")
                #rospy.loginfo(person.trouserRect.points[0])
                #rospy.loginfo("PT2")
                #rospy.loginfo(person.trouserRect.points[1])
                #rospy.loginfo(imCrop)
                if imCrop.size !=0:
                    imCrop_msg = self._bridge.cv2_to_imgmsg(imCrop, encoding="bgr8") 
                    rospy.logdebug("-----------        trouserRect               ----------------")
                    dominant_color2=self.callColorDetection(imCrop_msg)

                    #current_person.colorNameMap[PersonMetaInfo.TROUSER_RECT]=dominant_color2.main_color.color_name
                    #current_person.colorRGBMap[PersonMetaInfo.TROUSER_RECT]=dominant_color2.main_color.rgb

                    current_person.setMainColor(PersonMetaInfo.TROUSER_RECT,dominant_color2.main_color.color_name,dominant_color2.main_color.rgb)
                    current_person.setColorList(PersonMetaInfo.TROUSER_RECT,dominant_color2.main_colors.colorList)
                    rospy.loginfo("id:"+str(current_person.id)+"-trouserRect-color:"+str(dominant_color2.main_color.color_name))
                else:
                    rospy.logwarn("Crop Img =[]")

            #if len(person.people.points) ==2 :
            #   imCrop = im[int(person.people.points[0].y):int(person.people.points[1].y), int(person.people.points[0].x):int(person.people.points[1].x)]
            #   label self._faceProcess.detectFaceOnImg(imCrop)
            #   if label != None:
            #       current_person.label_id=label

            rospy.logwarn(
                "---------------------------------------------------: COLOR timeElasped since last operation:" + str(
                    round(time.time() - start_time, 3)) + "s")
            start_time = time.time()
            rospy.loginfo("------- Process Data: FACE DETECTION -------")
            rospy.logdebug("FACE DETECTION: BOUNDING BOX:")
            rospy.logdebug(person)
            isImgFace = False
            if self.is_face_bounding_box_used:
                target_box = person.headRect.points
                isImgFace = True
            else:
                target_box = person.boundingBox.points



            if len(target_box) !=0:
                imCropP = cv_image[int(target_box[0].y):int(target_box[1].y), int(target_box[0].x):int(target_box[1].x)]
                #cv2.imshow('image',imCropP)
                #cv2.waitKey(0)
                msg_im = self._bridge.cv2_to_imgmsg(imCropP, encoding="bgr8")
                label, score = self._faceProcess.detectFaceOnImg(msg_im,isImgFace)
                current_person.label_id=str(label)
                current_person.label_score=score
            else:
                rospy.logwarn("No head bounding box for person:"+str(person.id))
                current_person.label_id = str('None')
                current_person.label_score = 0.0

            personMetoInfoMap[current_person.id]=copy.deepcopy(current_person)
        rospy.loginfo("DETECTED PEOPLE ")
        rospy.logdebug(personMetoInfoMap)

        rospy.logwarn("---------------------------------------------------: FACE: timeElasped since last operation:" + str(
            round(time.time() - start_time, 3)) + "s")
        start_time = time.time()

        return personMetoInfoMap

    def getRandomPt(self,h,w):
        x=random.uniform(0, h)
        y=random.uniform(0, w)
        return x,y
