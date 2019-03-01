#!/usr/bin/env python  
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

from ros_people_mng_msgs.msg import PeopleMetaInfoDetails,PeopleMetaInfo,PeopleMetaInfoList
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction,ProcessPeopleFromImgResult
from ros_people_mng_srvs.srv import ProcessPeopleFromImg

from process.DetectPeopleMeta import DetectPeopleMeta,PersonMetaInfo
from process.DisplayMetaData import DisplayMetaData
from geometry_msgs.msg import Point32

class PeopleMngNode():


    def __init__(self):
        rospy.init_node('people_mng_node', anonymous=False)
        self.configure()

        # Subscribe to the image 
        self.sub_rgb = rospy.Subscriber("/image", Image, self.rgb_callback, queue_size=1)
        self.pub_people_meta_info = rospy.Publisher("/people_meta_info", PeopleMetaInfoList, queue_size=1)
        self.pub_people_meta_info_img = rospy.Publisher("/people_meta_info_img", Image, queue_size=1)

        #declare ros service 
        self.detectPeopleMetaSrv = rospy.Service('detect_people_meta_srv', ProcessPeopleFromImg, self.detectPeopleMetaSrvCallback)

         # create action server and start it
        self._actionServer = actionlib.SimpleActionServer('detect_people_meta_action', ProcessPeopleFromImgAction, self.executePeopleMetaActionServer, False)
        self._actionServer.start()

        self.current_img=None

        rospy.spin()


    def configure(self):
        self.is_face_bounding_box_used=rospy.get_param('/is_face_bounding_box_used',False)
        rospy.loginfo("Param: is_face_bounding_box_used:"+str(self.is_face_bounding_box_used))
        self._bridge = CvBridge()
        self._detect_people_meta=DetectPeopleMeta(self.is_face_bounding_box_used)
        data_folder=rospy.get_param('imgtest_folder','../data')
        self.displayMetaData=DisplayMetaData(data_folder+"/icon/",False,True,False)
        


    def rgb_callback(self, data):
        #FIXME need to protect to avoid concurrency?
        self.current_img=data
        image_to_process= self.current_img
        start_time=time.time()
        rospy.logwarn("---------------------------------------------------: timeElasped since start:"+str(0)+"s")
        try:
            peopleMetaInfoList=PeopleMetaInfoList()
            people_list=[]
            result=self._detect_people_meta.processImg(image_to_process)
            for person in result.values():
                rospy.logdebug('-')
                rospy.logdebug(str(person))
                current_peopleMeta=self.convertPeoplToRosMsg(person)
                people_list.append(current_peopleMeta)
            peopleMetaInfoList.peopleList=people_list
            peopleMetaInfoList.img=image_to_process
            #publish metaData
            rospy.logwarn( "---------------------------------------------------: timeElasped since start:" + str(round(time.time()-start_time,3)) + "s")
            self.pub_people_meta_info.publish(peopleMetaInfoList)

            #compute display
            cv_image = self._bridge.imgmsg_to_cv2(image_to_process, desired_encoding="bgr8")
            cv_img_to_display =self.displayMetaData.displayResult(peopleMetaInfoList,cv_image)
            msg_img = self._bridge.cv2_to_imgmsg(cv_img_to_display, encoding="bgr8")   
            #publish image with MetaData
            self.pub_people_meta_info_img.publish(msg_img)
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding :, error:[%s]", str(e))
        
    
    def detectPeopleMetaSrvCallback(self,req):
        image_to_process=''
        if len(req.img.goal.data) == 0:
            image_to_process=self.current_img
        else:
            image_to_process=req.img

        peopleMetaInfoList=PeopleMetaInfoList()
        people_list=[]
        result=self._detect_people_meta.processImg(image_to_process)
        for person in result.values():
            #rospy.logwarn('-')
            #rospy.logwarn(str(person))
            current_peopleMeta=self.convertPeoplToRosMsg(person)
            people_list.append(current_peopleMeta)
        peopleMetaInfoList.peopleList=people_list
        return peopleMetaInfoList
        

    def processImg(self, img):
        #TODO
        pass

    def executePeopleMetaActionServer(self, goal):
        image_to_process=''
        #rospy.logwarn(goal.img)
        if len(goal.img.data) == 0:
            if self.current_img != None:
                image_to_process=self.current_img
            else:
                 rospy.logwarn("current_img is currently no set, no image to process")
                 self._actionServer.set_aborted()
                 return
        else:
            image_to_process=goal.img
        isActionSucceed=False
        action_result = ProcessPeopleFromImgResult()
        try:
            peopleMetaInfoList=PeopleMetaInfoList()
            people_list=[]
            result=self._detect_people_meta.processImg(image_to_process)
            for person in result.values():
                rospy.logdebug('-')
                rospy.logdebug(str(person))
                current_peopleMeta=self.convertPeoplToRosMsg(person)
                people_list.append(current_peopleMeta)
            peopleMetaInfoList.peopleList=people_list
            action_result.peopleMetaList=peopleMetaInfoList
            isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(action_result), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded(action_result)
        else:
            self._actionServer.set_aborted()

    def convertPeoplToRosMsg(self,people):
        rospy.logdebug(people)
        current_people=PeopleMetaInfo()
        current_people.id=str(people.id)
        current_people.label_id=people.label_id
        current_people.label_score=people.label_score
        current_people.handPosture=people.handPosture
        current_people.posture=people.posture
        current_people.distanceEval=people.distanceEval
        current_people.shirt_color_name=people.getMainColor(PersonMetaInfo.SHIRT_RECT)
        current_people.trouser_color_name=people.getMainColor(PersonMetaInfo.TROUSER_RECT)
        current_people_details=PeopleMetaInfoDetails()
        current_people_details.boundingBox.points=people.getBoundingBox(PersonMetaInfo.PERSON_RECT)
        current_people_details.shirtRect.points=people.getBoundingBox(PersonMetaInfo.SHIRT_RECT)
        current_people_details.shirtColorList=people.getColorList(PersonMetaInfo.SHIRT_RECT)
        current_people_details.trouserRect.points=people.getBoundingBox(PersonMetaInfo.TROUSER_RECT)
        current_people_details.trouserColorList=people.getColorList(PersonMetaInfo.TROUSER_RECT)
        current_people.details=current_people_details
        pt = Point32()
        pt.x = people.position_x
        pt.y = people.position_y
        current_people.point2Map=pt
        return current_people


def main():
    #""" main function
    #"""
    node = PeopleMngNode()

if __name__ == '__main__':
    main()