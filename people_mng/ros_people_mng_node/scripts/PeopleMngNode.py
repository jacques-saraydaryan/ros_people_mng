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

from ros_people_mng_msgs.msg import PeopleMetaInfo,PeopleMetaInfoList
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction,ProcessPeopleFromImgResult
from ros_people_mng_srvs.srv import ProcessPeopleFromImg

from process.DetectPeopleMeta import DetectPeopleMeta,PersonMetaInfo


class PeopleMngNode():


    def __init__(self):
        rospy.init_node('people_mng_node', anonymous=False)
        self.configure()

        # Subscribe to the image 
        self.sub_rgb = rospy.Subscriber("/image", Image, self.rgb_callback, queue_size=1)
        self.pub_people_meta_info = rospy.Publisher("/people_meta_info", PeopleMetaInfoList, queue_size=1)

        #declare ros service 
        self.detectPeopleMetaSrv = rospy.Service('detect_people_meta_srv', ProcessPeopleFromImg, self.detectPeopleMetaSrvCallback)

         # create action server and start it
        self._actionServer = actionlib.SimpleActionServer('detect_people_meta_action', ProcessPeopleFromImgAction, self.executePeopleMetaActionServer, False)
        self._actionServer.start()

        rospy.spin()


    def configure(self):
        self._bridge = CvBridge()
        self._detect_people_meta=DetectPeopleMeta()
        


    def rgb_callback(self, data):
        #FIXME need to protect to avoid concurrency?
        self.current_img=data
        
    
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
            image_to_process=self.current_img
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
        pass


    def convertPeoplToRosMsg(self,people):
        current_people=PeopleMetaInfo()
        current_people.id=str(people.id)
        current_people.label_id=people.label_id
        current_people.handCall=people.handCall
        current_people.posture=people.posture
        current_people.distanceEval=people.distanceEval
        current_people.shirt_color_name=people.getMainColor(PersonMetaInfo.SHIRT_RECT)
        current_people.trouser_color_name=people.getMainColor(PersonMetaInfo.TROUSER_RECT)
        return current_people


def main():
    #""" main function
    #"""
    node = PeopleMngNode()

if __name__ == '__main__':
    main()