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

from process.DetectPeopleMeta import DetectPeopleMeta


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
        #TODO
        pass
    
    def detectPeopleMetaSrvCallback(self,req):
        self._detect_people_meta.processImg(req.img)
        #TODO
        #main,colorDlist=self.processImg(req.img)
        #return {'main_color':main,'main_colors':colorDlist}
        pass
        

    def processImg(self, img):
        #TODO
        pass


    def executePeopleMetaActionServer(self, goal):
        
        isActionSucceed=False
        action_result = ProcessPeopleFromImgResult()
        try:
        #TODO
        #    main,colorDlist =self.processImg(goal.img)
        #    action_result.main_color=main
        #    action_result.main_colors=colorDlist
            isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(action_result), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded(action_result)
        else:
            self._actionServer.set_aborted()
        pass




def main():
    #""" main function
    #"""
    node = PeopleMngNode()

if __name__ == '__main__':
    main()