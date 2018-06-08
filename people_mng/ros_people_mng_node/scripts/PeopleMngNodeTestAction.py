#! /usr/bin/env python
__author__ ='Jacques Saraydaryan'
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from  ros_people_mng_actions.msg import ProcessPeopleFromImgAction, ProcessPeopleFromImgGoal
import actionlib


def LoadImgAndUseAction():
    rospy.init_node('people_mng_node_action_test', anonymous=False)
    _bridge = CvBridge()
    test_folder=rospy.get_param('imgtest_folder','../data')

    #Load Image
    img_loaded1 = cv2.imread(test_folder+'/group-in-line.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/group-diff-position.jpg')
    #img_loaded1 = cv2.imread(test_folder+'/couple_raph.jpg')

    
    msg_im1 = _bridge.cv2_to_imgmsg(img_loaded1, encoding="bgr8")   
#
    client = actionlib.SimpleActionClient('detect_people_meta_action', ProcessPeopleFromImgAction)
#
    client.wait_for_server()
#
    goal = ProcessPeopleFromImgGoal(img=msg_im1)
#
    client.send_goal(goal)
#
    client.wait_for_result()
#
    content_result=client.get_result()
    rospy.loginfo(content_result)


    goal = ProcessPeopleFromImgGoal()

    client.send_goal(goal)

    client.wait_for_result()

    content_result=client.get_result()
    rospy.loginfo(content_result)


if __name__ == '__main__':
    try:
        LoadImgAndUseAction()
    except rospy.ROSInterruptException:
        pass