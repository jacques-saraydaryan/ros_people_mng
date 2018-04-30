#! /usr/bin/env python
__author__ ='Jacques Saraydaryan'
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from  ros_color_detection_actions.msg import DetectColorFromImgAction, DetectColorFromImgGoal
import actionlib


def LoadImgAndUseAction():
    rospy.init_node('color_detection_node_action_test', anonymous=False)
    _bridge = CvBridge()
    test_folder=rospy.get_param('imgtest_folder','/data')

    #Load Image
    #img_loaded1 = cv2.imread(test_folder+'/blue-short.jpg')
    img_loaded1 = cv2.imread(test_folder+'/top-purple.jpg')
    msg_im1 = _bridge.cv2_to_imgmsg(img_loaded1, encoding="bgr8")

    client = actionlib.SimpleActionClient('detect_color_action', DetectColorFromImgAction)

    client.wait_for_server()

    goal = DetectColorFromImgGoal(img=msg_im1)

    client.send_goal(goal)

    client.wait_for_result()

    content_result=client.get_result()

    result=content_result.main_color

    rospy.loginfo("color:%s, color_web:%s, color_temp:%s, color_brightness_name:%s, RGB:[%s,%s,%s], percentage:%s",str(result.color_name),str(result.color_web),str(result.color_temp),str(result.color_brightness_name),str(result.rgb[0]),str(result.rgb[1]),str(result.rgb[2]),str(result.percentage_of_img))
    rospy.loginfo("ALL MAIN COLORS")
    rospy.loginfo(content_result)


if __name__ == '__main__':
    try:
        LoadImgAndUseAction()
    except rospy.ROSInterruptException:
        pass