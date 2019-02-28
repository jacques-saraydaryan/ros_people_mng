#!/usr/bin/env python  
__author__ = 'Jacques Saraydaryan'

import rospy
from sensor_msgs.msg import Image
from ros_people_mng_msgs.msg import PeopleMetaInfoDetails,PeopleMetaInfo,PeopleMetaInfoList
from process.PeopleMetaTrackerMng import PeopleMetaTrackerMng


class PeopleMngNode:

    def __init__(self):
        rospy.init_node('people_mng_tracker', anonymous=False)

        self.tracker = PeopleMetaTrackerMng()

        # Subscribe to the image 
        self.sub_rgb = rospy.Subscriber("/people_meta_info", PeopleMetaInfoList, self.detect_people_meta_callback, queue_size=1)
        self.pub_people_meta_info = rospy.Publisher("/people_meta_info_tracked", PeopleMetaInfoList, queue_size=1)
        rospy.spin()


    def detect_people_meta_callback(self,req):
        #rospy.loginfo("--------------------> Get data into tracker")
        self.tracker.track_people(req.peopleList)
        #req.img --> image to display meta info


def main():
    #""" main function
    #"""
    node = PeopleMngNode()

if __name__ == '__main__':
    main()