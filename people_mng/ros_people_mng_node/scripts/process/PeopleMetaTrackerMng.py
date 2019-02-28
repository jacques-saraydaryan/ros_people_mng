#!/usr/bin/env python

__author__ = 'Jacques Saraydaryan'

import uuid
import time
import math
import rospy
from PeopleMetaSimilarity import PeopleMetaSimilarity
from TrackedPersonMetaInfo import TrackedPersonMetaInfo
from PersonMetaInfo import PersonMetaInfo
import threading


class PeopleMetaTrackerMng:
    SCORE_THRESHOLD = 0.4
    FORGETTING_PERIOD = 1
    isForgettingThreadEnd = False

    def __init__(self):
        self.trackedPeopleMap = {}
        self.forget_param_map = {}
        self.peopleSimilarity = PeopleMetaSimilarity()
        self.tracked_people_map_lock = threading.Lock()
        self.forgetting_thread = threading.Thread(target=self.forgetting_callback, args=(self.FORGETTING_PERIOD,))
        self.configure()

        self.forgetting_thread.start()

    def configure(self):
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_NONE]=(10,10)
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_OPERATOR] = (10, 10)
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_PERSON_OF_INTEREST] = (10, 10)

    def track_people(self, peopleList):
        max_score = 0
        max_id = ""
        for people in peopleList:
            (x, y, z) = self.get_people_pose(people.details.boundingBox.points)

            ## protected section on tracked people map
            self.tracked_people_map_lock.acquire()
            for tracked_people_key in self.trackedPeopleMap:

                score = self.peopleSimilarity.evaluate_people\
                        (
                            people,
                            self.trackedPeopleMap[tracked_people_key],
                            x, y, z
                        )
                if score > max_score:
                    max_score = score
                    max_id = tracked_people_key
            self.tracked_people_map_lock.release()
            ## End of protected section

            if max_score < self.SCORE_THRESHOLD:
                # According to the score update existing trackedPeople or create a new one
                new_tracked_people = TrackedPersonMetaInfo(
                    str(uuid.uuid1()),
                    people.label_id,
                    people.label_score,
                    people.details.boundingBox, people.details.shirtRect,
                    people.details.trouserRect, people.details.shirtColorList,
                    people.details.trouserColorList
                )
                new_tracked_people.setMainColor(PersonMetaInfo.SHIRT_RECT, people.shirt_color_name, (0, 0, 0))
                new_tracked_people.setMainColor(PersonMetaInfo.TROUSER_RECT, people.trouser_color_name, (0, 0, 0))
                new_tracked_people.setPose(x, y, z)
                self.addTrackedPeople(new_tracked_people)

            else:
                self.updateTrackedPeople(max_id,x,y,z)

    def addTrackedPeople(self, new_tracked_people):
        self.tracked_people_map_lock.acquire()
        self.trackedPeopleMap[new_tracked_people.id] = new_tracked_people
        self.tracked_people_map_lock.release()

    def updateTrackedPeople(self, id, x, y, z):
        self.tracked_people_map_lock.acquire()
        self.trackedPeopleMap[id].setPose(x, y, z)
        self.trackedPeopleMap[id].incWeight()
        self.tracked_people_map_lock.release()

    # def get_tracked_people(self, id):
    #     self.tracked_people_map_lock.acquire()
    #     current_tracked = self.trackedPeopleMap[id]
    #     self.tracked_people_map_lock.release()
    #     return current_tracked

    # def remove_tracked_people(self,id):
    #     self.tracked_people_map_lock.acquire()
    #     del self.trackedPeopleMap[id]
    #     self.tracked_people_map_lock.release()

    def get_people_pose(self, points):
        # get center of the bounding box
        x = 0
        y = 0
        z = 0

        x_center_box = (points[1].x - points[0].x) / 2 + points[0].x
        y_center_box = (points[1].y - points[0].y) / 2 + points[0].y
        # TODO compute the x ,y and z according to the computed distance relative to the current camera pose

        return x, y, z

    def forgetting_callback(self,period):

            while not self.isForgettingThreadEnd:
                self.tracked_people_map_lock.acquire()
                id_to_remove = []
                for tracked_people_key in self.trackedPeopleMap:
                    #get current tracked people
                    current_tracked_people = self.trackedPeopleMap[tracked_people_key]
                    #compute elasped time since last update
                    elapsed_time = time.time()-current_tracked_people.last_update_time
                    # get forget parameters according people role
                    (weigth_threshold,time_threshold) = self.forget_param_map[current_tracked_people.role]
                    # compute forget function
                    forget_result=  self.forget_function(elapsed_time,current_tracked_people.weight,weigth_threshold,time_threshold)

                    if forget_result < 0:
                        # remove current tracked people
                        id_to_remove.append(tracked_people_key)

                for id in id_to_remove:
                    try:
                        del self.trackedPeopleMap[str(id)]
                    except KeyError as e:
                        rospy.logwarn("unable to del tracked people e:"+str(e))

                self.tracked_people_map_lock.release()

                time.sleep(period)

    def forget_function(self,t,weight,weigth_threshold,time_threshold):
        value = ( weigth_threshold * -(1 / min(weigth_threshold, float(weight))) ) * t + time_threshold
        if value < 0:
            return -100
        else:
            return 1 + math.log( value )



