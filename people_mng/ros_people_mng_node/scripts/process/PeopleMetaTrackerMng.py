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
    SCORE_THRESHOLD = 0.2
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
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_NONE]=(10,30)
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_OPERATOR] = (10, 30)
        self.forget_param_map[TrackedPersonMetaInfo.ROLE_PERSON_OF_INTEREST] = (10, 30)

    def track_people(self, peopleList):
        max_score = [0,0,0,0,0]
        max_id = ""
        for people in peopleList:
            (x, y, z) = self.get_people_pose(people.point2Map.x,people.point2Map.y,0)

            ## protected section on tracked people map
            self.tracked_people_map_lock.acquire()
            for tracked_people_key in self.trackedPeopleMap:

                score = self.peopleSimilarity.evaluate_people\
                        (
                            people,
                            self.trackedPeopleMap[tracked_people_key],
                            x, y, z
                        )
                rospy.loginfo("---- CURRENT TOTAL SCORE:" + str(score) + ", People_label:" + people.label_id+", TRACKED PEOPLE ID:" + str(tracked_people_key))
                if score[0] > max_score[0]:
                    max_score = score
                    max_id = tracked_people_key
            self.tracked_people_map_lock.release()
            ## End of protected section

            rospy.loginfo("MAX SCORE:" + str(
                max_score) + ", People_label:" + people.label_id + ", TRACKED PEOPLE ID:" + str(max_id))
            if max_score[0] < self.SCORE_THRESHOLD:


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
                rospy.loginfo("CREATE NEW TRACKED PEOPLE:" + str(new_tracked_people.id))
                self.addTrackedPeople(new_tracked_people)

            else:
                rospy.loginfo("UPDATE EXISTING TRACKED PEOPLE:" + str(max_id))
                self.updateTrackedPeople(max_id,people,x,y,z,max_score)
        return self.getTrackedPeopleList()

    def addTrackedPeople(self, new_tracked_people):
        self.tracked_people_map_lock.acquire()
        self.trackedPeopleMap[new_tracked_people.id] = new_tracked_people
        self.tracked_people_map_lock.release()

    def updateTrackedPeople(self, id , people, x, y, z , score):
        self.tracked_people_map_lock.acquire()
        self.trackedPeopleMap[id].setPose(x, y, z)
        self.trackedPeopleMap[id].incWeight()
        self.trackedPeopleMap[id].setBoundingBox(PersonMetaInfo.PERSON_RECT,people.details.boundingBox)
        self.trackedPeopleMap[id].last_score=score[0]
        self.trackedPeopleMap[id].last_score_face=score[1]
        self.trackedPeopleMap[id].last_score_c_s=score[2]
        self.trackedPeopleMap[id].last_score_c_t=score[3]
        self.trackedPeopleMap[id].last_score_pose=score[4]
        self.tracked_people_map_lock.release()

    def getTrackedPeopleList(self):
        self.tracked_people_map_lock.acquire()
        result=self.trackedPeopleMap.values()
        self.tracked_people_map_lock.release()
        return result

    # def get_tracked_people(self, id):
    #     self.tracked_people_map_lock.acquire()
    #     current_tracked = self.trackedPeopleMap[id]
    #     self.tracked_people_map_lock.release()
    #     return current_tracked

    # def remove_tracked_people(self,id):
    #     self.tracked_people_map_lock.acquire()
    #     del self.trackedPeopleMap[id]
    #     self.tracked_people_map_lock.release()

    def get_people_pose(self, x,y,z):
        # get center of the bounding box
        r_x = x
        r_y = y
        r_z = z

        # TODO compute the transformation from camera frame to targeted frame (map)

        return r_x, r_y, r_z

    def stop_forgetting_function(self):
        self.isForgettingThreadEnd=True
        self.forgetting_thread.join(5)

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
                    self.trackedPeopleMap[tracked_people_key].ttl=forget_result
                    if forget_result <= 0:
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
        value = ( weigth_threshold * -(1 / min(float(weigth_threshold), float(weight))) ) * t + float(time_threshold)
        if value < 0:
            return 0
        else:
            return 1 + math.log( value )



