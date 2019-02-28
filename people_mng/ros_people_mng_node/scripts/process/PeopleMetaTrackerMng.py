#!/usr/bin/env python

__author__ = 'Jacques Saraydaryan'

import uuid

from PeopleMetaSimilarity import PeopleMetaSimilarity
from TrackedPersonMetaInfo import TrackedPersonMetaInfo
from PersonMetaInfo import PersonMetaInfo


class PeopleMetaTrackerMng:
    SCORE_THRESHOLD = 0.4

    def __init__(self):
        self.trackedPeopleMap={}
        self.peopleSimilarity=PeopleMetaSimilarity()


    def track_people(self,peopleList):
        max_score = 0
        max_id = ""
        for people in peopleList:
            (x,y,z)=self.get_people_pose(people.details.boundingBox.points)
            for tracked_people_key in self.trackedPeopleMap:

                score = self.peopleSimilarity.evaluate_people(people, self.trackedPeopleMap[tracked_people_key], x,y,z)

                if score > max_score:
                    max_score = score
                    max_id = tracked_people_key

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
                new_tracked_people.setMainColor(PersonMetaInfo.SHIRT_RECT, people.shirt_color_name,(0,0,0))
                new_tracked_people.setMainColor(PersonMetaInfo.TROUSER_RECT, people.trouser_color_name, (0, 0, 0))
                new_tracked_people.setPose(x,y,z)
                self.trackedPeopleMap[new_tracked_people.id] = new_tracked_people

            else:
                # TODO
                self.trackedPeopleMap[max_id].update()


    def get_people_pose(self,points):
        # get center of the bounding box
        x = 0
        y = 0
        z = 0

        x_center_box=(points[1].x-points[0].x)/2 + points[0].x
        y_center_box=(points[1].y-points[0].y)/2 + points[0].y
        #TODO compute the x ,y and z according to the computed distance relative to the current camera pose

        return x,y,z