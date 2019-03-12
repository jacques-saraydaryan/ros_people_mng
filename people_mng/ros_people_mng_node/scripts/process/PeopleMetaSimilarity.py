#!/usr/bin/env python

__author__ = 'Jacques Saraydaryan'

import math
import rospy
from PersonMetaInfo import PersonMetaInfo
from process_score.face_score.SimpleFaceScore import SimpleFaceScore
from process_score.face_score.MemoryFaceScore import MemoryFaceScore

from process_score.color_score.SimpleColorScore import SimpleColorScore
from process_score.color_score.AverageColorScore import AverageColorScore
from process_score.pose_score.SimplePoseScore import SimplePoseScore



class PeopleMetaSimilarity:
    PROCESS_FACE_SIMPLE_SCORE="PROCESS_FACE_SIMPLE_SCORE"
    PROCESS_FACE_MEMORY_SCORE = "PROCESS_FACE_MEMORY_SCORE"

    PROCESS_COLOR_SIMPLE_SCORE = "PROCESS_COLOR_SIMPLE_SCORE"
    PROCESS_COLOR_AVG_SCORE = "PROCESS_COLOR_AVG_SCORE"

    PROCESS_POSE_SIMPLE_SCORE = "PROCESS_POSE_SIMPLE_SCORE"

    current_face_score = "PROCESS_FACE_MEMORY_SCORE"
    current_color_score = "PROCESS_COLOR_AVG_SCORE"
    current_pose_score = "PROCESS_POSE_SIMPLE_SCORE"

    WEIGHT_FACE_SCORE = 10
    WEIGHT_COLOR_SCORE = 3
    WEIGHT_POSE_SCORE = 5

    def __init__(self):
        self.configure()
        rospy.loginfo("TRACKER SIMILARITY---- FACE-SCORE:"+self.current_face_score)
        rospy.loginfo("TRACKER SIMILARITY---- COLOR-SCORE:" + self.current_color_score)
        rospy.loginfo("TRACKER SIMILARITY---- POSE-SCORE:" + self.current_pose_score)


    def configure(self):
        self.processScoreMap={}
        self.processScoreMap[self.PROCESS_FACE_SIMPLE_SCORE]=SimpleFaceScore()
        self.processScoreMap[self.PROCESS_FACE_MEMORY_SCORE] = MemoryFaceScore()
        self.processScoreMap[self.PROCESS_COLOR_SIMPLE_SCORE] = SimpleColorScore()
        self.processScoreMap[self.PROCESS_COLOR_AVG_SCORE] = AverageColorScore()
        self.processScoreMap[self.PROCESS_POSE_SIMPLE_SCORE] = SimplePoseScore()

    def evaluate_people(self, people, tracked_people, new_pose):
        """
        Evaluate the score between current people and registered tracked people
        Score include Face score, color score (HSV) and pose likelihood
        :param people: current people
        :param tracked_people: registered tracked people
        :param new_pose: current people pose
        :return: global score of likelihood [0,1]
        """

        score_face = self.processScoreMap[self.current_face_score].face_score(people, tracked_people)
        score_color_shirt = self.processScoreMap[self.current_color_score].color_score(people.details.shirtColorList,tracked_people, PersonMetaInfo.SHIRT_RECT)
        score_color_trouser = self.processScoreMap[self.current_color_score].color_score(people.details.trouserColorList, tracked_people, PersonMetaInfo.TROUSER_RECT)
        score_pose = self.processScoreMap[self.current_pose_score].pose_score(people, tracked_people, new_pose)

        rospy.logdebug("----SCORE:  Face:" + str(score_face) + ", COLOR SHIRT:" + str(score_color_shirt) + ", COLOR_TROUSER:" + str(score_color_trouser) + ", POSE:"+str(score_pose))
        final_score = (
                       self.WEIGHT_FACE_SCORE * score_face + self.WEIGHT_COLOR_SCORE * score_color_shirt +
                       self.WEIGHT_COLOR_SCORE * score_color_trouser + self.WEIGHT_POSE_SCORE * score_pose
               ) / (
                       self.WEIGHT_FACE_SCORE + self.WEIGHT_COLOR_SCORE * 2 + self.WEIGHT_POSE_SCORE
               )

        return final_score,score_face,score_color_shirt,score_color_trouser,score_pose


    def update_process_score(self, tracked_people_id_to_remove_list):
        self.processScoreMap[self.current_face_score].update_face_score_content(tracked_people_id_to_remove_list)
        self.processScoreMap[self.current_color_score].update_color_score_content(tracked_people_id_to_remove_list)
        self.processScoreMap[self.current_pose_score].update_pose_score_content()


    def update_tracked_people(self,people, tracked_people):
        self.processScoreMap[self.current_face_score].add_face_to_tracked(people, tracked_people)
        self.processScoreMap[self.current_color_score].add_color_to_tracked(
                                                                        people.details.shirtColorList,
                                                                        tracked_people, PersonMetaInfo.SHIRT_RECT
        )
        self.processScoreMap[self.current_color_score].add_color_to_tracked(
                                                                        people.details.trouserColorList,
                                                                        tracked_people, PersonMetaInfo.TROUSER_RECT
        )
