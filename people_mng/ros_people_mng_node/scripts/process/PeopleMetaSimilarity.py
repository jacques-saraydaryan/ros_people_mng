#!/usr/bin/env python

__author__ = 'Jacques Saraydaryan'
import math
import rospy
from PersonMetaInfo import PersonMetaInfo


class PeopleMetaSimilarity:
    WEIGHT_FACE_SCORE = 10
    WEIGHT_COLOR_SCORE = 3
    WEIGHT_POSE_SCORE = 5

    DISTANCE_FONCTION_THRESHOLD = 2

    def __init__(self):
        pass

    def evaluate_people(self, people, tracked_people, x, y, z):
        """
        Evaluate the score between current people and registered tracked people
        Score include Face score, color score (HSV) and pose likelihood
        :param people: current people
        :param tracked_people: registered tracked people
        :param x: current people coordinate x
        :param y: current people coordinate y
        :param z: current people coordinate z
        :return: global score of likelihood [0,1]
        """

        score_face = self.compute_face_score(people, tracked_people)

        score_color_shirt = self.compute_color_score(
            people.details.shirtColorList,
            tracked_people.getColorList(PersonMetaInfo.SHIRT_RECT)
        )

        score_color_trouser = self.compute_color_score(
            people.details.trouserColorList,
            tracked_people.getColorList(PersonMetaInfo.TROUSER_RECT)
        )

        score_pose = self.compute_pose_distance(people, tracked_people, x, y, z)

        rospy.loginfo("----SCORE:  Face:" + str(score_face) + ", COLOR SHIRT:" + str(score_color_shirt) + ", COLOR_TROUSER:" + str(score_color_trouser) + ", POSE:"+str(score_pose))
        final_score = (
                       self.WEIGHT_FACE_SCORE * score_face + self.WEIGHT_COLOR_SCORE * score_color_shirt +
                       self.WEIGHT_COLOR_SCORE * score_color_trouser + self.WEIGHT_POSE_SCORE * score_pose
               ) / (
                       self.WEIGHT_FACE_SCORE + self.WEIGHT_COLOR_SCORE * 2 + self.WEIGHT_POSE_SCORE
               )
        return final_score,score_face,score_color_shirt,score_color_trouser,score_pose

    def compute_face_score(self, people, tracked_people):
        """
        Compute Face score
        Currently only check if same id and return % of detected face
        :param people: current people
        :param tracked_people: registered tracked people
        :return: 0 is face label do not match, % of face detection otherwise
        """
        if people.label_id == tracked_people.label_id:
            if people.label_id != "None":
                rospy.loginfo("TRACKER: score to label:" + people.label_id)
            # if tracked_people.label_score < people.label_score:
            #    tracked_people.label_score=people.label_score
            return people.label_score
        return 0

    def compute_color_score(self, people_color_list, tracked_people_color_list):
        """
        Compute distance between 2 HSV color
        Need to take into account S And V ? to investigate
        :param people_color_list: current people color
        :param tracked_people_color_list: registred tracked people color
        :return: distance between color [0,1]
        """

        if len(people_color_list) == 0 or len(tracked_people_color_list) == 0:
            return 0

        people_hsv = self.get_max_hsv_color(people_color_list)
        tracked_people_hsv = self.get_max_hsv_color(tracked_people_color_list)

        # https://stackoverflow.com/questions/35113979/calculate-distance-between-colors-in-hsv-space
        dh = min(abs(people_hsv[0] - tracked_people_hsv[0]), 360 - abs(people_hsv[0] - tracked_people_hsv[0])) / float(
            360)
        # ds = abs(people_hsv[1] - tracked_people_hsv[1])/float(100)
        # dv = abs(people_hsv[2] - tracked_people_hsv[2]) / float(255)

        # distance = math.sqrt(dh * dh + ds * ds + dv * dv)
        return 1 - dh

    def get_max_hsv_color(self, colorList):
        """
        Retrun the max color of a given color list (from k-mean clustering)
        :param colorList: color list to compute
        :return: normaliszed HSV
        """
        hsv = []
        hsv_result = []
        max_value = 0.0
        for color in colorList:
            if color.percentage_of_img > max_value:
                max_value = color.percentage_of_img
                hsv = color.hsv
        # normalisation due to the encoding format
        hsv_result.append(hsv[0] * 360)
        hsv_result.append(hsv[1] * 100)
        hsv_result.append(hsv[2] * 100)
        return hsv_result

    def compute_pose_distance(self, people, tracked_people, x, y, z):
        """
        :param people: current people
        :param tracked_people: evaluated tracked people
        :param x: current people x
        :param y: current people y
        :param z: current people z
        :return: distance score
        """
        distance = math.sqrt(pow(tracked_people.current_x - x, 2) + pow(tracked_people.current_y - y, 2) + pow(
            tracked_people.current_z - z, 2))

        result = math.exp(distance * float(-1) / float(self.DISTANCE_FONCTION_THRESHOLD))

        # TODO to complete according to last registered pose and time elapsed,

        return result
