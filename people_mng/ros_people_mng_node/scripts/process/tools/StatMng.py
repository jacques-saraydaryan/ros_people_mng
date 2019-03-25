#!/usr/bin/env python
__author__ = 'Jacques Saraydaryan'

import matplotlib.pyplot as plt
import numpy as np
import os
import time
import rospy
import pandas as pd

class StatMng:

    stat_score_matrix = []
    FILE_PREFIX = "PEOPLE_TRACKED_STAT"
    start_time=0

    def __init__(self, save_folder):
        self.save_folder = save_folder
        self.start_time = time.time()


    def update_stat(self, action_type, tracked_id, general_score, face_score, shirt_color_score, trouser_color_score, pose_score):
        """
        Add stat of a current tracked people
        :param tracked_id: id of the current tracked people
        :param general_score: weigthed sum of score
        :param face_score: score relative to face information
        :param shirt_color_score: score relative to the shirt color information
        :param trouser_color_score: score relative to the trouser color information
        :param pose_score: score relative to pose information
        :param action_type: type of action CREATE vs UPDATE vs DELETE
        """
        time_elapsed = time.time() -self.start_time
        self.stat_score_matrix.append([time_elapsed, action_type, tracked_id,general_score,face_score,shirt_color_score,trouser_color_score,pose_score])

    def reset_stat(self):
        self.stat_score_matrix=[]

    def save_stat(self):
        file_to_save = self.adjustFileName(self.save_folder,self.FILE_PREFIX)
        rospy.loginfo("Save current stat to :" + file_to_save)
        np.save(file_to_save , self.stat_score_matrix)


    def adjustFileName(self, file_path, file_name_prefix):
        if os.path.isfile(file_path+'0-'+ file_name_prefix):
            index=1
            while os.path.isfile(file_path+str(index)+'-'+file_name_prefix):
                index=index+1
            return file_path+str(index)+'-'+ file_name_prefix
        else:
            return file_path+'0-'+ file_name_prefix



    def load_and_display(self,load_folder, file_name):
        data_matrix = []
        tracked_stat_map={}
        if os.path.isfile(load_folder + file_name):
            data_matrix.append(np.load(load_folder + file_name))
        else:
            rospy.logwarn("Unable to load stat file : "+ str(load_folder + file_name))
            return

        for row in data_matrix[0]:
            tracked_row = []
            # time
            tracked_row.append(float(row[0]))
            # action
            tracked_row.append(row[1])
            # score
            tracked_row.append(float(row[3]))
            # score face
            tracked_row.append(float(row[4]))
            # score shirt color
            tracked_row.append(float(row[5]))
            # score trouser color
            tracked_row.append(float(row[6]))
            # score pose
            tracked_row.append(float(row[7]))

            if tracked_stat_map.has_key(row[2]):
                tracked_stat_map[row[2]].append(tracked_row)
            else:
                tracked_stat_map[row[2]] = []
                tracked_stat_map[row[2]].append(tracked_row)

        fig, axs = plt.subplots(5, 1)
        #plt.subplot(311)
        rospy.loginfo(str(data_matrix))
        for key in tracked_stat_map:
            data_np=np.asarray(tracked_stat_map[key])
            axs[0].set_title('General Score')
            p=axs[0].plot(data_np[:,0], data_np[:,2], label=key)
            #axs[0].fill_between(data_np[:,0],0, data_np[:,2],where= data_np[:,2] >0, alpha=0.5)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[2]))
            axs[0].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

            #plt.subplot(312)
        for key in tracked_stat_map:
            data_np=np.asarray(tracked_stat_map[key])
            axs[1].set_title('Face Score')
            p =axs[1].plot(data_np[:,0], data_np[:,3], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[3]))
            axs[1].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

        for key in tracked_stat_map:
            data_np = np.asarray(tracked_stat_map[key])
            axs[2].set_title('Shirt Color Score')
            p =axs[2].plot(data_np[:, 0], data_np[:, 4], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[4]))
            axs[2].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

        for key in tracked_stat_map:
            data_np = np.asarray(tracked_stat_map[key])
            axs[3].set_title('Trouser Color Score')
            p =axs[3].plot(data_np[:, 0], data_np[:, 5], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[5]))
            axs[3].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

            #plt.subplot(313)
        for key in tracked_stat_map:
            data_np=np.asarray(tracked_stat_map[key])
            axs[4].set_title('Pose Score')
            p =axs[4].plot(data_np[:,0], data_np[:,6], label=key)
            tmp = []
            tmp2 = []
            for row in tracked_stat_map[key]:
                tmp.append(float(row[0]))
                tmp2.append(float(row[6]))
            axs[4].fill_between(tmp, 0, tmp2, alpha=0.5, color=p[0].get_color())

        plt.legend(loc='best')
        plt.show()


def main():
    #""" main function
    #"""

    load_folder = '/tmp/'
    file_name='0-PEOPLE_TRACKED_STAT.npy'

    statMng = StatMng(load_folder)
    statMng.load_and_display(load_folder,file_name)

if __name__ == '__main__':
    main()