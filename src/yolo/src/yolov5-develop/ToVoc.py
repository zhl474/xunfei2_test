# -*- coding: utf-8 -*-
"""
Created on Thu Jun 18 13:49:59 2020

@author: xxf
"""
import os
import numpy as np
import codecs
import json
from glob import glob
import shutil
from sklearn.model_selection import train_test_split

saved_path = "D:\\安装包\\playezio-darknet-master\\darknet\\build\\darknet\\x64\\data\\TestData\\"  # 保存路径

#创建要求文件夹
if not os.path.exists(saved_path + "Annotations"):
    os.makedirs(saved_path + "Annotations")
if not os.path.exists(saved_path + "JPEGImages/"):
    os.makedirs(saved_path + "JPEGImages/")
if not os.path.exists(saved_path + "ImageSets/Main/"):
    os.makedirs(saved_path + "ImageSets/Main/")


# 6.split files for txt
txtsavepath = saved_path + "ImageSets/Main/"
ftrainval = open(txtsavepath + '/trainval.txt', 'w')
ftest = open(txtsavepath + '/test.txt', 'w')
ftrain = open(txtsavepath + '/train.txt', 'w')
fval = open(txtsavepath + '/val.txt', 'w')
total_files = glob("D:\\安装包\\playezio-darknet-master\\darknet\\build\\darknet\\x64\\data\\TestData\\JPEGImages\\*[jpg.jpeg,png]")
#total_files = glob("D:\\安装包\\playezio-darknet-master\\darknet\\build\\darknet\\x64\\data\\TestData\JPEGImages\\*.jpeg")
#total_files = glob("D:\\安装包\\playezio-darknet-master\\darknet\\build\\darknet\\x64\\data\\TestData\JPEGImages\\*.png")
print(total_files)
#total_files = [i.split("/")[-1].split(".jpg") for i in total_files]
#print(total_files)
test_filepath = ""
for file in total_files:
    ftrainval.write(file + "\n")
# test
# for file in os.listdir(test_filepath):
#    ftest.write(file.split(".jpg")[0] + "\n")
# split
train_files, val_files = train_test_split(total_files, test_size=0.15, random_state=42)
# train
for file in train_files:
    ftrain.write(file + "\n")
# val
for file in val_files:
    fval.write(file + "\n")

ftrainval.close()
ftrain.close()
fval.close()
