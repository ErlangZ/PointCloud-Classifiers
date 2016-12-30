#!/usr/bin/env python
# -*- coding: utf8 -*-
########################################################################
# 
# Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
# 
########################################################################
 
"""
File: merge_features.py
Author: erlangz(erlangz@baidu.com)
Date: 2016/12/30 11:42:05
"""
with open("./features--0") as f0, open("./features--1") as f1, open("./features--2") as f2, open("./features--3") as f3, open("./features--all", "w") as f:
    for lines in zip(f0.xreadlines(), f1.xreadlines(), f2.xreadlines(), f3.xreadlines()):
        lines = [l.strip() for l in lines]
        name, label, data0 = lines[0].split("\t", 2)
        _, _, data1 = lines[1].split("\t", 2)
        _, _, data2 = lines[2].split("\t", 2)
        _, _, data3 = lines[3].split("\t", 2)
        print >> f, "\t".join((name, label, data0, data1, data2, data3,))

with open("./features--all") as f:
    columns = len(f.readline().split("\t"))
    hog_feature_columns = (columns - 5)/3
    print "All Column: " + str(columns) + " HogFeaturesColumn:" + str(hog_feature_columns) + " BoundingBoxFeature:" + str(3)
