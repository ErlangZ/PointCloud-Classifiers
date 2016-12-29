#!/usr/bin/env python
# -*- coding: utf8 -*-
########################################################################
# 
# Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
# 
########################################################################
 
"""
File: read.py
Author: erlangz(erlangz@baidu.com)
Date: 2016/12/29 17:43:23
"""
with open("../data/origin_data/features-88") as f:
    for l in f.xreadlines():
        print len(l.split("\t"))
        break
