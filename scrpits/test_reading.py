#!/usr/bin/env python
# coding:utf-8
import numpy
def readTrajFromFile():
    with open('../trajectory/12.st', 'r') as f:
        way_point_list = f.readlines
        result = []
        for line in f:
            if '\xef\xbb\xbf' in line:
                str1 = line.replace('\xef\xbb\xbf','')
                result.append(map(float, str1.split(' ')))
            else:
                result.append(map(float, line.split(' ')))
        print (result)
if __name__ == '__main__':
