#!/usr/bin/env python
import os
from os import listdir
from os.path import isfile, isdir, join

class CreateFile(object):
    
    def __init__(self, folder):
        self.folder = folder
    def getPath(self):
        items = listdir(self.folder)
        cnts = []
        for item in items:
            path = os.path.splitext(item)[0]
            cnt = int(path.replace('image', ''))
            if cnt:
                cnts.append(cnt)
            cnts = sorted(cnts)
        cnt = cnts[-1]
        if not cnt:
           cnt = 1 
        return join(self.folder, ('image%s.jpg' % str(cnt).zfill(4)))
    
if __name__ == '__main__':
    cf = CreateFile("/Users/grant/Tello/Images")
    print(cf.getPath())
