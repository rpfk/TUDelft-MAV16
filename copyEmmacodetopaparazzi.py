# -*- coding: utf-8 -*-
"""
Created on Wed Mar  2 21:41:28 2016

@author: wplaetinck
"""

import shutil as s
import os
import sys

pathname = os.path.dirname(sys.argv[0])
print 'path =', pathname

basepath = pathname.strip("TUDelft-MAV16")
print 'basepath =', basepath

# Add files here
files = [["TUDelft-MAV16/Emma conf and modules/emma_airframe.xml","paparazzi/conf/airframes/TUDELFT/emma_airframe.xml"],
         ["TUDelft-MAV16/Emma conf and modules/tudelft_course2016_avoid_orange_cyberzoo.xml","paparazzi/conf/flight_plans/TUDELFT/tudelft_course2016_avoid_orange_cyberzoo.xml"],
         ["TUDelft-MAV16/Emma conf and modules/Emma 1/emma1.c","paparazzi/sw/airborne/modules/emma1/emma1.c"],
         ["TUDelft-MAV16/Emma conf and modules/Emma 1/emma1.h","paparazzi/sw/airborne/modules/emma1/emma1.h"],
         ["TUDelft-MAV16/Emma conf and modules/Emma 1/emma1.xml","paparazzi/conf/modules/emma1.xml"]]

print 
switch = input("""Which direction to copy 
1 To Github, 2 To Paparazzi:    """)

for f in files:
    
    if switch == 1:
        print "Copy from Paparazzi to Github"        
        s.copyfile(basepath+f[1],basepath+f[0])

    if switch == 2:
        print "Copy from Github to Paparazzi"
        s.copyfile(basepath+f[0],basepath+f[1])
        



