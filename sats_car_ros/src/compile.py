#!/usr/bin/env python

import py_compile
import os
import stat

import rospkg

r = rospkg.RosPack()
path = r.get_path('sats_car_ros')
#print path
def cmp_permissions(x):
  py_compile.compile(x)
  st = os.stat(x+'c')
  os.chmod(x+'c',st.st_mode | stat.S_IEXEC)
cmp_permissions(path+'/src/'+'controller.py')
cmp_permissions(path+'/src/'+'sensor_hub.py')
