#!/usr/bin/env python

import rospy 
from sats_car_ros.msg import MasterCommand

def pubsub():
  rospy.init_node('masterpub',anonymous=True)
  rospy.Subscriber('/master',MasterCommand,master_callback)
  rospy.Publisher('/echo',MasterCommand,queue_size=1)

  rate = rospy.Rate(1) 
  mc = MasterCommand()
  while not rospy.is_shutdown():
      mc.master_on = not mc.master_on
      mc.update_param = not mc.update_param
def master_callback(data): 
  global master_on
  master_on = data.master_on
  rospy.loginfo(data)

############ Execution of main code ####################
if __name__=='__main__':
  try:
      pubsub()
  except rospy.ROSInterruptException:
    pass


