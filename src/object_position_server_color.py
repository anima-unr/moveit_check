#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
import rosnode
## END_SUB_TUTORIAL

import baxter_interface

from std_msgs.msg import String
from tf.transformations import *
from unr_object_manipulation.srv import *
from moveit_check.srv import *
from moveit_check.msg import colorPos2d
from moveit_check.msg import color_pos_2d_array


object_pos_list = dict()

class obj_pos:
    x = 0
    y = 0



def object_position_service_func(req):

  print(object_pos_list[req.object].x,object_pos_list[req.object].y)
  return object_pos_list[req.object].x,object_pos_list[req.object].y









			
	
def color_list_callback(data):



  for index in range(0,len(data.color_pos_2d_list)):
    val_ = data.color_pos_2d_list[index].object_name 
    object_pos_list[val_] = obj_pos()
    object_pos_list[val_].x = data.color_pos_2d_list[index].x 
    object_pos_list[val_].y = data.color_pos_2d_list[index].y






def main():
  rospy.init_node('object_position_server_color')
  print "============ Starting Service"

  sub_img = rospy.Subscriber("color_list", color_pos_2d_array, color_list_callback)
  
  s = rospy.Service('object_position_service_color', object_position_color, object_position_service_func)
  
  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

