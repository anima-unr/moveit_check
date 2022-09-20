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
from moveit_check.msg import *

flag = False
object_dict = {
  "Cup": 6,
  "Bread1": 2,
  "Bread2": 5,
  "Meat": 0,
  "Lettuce": 1,
  "Tea":7,
  "Sugar": 4
}
list_of_key = list(object_dict.keys())
list_of_value = list(object_dict.values())
object_position_list_ = list()
object_position_ = object_position()
object_pick_pose_target = dict()



def object_position_service_func(req):
  global object_pick_pose_target
  print(req,"hrello")
  print object_position_list_
  return object_position_list_









			
	
def ar_tag_callback(data):
  global flag,list_of_key,list_of_value,object_pick_pose_target
  place_pose_target = geometry_msgs.msg.Pose()
  
  # print len(data.markers)
  x = 0
  for index in range(0,len(data.markers)):
    for index_ in range(0,len(list_of_value)):
      if data.markers[index].id == list_of_value[index_]:
        # print list_of_key[index_]
        val_ = list_of_key[index_]
        q_orig = [data.markers[index].pose.pose.orientation.x,data.markers[index].pose.pose.orientation.y,data.markers[index].pose.pose.orientation.z,data.markers[index].pose.pose.orientation.w]
        q_rot = quaternion_from_euler(3.22886, 0, 0)
        q_new = quaternion_multiply(q_rot, q_orig)
        object_position_.pose =  data.markers[index].pose.pose
        object_position_.obj_name = val_
        object_position_list_.append(object_position_)
        print(object_position_list_)

        # object_pick_pose_target[val_] = geometry_msgs.msg.Pose()
        # object_pick_pose_target[val_].position.x = data.markers[index].pose.pose.position.x
        # object_pick_pose_target[val_].position.x = data.markers[index].pose.pose.position.x 
        # object_pick_pose_target[val_].position.y = data.markers[index].pose.pose.position.y 
        # object_pick_pose_target[val_].position.z =  data.markers[index].pose.pose.position.z 
        # object_pick_pose_target[val_].orientation.x = q_new[0]
        # object_pick_pose_target[val_].orientation.y = q_new[1]
        # object_pick_pose_target[val_].orientation.z = q_new[2]
        # object_pick_pose_target[val_].orientation.w = q_new[3]



def main():
  rospy.init_node('object_position_list_server')
  

  sub_img = rospy.Subscriber("ar_pose_marker_kinect", AlvarMarkers, ar_tag_callback)
  
  s = rospy.Service('object_position_list_service', object_position_ar_tag_list, object_position_service_func)
  
  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

