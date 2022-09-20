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
## END_SUB_TUTORIAL

import baxter_interface

from std_msgs.msg import String
from tf.transformations import *
from unr_object_manipulation.srv import *
from moveit_check.srv import *

flag = False
object_dict = {
  "green_leg": 1,
  "yellow_bar": 3,
  "blue_leg": 2,
  "orange_top": 8,
  "purple_top": 7,
  "pink_bar": 4
}
list_of_key = list(object_dict.keys())
list_of_value = list(object_dict.values())

object_pick_pose_target_hand = dict()




def object_position_service_hand_func(req):
  global object_pick_pose_target_hand
  print(object_pick_pose_target_hand,"\n\n\n\n")
  return object_pick_pose_target_hand[req.object]





def ar_tag_callback_hand(data):
  global object_pick_pose_target_hand
  pick_post_target_hand = geometry_msgs.msg.Pose()
  for index in range(0,len(data.markers)):
    for index_ in range(0,len(list_of_value)):
      if data.markers[index].id == list_of_value[index_]:
        val_ = list_of_key[index_]
        object_pick_pose_target_hand[val_] = geometry_msgs.msg.Pose()
      # if data.markers[index].id == 11: #done
      #   object_pick_pose_target_hand[val_].position.x = data.markers[index].pose.pose.position.x
      #   object_pick_pose_target_hand[val_].position.y = data.markers[index].pose.pose.position.y
      #   object_pick_pose_target_hand[val_].position.z =  data.markers[index].pose.pose.position.z 
      # if data.markers[index].id == 10 or data.markers[index].id == 11: #done
        object_pick_pose_target_hand[val_].position.x = data.markers[index].pose.pose.position.x 
        object_pick_pose_target_hand[val_].position.x = data.markers[index].pose.pose.position.x + 0.05
        object_pick_pose_target_hand[val_].position.y = data.markers[index].pose.pose.position.y
        object_pick_pose_target_hand[val_].position.z =  data.markers[index].pose.pose.position.z - 0.07

			
	



def main():
  rospy.init_node('object_position_right_hand_server')
  print("============ Starting Service")

  
  sub_img_hand = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_tag_callback_hand)
  
  s_ = rospy.Service('object_position_hand_service', object_position_ar_tag, object_position_service_hand_func)
  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

