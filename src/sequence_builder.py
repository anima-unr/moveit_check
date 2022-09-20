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
from sound_play.libsoundplay import SoundClient
## END_SUB_TUTORIAL

import baxter_interface

from std_msgs.msg import String
from tf.transformations import *
from unr_object_manipulation.srv import *
from moveit_check.srv import *

sound_handle=SoundClient()
object_init_pose= dict()
object_final_pose= dict()
object_list = ["blue_leg","green_leg","yellow_bar","pink_bar","orange_top","purple_top"] #cup =1, Tea=2, Sugar=3


def saving_init_position():
	for index_ in range(0,len(object_list)):
		rospy.wait_for_service('object_position_service')
		while 1:
				object_position_service = rospy.ServiceProxy('object_position_service', object_position_ar_tag)
				resp1 = object_position_service(object_list[index_])
				if round(resp1.pose.position.x,2)!=0 and round(resp1.pose.position.y,2)!=0:
					break
		object_init_pose[index_] = geometry_msgs.msg.Pose()
		object_init_pose[index_] = resp1.pose
		print(object_list[index_],object_init_pose[index_])

def tracking_object():
	global object_init_pose
	object_checked = [0,0,0,0,0,0,0]
	object_tracked = list()
	object_moved = 0
	seq_string = ['[']
	while object_moved<len(object_list):
		for index_ in range(0,len(object_list)):
			rospy.wait_for_service('object_position_service')
			while 1:
				object_position_service = rospy.ServiceProxy('object_position_service', object_position_ar_tag)
				resp1 = object_position_service(object_list[index_])
				# print("hey")
				if round(resp1.pose.position.x,2)!=0 and round(resp1.pose.position.y,2)!=0:
					break
				
				

			if (abs(resp1.pose.position.x - object_init_pose[index_].position.x)>0.05 or abs(resp1.pose.position.y - object_init_pose[index_].position.y)>0.05) and object_checked[index_] == 0:
				#print(round(resp1.pose.position.x,2),round(object_init_pose[index_].position.x,2),abs(resp1.pose.position.x - object_init_pose[index_].position.x))
				#print(object_list[index_])
				# object_tracked.append(object_list[index_])
				print(object_list[index_])
				sound_handle.say(object_list[index_])
				object_checked[index_] = 1;
				object_moved = object_moved + 1
				seq_string.append(str(index_+1))
	seq_string.append(']')
	print(seq_string)
	seq_string = ''.join(seq_string)
	return seq_string
	



def main():
  rospy.init_node('sequence_builder')
  
  val = raw_input("Are you ready to track the objects? Please type Yes if you are: ")

  if val=="Yes" or val=="yes":
  	print("Saving initial position")
  	saving_init_position()
  	print(object_init_pose)
  	print("Starting tracking object sequences")
  	seq_1 = tracking_object()
  	print(seq_1)
  	with open('sequence.txt', 'w') as f:
  		f.write(seq_1)
  	with open('sequence.txt', 'a') as f:
  		f.write("\n")
  val = raw_input("Are you ready to track the objects? Please type Yes if you are: ")
  if val=="Yes" or val=="yes":
  	print("Saving initial position")
  	saving_init_position()
  	print(object_init_pose)
  	print("Starting tracking object sequences")
  	seq_2 = tracking_object()
  	print(seq_2)
  	with open('sequence.txt', 'a') as f:
  		f.write(seq_2)




  
  
  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
