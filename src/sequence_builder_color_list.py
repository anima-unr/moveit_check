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
from moveit_check.msg import colorPos2d
from moveit_check.msg import color_pos_2d_array

sound_handle=SoundClient()
object_init_pos= dict()
object_final_pos= dict()
object_list = ["blue_leg","green_leg","yellow_bar","pink_bar","orange_top","purple_top"] #cup =1, Tea=2, Sugar=3

class obj_pos:
    x = 0
    y = 0





def saving_init_position():
	for index_ in range(0,len(object_list)):
		rospy.wait_for_service('object_position_service_color')

		object_position_service_color = rospy.ServiceProxy('object_position_service_color', object_position_color)


		resp1 = object_position_service_color(object_list[index_])

		object_init_pos[index_] = obj_pos()
		object_init_pos[index_].x = resp1.x
		object_init_pos[index_].y = resp1.y
		print(object_list[index_],object_init_pos[index_].x,object_init_pos[index_].y)

def tracking_object():
	global object_init_pos
	object_checked = [0,0,0,0,0,0,0]
	object_tracked = list()
	object_moved = 0
	# seq_string = ['[']
	obj_seq = []

	print("hey*****************\n")
	while object_moved<len(object_list):
		for index_ in range(0,len(object_list)):
			rospy.wait_for_service('object_position_service_color')

			object_position_service_color = rospy.ServiceProxy('object_position_service_color', object_position_color)

			resp1 = object_position_service_color(object_list[index_])

				
			# print("hey*****************\n")

			if (abs(resp1.y - object_init_pos[index_].y)>180) and object_checked[index_] == 0:

				print(object_list[index_],resp1.y,resp1.x)
				# sound_handle.say(object_list[index_])
				object_checked[index_] = 1;
				object_moved = object_moved + 1
				obj_seq.append(index_)
				# seq_string.append(str(index_))
	# seq_string.append(']')
	print(obj_seq)
	# seq_string = ''.join(seq_string)
	return obj_seq
	# return seq_string
	



def main():
  rospy.init_node('sequence_builder_color')



  val = raw_input("Are you ready to provide good task sequence? Please type Yes if you are: ")

  if val=="Yes" or val=="yes":
  	print("Saving initial position")
  	saving_init_position()

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
