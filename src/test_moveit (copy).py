#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
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

flag = False
object_dict = {
  "Cup": 7,
  "Bread1": 11,
  "Bread2": 6,
  "Meat": 10,
  "TeaPot": 13
}
list_of_key = list(object_dict.keys())
list_of_value = list(object_dict.values())

class Pick_and_Place(object):


  def __init__(self, limb, side):

  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.


    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    self.group = moveit_commander.MoveGroupCommander("right_arm")
    self.group.set_max_velocity_scaling_factor(0.7);

    self.gripper = baxter_interface.Gripper("right")
    self.gripper.calibrate()
    self.gripper.set_holding_force(100.0)

    self.object_pick_pose_target = dict()
    self.object_place_pose_target = dict()
    self.object_pick_pose_target_hand = dict()
    self.marker = dict()
    

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.

    self.pub_x = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    print "============ Starting tutorial "

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % self.group.get_planning_frame()
    self.group.set_pose_reference_frame("pedestal")

    ## We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % self.group.get_end_effector_link()

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print self.robot.get_group_names()




  
  #def pick_place_implementation(self,pose_target):
  def pick_place_implementation(self,req):
    print "============ picking 1"
    # pose_target = geometry_msgs.msg.Pose()
    # #pose_target.orientation.w = 1.0
    # pose_target.position.x = 0.614834970443
    # pose_target.position.y = -0.273696490998
    # pose_target.position.z = -0.0880582686198

    # pose_target.orientation.x = 0.337657127707
    # pose_target.orientation.y = 0.94104972939
    # pose_target.orientation.z = -0.018643318663
    # pose_target.orientation.w = -0.00809305827207

    # print "adding box"
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = self.robot.get_planning_frame()
    # box_pose.pose.position.x = pose_target.position.x
    # box_pose.pose.position.y = pose_target.position.y
    # box_pose.pose.position.z = pose_target.position.z
    # # box_pose.pose.position.x = 0.137449324269
    # # box_pose.pose.position.y = 0.183503351278
    # # box_pose.pose.position.z = 1.16385190575
    # box_name = "box"
    # self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    #index = 0

    # while True:
    #   if index == 1:
    #     break
    #   else:
    print req.object
    pick_pose_offset = copy.deepcopy(self.object_pick_pose_target[req.object])
      
    pick_pose_offset.position.z = pick_pose_offset.position.z + 0.15
    print "pick_pose_offset"
    print pick_pose_offset
    rospy.sleep(3)

    self.moveToPose(pick_pose_offset)

    print "============ grasping open"

    self.gripper.command_position(100.0)

    rospy.sleep(3)

    print "============ picking 2"
    print "pose_target using hand value"
    print self.object_pick_pose_target_hand[req.object]

    response = self.moveToPose(self.object_pick_pose_target_hand[req.object])
    if response == False:
	return False

    print "============ grasping close"

    self.gripper.command_position(0.0)

    rospy.sleep(3)

    print "pick_pose_offset"
    print pick_pose_offset
    print "++++++++++++ going back"

    #self.marker_delete(index)

    response = self.moveToPose(pick_pose_offset)
    if response == False:
	return False

    print "============ placing 1"
        # place_pose_target = geometry_msgs.msg.Pose()
        # #pose_target.orientation.w = 1.0
        # place_pose_target.position.x = 0.654689623209
        # place_pose_target.position.y = -0.588702890558
        # place_pose_target.position.z = -0.166256389495

        # place_pose_target.orientation.x = 0.451894425385
        # place_pose_target.orientation.y = 0.891042661821
        # place_pose_target.orientation.z = -0.033779026544
        # place_pose_target.orientation.w = -0.0263321189116



    place_pose_offset = copy.deepcopy(self.object_place_pose_target[0])
      
    place_pose_offset.position.z = place_pose_offset.position.z + 0.2
    print "place_pose_offset"
    print place_pose_offset
    rospy.sleep(3)



    response = self.moveToPose(place_pose_offset)
    if response == False:
	return False

        #print "============ grasping open"

        #self.gripper.command_position(100.0)

        #rospy.sleep(2)

    print "============ placing 2"
    print "place_pose_target"
    print self.object_place_pose_target[0]

    response = self.moveToPose(self.object_place_pose_target[0])
    if response == False:
	return False
    self.gripper.command_position(100.0)

        # print "============ grasping close"

        # self.gripper.command_position(0.0)

        # rospy.sleep(2)

    #self.marker_add(index,self.object_place_pose_target[index])


    print "place_pose_offset"
    print place_pose_offset
    print "++++++++++++ going back"



    self.moveToPose(place_pose_offset)


        #index = index + 1

        ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

        ## END_TUTORIAL

    print "============ STOPPING"
    #return True

  def moveToPose(self,pose):

        # define temp pose
    pose_target = geometry_msgs.msg.Pose()

        # format the pose correctly
    #print "HELLOOOOOOOOOOO"
    #print pose
    pose_target.orientation.x = pose.orientation.x
    pose_target.orientation.y = pose.orientation.y
    pose_target.orientation.z = pose.orientation.z
    pose_target.orientation.w = pose.orientation.w
    pose_target.position.x = pose.position.x
    pose_target.position.y = pose.position.y
    pose_target.position.z = pose.position.z

        # set things
    self.group.set_pose_target(pose_target)
    self.group.set_num_planning_attempts(5);
    self.group.set_planning_time(10.0);
    self.group.set_goal_position_tolerance(0.0075)
    self.group.set_goal_orientation_tolerance(0.0075)

    print("\tPlanning...")
    plan1 = self.group.plan()
    if plan1.joint_trajectory.points:
        # rospy.sleep(5)
    	print("\tExecuting...")
    	self.group.go(wait=True)
	return True
    else:
	return False

  def input_pos(self,filename):
    index = 0



    with open(filename, 'r') as f:
      while True:
        if index == 1:
          break
        else:
        
          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          print data
          self.object_pick_pose_target[index] = geometry_msgs.msg.Pose()
          self.object_pick_pose_target[index].position.x = float(data[0])
          self.object_pick_pose_target[index].position.y = float(data[1])
          self.object_pick_pose_target[index].position.z = float(data[2])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          print data
          self.object_pick_pose_target[index].orientation.x = float(data[0])
          self.object_pick_pose_target[index].orientation.y = float(data[1])
          self.object_pick_pose_target[index].orientation.z = float(data[2])
          self.object_pick_pose_target[index].orientation.w = float(data[3])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')

          self.object_place_pose_target[index] = geometry_msgs.msg.Pose()
          self.object_place_pose_target[index].position.x = float(data[0])
          self.object_place_pose_target[index].position.y = float(data[1])
          self.object_place_pose_target[index].position.z = float(data[2])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          
          self.object_place_pose_target[index].orientation.x = float(data[0])
          self.object_place_pose_target[index].orientation.y = float(data[1])
          self.object_place_pose_target[index].orientation.z = float(data[2])
          self.object_place_pose_target[index].orientation.w = float(data[3])

          print self.object_pick_pose_target[index]
          print self.object_place_pose_target[index]

          index = index + 1

        # print "adding box"
    rospy.sleep(2)
    
    #rate.sleep()
        
    #self.vel = vel



    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = self.robot.get_planning_frame()
    # box_pose.pose.position.x = self.object_pick_pose_target[0].position.x 
    # box_pose.pose.position.y = self.object_pick_pose_target[0].position.y
    # box_pose.pose.position.z = self.object_pick_pose_target[0].position.z
    # # box_pose.pose.position.x = 0.137449324269
    # # box_pose.pose.position.y = 0.183503351278
    # # box_pose.pose.position.z = 1.16385190575
    # box_name = "box"
    # self.scene.add_box(box_name, box_pose, size=(0.08, 0.08, 0.08))

  def marker_add(self,index,pose_target):
 
    self.marker[index] = Marker()
    self.marker[index].header.frame_id = self.robot.get_planning_frame()
    self.marker[index].header.stamp = rospy.Time.now()
    self.marker[index].ns = "robot"
    self.marker[index].action = self.marker[index].ADD
    self.marker[index].type = self.marker[index].SPHERE
    self.marker[index].id = index

    self.marker[index].scale.x = 0.1
    self.marker[index].scale.y = 0.1
    self.marker[index].scale.z = 0.1
        
            #
        
    self.marker[index].color.a = 1.0
    self.marker[index].color.r = 0.0
    self.marker[index].color.g = 1.0 
    self.marker[index].color.b = 0.0

        # vel.pose.position.x = 1;
        # vel.pose.position.y = 1;
        # vel.pose.position.z = 1;
        # vel.pose.orientation.x = 0.0;
        # vel.pose.orientation.y = 0.0;
        # vel.pose.orientation.z = 0.0;
        # vel.pose.orientation.w = 1.0;

    self.marker[index].pose.position.x = pose_target.position.x
    self.marker[index].pose.position.y = pose_target.position.y
    self.marker[index].pose.position.z = pose_target.position.z
    self.marker[index].pose.orientation.x = pose_target.orientation.x 
    self.marker[index].pose.orientation.y = pose_target.orientation.y 
    self.marker[index].pose.orientation.z = pose_target.orientation.z 
    self.marker[index].pose.orientation.w = pose_target.orientation.w 

        



    

    
        #vel.text = "Hello"
    print "check\n\n\n\n\n\n\n"
      
    self.pub_x.publish(self.marker[index])
      #rate = rospy.Rate(1)
    # print "sending marker", self.marker[index]
        

  def marker_delete(self,index):
    self.marker[index].action = self.marker[index].DELETE
    self.pub_x.publish(self.marker[index])
	
  #def PickAndPlaceObject(self,

  def ar_tag_callback_hand(self,data):
	print "in ar_tag_callback_hand"
	pick_post_target_hand = geometry_msgs.msg.Pose()
	for index in range(0,len(data.markers)):
		for index_ in range(0,len(list_of_value)):
			val_ = list_of_key[index_]
			self.object_pick_pose_target_hand[val_] = geometry_msgs.msg.Pose()
			if data.markers[index].id == 11: #done
				self.object_pick_pose_target_hand[val_].position.x = data.markers[index].pose.pose.position.x 
            			self.object_pick_pose_target_hand[val_].position.y = data.markers[index].pose.pose.position.y 
            			self.object_pick_pose_target_hand[val_].position.z =  data.markers[index].pose.pose.position.z 

			
	
  def ar_tag_callback(self,data):
    global flag,list_of_key,list_of_value
   
    if flag == False:

      print "HERE"
      place_pose_target = geometry_msgs.msg.Pose()
      #pose_target.orientation.w = 1.0

      print len(data.markers) 

      # q_orig = quaternion_from_euler(0, 0, 0)

      x = 0
      for index in range(0,len(data.markers)):
      	
          # print data.markers[x].pose.pose.position.z
          
        for index_ in range(0,len(list_of_value)):
       
          if data.markers[index].id == list_of_value[index_]:
            print list_of_key[index_]
            val_ = list_of_key[index_]
   	    
            q_orig = [data.markers[index].pose.pose.orientation.x,data.markers[index].pose.pose.orientation.y,data.markers[index].pose.pose.orientation.z,data.markers[index].pose.pose.orientation.w]
          #q_rot = quaternion_from_euler(3.31613,0, 0.349066)
 	    q_rot = quaternion_from_euler(3.22886, 0, 0)
            q_new = quaternion_multiply(q_rot, q_orig)
	    # 10 - 0.079 0.046 0
	    #11 - 0.079 0.049 0.006 #orange part - 0.079 0.048 0.006
            
            print "here\n\n\n\n\n\n\n\n\n\n"
            self.object_pick_pose_target[val_] = geometry_msgs.msg.Pose()
          # print data.markers[x].pose.pose.position.z
	    if data.markers[index].id == 6:
		self.object_pick_pose_target[val_].position.x = data.markers[index].pose.pose.position.x - 0.055
            	self.object_pick_pose_target[val_].position.y = data.markers[index].pose.pose.position.y + 0.045
            	self.object_pick_pose_target[val_].position.z =  data.markers[index].pose.pose.position.z - 0.06

		self.object_pick_pose_target[val_].orientation.x = q_new[0]
            	self.object_pick_pose_target[val_].orientation.y = q_new[1]
            	self.object_pick_pose_target[val_].orientation.z = q_new[2]
            	self.object_pick_pose_target[val_].orientation.w = q_new[3]
		print self.object_pick_pose_target[val_]
	    if data.markers[index].id == 11: #done
		self.object_pick_pose_target[val_].position.x = data.markers[index].pose.pose.position.x - 0.055
            	self.object_pick_pose_target[val_].position.y = data.markers[index].pose.pose.position.y + 0.054
            	self.object_pick_pose_target[val_].position.z =  data.markers[index].pose.pose.position.z - 0.05

		self.object_pick_pose_target[val_].orientation.x = q_new[0]
            	self.object_pick_pose_target[val_].orientation.y = q_new[1]
            	self.object_pick_pose_target[val_].orientation.z = q_new[2]
            	self.object_pick_pose_target[val_].orientation.w = q_new[3]
		print self.object_pick_pose_target[val_]
	    if data.markers[index].id == 10: #meat
		self.object_pick_pose_target[val_].position.x = data.markers[index].pose.pose.position.x - 0.055
            	self.object_pick_pose_target[val_].position.y = data.markers[index].pose.pose.position.y + 0.044
            	self.object_pick_pose_target[val_].position.z =  data.markers[index].pose.pose.position.z - 0.06

		self.object_pick_pose_target[val_].orientation.x = q_new[0]
            	self.object_pick_pose_target[val_].orientation.y = q_new[1]
            	self.object_pick_pose_target[val_].orientation.z = q_new[2]
            	self.object_pick_pose_target[val_].orientation.w = q_new[3]
		print self.object_pick_pose_target[val_]
	    if data.markers[index].id == 13: 
		self.object_pick_pose_target[val_].position.x = data.markers[index].pose.pose.position.x - 0.079
            	self.object_pick_pose_target[val_].position.y = data.markers[index].pose.pose.position.y + 0.049
            	self.object_pick_pose_target[val_].position.z =  data.markers[index].pose.pose.position.z - 0.04

		self.object_pick_pose_target[val_].orientation.x = q_new[0]
            	self.object_pick_pose_target[val_].orientation.y = q_new[1]
            	self.object_pick_pose_target[val_].orientation.z = q_new[2]
            	self.object_pick_pose_target[val_].orientation.w = q_new[3]
		print self.object_pick_pose_target[val_]
	    else:
		val_x = 0.079
		val_y = 0.049
		val_z = - 0.05 
            

            

	self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
        self.object_place_pose_target[x].position.x = 0.679571021836
        self.object_place_pose_target[x].position.y = -0.522191374955
        self.object_place_pose_target[x].position.z =  -0.197022192408

        self.object_place_pose_target[x].orientation.x = 0.104515921529
        self.object_place_pose_target[x].orientation.y = -0.990668560413
        self.object_place_pose_target[x].orientation.z = -0.0735454054503
        self.object_place_pose_target[x].orientation.w = 0.0473634763548
          
    
      

   #    for index in range(0,len(data.markers)):
   #      if data.markers[index].id == 8:
	  # q_orig = [data.markers[index].pose.pose.orientation.x,data.markers[index].pose.pose.orientation.y,data.markers[index].pose.pose.orientation.z,data.markers[index].pose.pose.orientation.w]
   #        #q_rot = quaternion_from_euler(3.31613,0, 0.349066)
 	 #  q_rot = quaternion_from_euler(3.31613, 0, 0.349066)
   #        q_new = quaternion_multiply(q_rot, q_orig)
   #        print q_new
   #        # print data.markers[x].pose.pose.position.z
   #        self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
   #        self.object_place_pose_target[x].position.x = data.markers[index].pose.pose.position.x - 0.06
   #        self.object_place_pose_target[x].position.y = data.markers[index].pose.pose.position.y + 0.05
   #        self.object_place_pose_target[x].position.z =  data.markers[index].pose.pose.position.z - 0.03

   #        self.object_place_pose_target[x].orientation.x = q_new[0]
   #        self.object_place_pose_target[x].orientation.y = q_new[1]
   #        self.object_place_pose_target[x].orientation.z = q_new[2]
   #        self.object_place_pose_target[x].orientation.w = q_new[3]
          #x = x+1

      rospy.sleep(2)
      #x =0
      #self.object_place_pose_target[x] = self.object_place_pose_target[x]
      #print self.object_pick_pose_target["Cup"]
      #print self.object_place_pose_target[x]
      # for index in range(0, 1):
      #self.marker_add(0,self.object_pick_pose_target[0]) #sending 1 in case of pick position
      #self.pick_place_implementation()
      flag = True
    # print "adding box"
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = self.robot.get_planning_frame()
    # box_pose.pose.position.x = pose_target.position.x
    # box_pose.pose.position.y = pose_target.position.y
    # box_pose.pose.position.z = pose_target.position.z
    # # box_pose.pose.position.x = 0.137449324269
    # # box_pose.pose.position.y = 0.183503351278
    # # box_pose.pose.position.z = 1.16385190575
    # box_name = "box"
    # self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    # box_pose.header.frame_id = self.robot.get_planning_frame()
    # box_pose.pose.position.x = 0.505527106173
    # box_pose.pose.position.y = -0.512224242973
    # box_pose.pose.position.z = -0.162671165001
    # box_name = "boxx"
    # self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    # self.pick_place_implementation(pose_target)
    # print data.markers[1].id
    # print data.markers[0].pose.pose.position.x
    # print data.markers[1].pose.pose.position.x

def main():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  ## BAnima - enabling robot
  
  rs = baxter_interface.RobotEnable()
  rs.enable()
  pickplace = Pick_and_Place('left_arm','left')

  sub_img = rospy.Subscriber("ar_pose_marker_kinect", AlvarMarkers, pickplace.ar_tag_callback)
  sub_img_hand = rospy.Subscriber("ar_pose_marker", AlvarMarkers, pickplace.ar_tag_callback_hand)
  print "Hello"
  s = rospy.Service('pick_and_place_object', pick_and_place, pickplace.pick_place_implementation)
  # pickplace.input_pos("position_record.txt")
  # pickplace.marker_add(index,pickplace.object_pick_pose_target[0])
  # for index in range(0, 1):
  #   pickplace.marker_add(index,pickplace.object_pick_pose_target[index]) #sending 1 in case of pick position
  # pickplace.pick_place_implementation()

  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

