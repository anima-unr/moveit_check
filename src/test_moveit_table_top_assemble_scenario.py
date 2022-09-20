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
import threading as thread
from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String
from tf.transformations import *
from unr_object_manipulation.srv import *
from moveit_check.srv import *

sound_handle=SoundClient()

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
place_pose_cup = geometry_msgs.msg.Pose()

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)
# Pick and place enum
STATE = enum('APPROACHING', 'PICKING', 'PICKED', 'PLACING', 'PLACED', 'NEUTRAL', 'IDLE')

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
    self.work_thread = None
    self.stop = False
    
    

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

  def pick_place_top(self,req):

    self.state = STATE.NEUTRAL
    

    print "============ picking 1"
    obj = req.object
    # replaced_text = obj.replace('_', ' ')
    if req.object == "orange_top":
      sound_handle.say("Can you please bring me the orange top. It is out of my reach.","voice_cmu_us_clb_arctic_clunits")
    elif req.object == "purple_top":
      sound_handle.say("Can you please bring me the purple top. It is out of my reach.","voice_cmu_us_clb_arctic_clunits")
    rospy.sleep(1)
    if req.object == "orange_top" or req.object == "purple_top":
      
      self.object_pick_pose_target[req.object] = geometry_msgs.msg.Pose()
      self.object_pick_pose_target[req.object].position.x = 0.748416216519
      self.object_pick_pose_target[req.object].position.y = -0.286091846843
      self.object_pick_pose_target[req.object].position.z =  0.259872519598

      self.object_pick_pose_target[req.object].orientation.x = -0.534172962975
      self.object_pick_pose_target[req.object].orientation.y = -0.531391929358
      self.object_pick_pose_target[req.object].orientation.z = -0.556148738099
      self.object_pick_pose_target[req.object].orientation.w = -0.35068567714

    

      self.moveToPose(self.object_pick_pose_target[req.object])
      rospy.sleep(1)

      print "============ grasping open"

      self.gripper.command_position(100.0)
      self.state = STATE.APPROACHING

      rospy.sleep(1)
      self.state = STATE.PICKING
      print "============ grasping close"
      self.gripper.command_position(0.0)




  def pick_place_implementation(self,req):
    global place_pose_cup
    if req.object =="orange_top" or req.object == "purple_top":
      self.pick_place_top(req)
    else:
      print "============ picking 1"
      print req.object
      rospy.wait_for_service('object_position_service')
      self.state = STATE.NEUTRAL
      while 1:
        object_position_service = rospy.ServiceProxy('object_position_service', object_position_ar_tag)
        resp1 = object_position_service(req.object)
        if round(resp1.pose.position.x,2)!=0 and round(resp1.pose.position.y,2)!=0:
          break
      self.object_pick_pose_target[req.object] = resp1.pose
      pick_pose_offset = copy.deepcopy(self.object_pick_pose_target[req.object])
      pick_pose_offset.position.z = pick_pose_offset.position.z + 0.15
      print "pick_pose_offset"
      print pick_pose_offset
      rospy.sleep(3)
      self.moveToPose(pick_pose_offset)

      print "============ grasping open"

      self.gripper.command_position(100.0)
      self.state = STATE.APPROACHING
      rospy.sleep(3)
      


      rospy.sleep(3)

      print "+++++moving hand to find position"
      pick_pose_offset_copy = copy.deepcopy(pick_pose_offset)
      
      q_orig = [pick_pose_offset_copy.orientation.x,pick_pose_offset_copy.orientation.y,pick_pose_offset_copy.orientation.z,pick_pose_offset_copy.orientation.w]
      q_rot = quaternion_from_euler(-0.349066,0,0)
      q_new = quaternion_multiply(q_rot, q_orig)
      pick_pose_offset_copy.orientation.x = q_new[0]
      pick_pose_offset_copy.orientation.y = q_new[1]
      pick_pose_offset_copy.orientation.z = q_new[2]
      pick_pose_offset_copy.orientation.w = q_new[3]
      self.moveToPose(pick_pose_offset_copy)
      rospy.sleep(3)
      print "============ picking 2"
      print "pose_target using hand value"
      self.object_pick_pose_target_hand = copy.deepcopy(self.object_pick_pose_target[req.object])
      rospy.wait_for_service('object_position_hand_service')
      while 1:
          object_position_hand_service = rospy.ServiceProxy('object_position_hand_service', object_position_ar_tag)
          resp1 = object_position_hand_service(req.object)
          if round(resp1.pose.position.x,2)!=0 and round(resp1.pose.position.y,2)!=0:
            break
      
      self.object_pick_pose_target_hand.position = resp1.pose.position
      # pick_pose_ = copy.deepcopy(self.object_pick_pose_target[req.object])
      # pick_pose_.position.z = pick_pose_.position.z - 0.03
      print self.object_pick_pose_target_hand
      self.moveToPose(pick_pose_offset)
      rospy.sleep(3)
      self.state = STATE.PICKING
      response = self.moveToPose(self.object_pick_pose_target_hand)
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


    x = 0
    if req.object == "green_leg":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.580840699126
      self.object_place_pose_target[x].position.y = -0.400578428144
      self.object_place_pose_target[x].position.z =  -0.182307297867

      self.object_place_pose_target[x].orientation.x = 0.0724943511833
      self.object_place_pose_target[x].orientation.y = 0.996876649679
      self.object_place_pose_target[x].orientation.z = 0.00935206007186
      self.object_place_pose_target[x].orientation.w = 0.02990072481
      place_pose_cup = copy.deepcopy(self.object_place_pose_target[0])
    elif req.object == "blue_leg":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.558618033195
      self.object_place_pose_target[x].position.y = -0.726332804356
      self.object_place_pose_target[x].position.z =  -0.178524595463

      self.object_place_pose_target[x].orientation.x = 0.0581072914949
      self.object_place_pose_target[x].orientation.y = 0.994199128789
      self.object_place_pose_target[x].orientation.z = -0.0647094242811
      self.object_place_pose_target[x].orientation.w = 0.0632797392516
    elif req.object == "pink_bar":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.71442250824
      self.object_place_pose_target[x].position.y = -0.598826152346
      self.object_place_pose_target[x].position.z =  -0.151125640747

      self.object_place_pose_target[x].orientation.x = 0.741562899462
      self.object_place_pose_target[x].orientation.y = 0.663752680886
      self.object_place_pose_target[x].orientation.z = 0.0304777152692
      self.object_place_pose_target[x].orientation.w = 0.0926712125172
    elif req.object == "yellow_bar":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.395579434105
      self.object_place_pose_target[x].position.y = -0.555733126159
      self.object_place_pose_target[x].position.z =  -0.150796733874

      self.object_place_pose_target[x].orientation.x = 0.736718675006
      self.object_place_pose_target[x].orientation.y = 0.675945669702
      self.object_place_pose_target[x].orientation.z = -0.00257977650041
      self.object_place_pose_target[x].orientation.w = -0.0183409449839
    elif req.object == "orange_top":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.496101515387
      self.object_place_pose_target[x].position.y = -0.675208157198
      self.object_place_pose_target[x].position.z =  -0.0452322224107

      self.object_place_pose_target[x].orientation.x = -0.581787077524
      self.object_place_pose_target[x].orientation.y = -0.62259712887
      self.object_place_pose_target[x].orientation.z = -0.393417124504
      self.object_place_pose_target[x].orientation.w = -0.345137041908
    elif req.object == "purple_top":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.553719732797
      self.object_place_pose_target[x].position.y = -0.507609202771
      self.object_place_pose_target[x].position.z =  0.086544440762

      self.object_place_pose_target[x].orientation.x = -0.691971050838
      self.object_place_pose_target[x].orientation.y = -0.617515430259
      self.object_place_pose_target[x].orientation.z = -0.28748411639
      self.object_place_pose_target[x].orientation.w = -0.239172826671
    elif req.object == "Bread2":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.542516237619
      self.object_place_pose_target[x].position.y = -0.501256745496
      self.object_place_pose_target[x].position.z =  -0.208002193798

      self.object_place_pose_target[x].orientation.x = 0.0674411352316
      self.object_place_pose_target[x].orientation.y = 0.997425875999
      self.object_place_pose_target[x].orientation.z = -0.0176091896484
      self.object_place_pose_target[x].orientation.w = 0.0168294862179
    else:
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.679571021836
      self.object_place_pose_target[x].position.y = -0.522191374955
      self.object_place_pose_target[x].position.z =  -0.197022192408

      self.object_place_pose_target[x].orientation.x = 0.104515921529
      self.object_place_pose_target[x].orientation.y = -0.990668560413
      self.object_place_pose_target[x].orientation.z = -0.0735454054503
      self.object_place_pose_target[x].orientation.w = 0.0473634763548


    self.state = STATE.PLACING


    place_pose_offset = copy.deepcopy(self.object_place_pose_target[0])
      
    place_pose_offset.position.z = place_pose_offset.position.z + 0.25
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
    if req.object == "pink_bar" or req.object == "yellow_bar":
      rospy.sleep(3)

    # if req.object == "pink_bar" or req.object == "yellow_bar":
    #   self.object_place_pose_target[0].position.z = self.object_place_pose_target[0].position.z + 0.01
    #   response = self.moveToPose(self.object_place_pose_target[0])
    #   self.object_place_pose_target[0].position.y = self.object_place_pose_target[0].position.y + 0.04
    #   print "wiggle the bar"
    #   response = self.moveToPose(self.object_place_pose_target[0])
    #   self.object_place_pose_target[0].position.y = self.object_place_pose_target[0].position.y - 0.04
    #   print "wiggle the bar"
    #   response = self.moveToPose(self.object_place_pose_target[0])
    #   self.object_place_pose_target[0].position.y = self.object_place_pose_target[0].position.y + 0.04
    #   print "wiggle the bar"
    #   response = self.moveToPose(self.object_place_pose_target[0])
    #   self.gripper.command_position(100.0)
    #   rospy.sleep(3)
    #   response = self.moveToPose(place_pose_offset)
    if response == False:
      return False
    self.gripper.command_position(100.0)
    self.state = STATE.PLACED

        # print "============ grasping close"

        # self.gripper.command_position(0.0)

        # rospy.sleep(2)

    #self.marker_add(index,self.object_place_pose_target[index])


    print "place_pose_offset"
    print place_pose_offset
    print "++++++++++++ going back"



    self.moveToPose(place_pose_offset)
    self.state = STATE.IDLE

    
    

        #index = index + 1

        ## When finished shut down moveit_commander.
    # moveit_commander.roscpp_shutdown()

        ## END_TUTORIAL


    print "============ STOPPING"
    #return True

  def moveToPose(self,pose):

        # define temp pose
    pose_target = geometry_msgs.msg.Pose()

        # format the pose correctly

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
      print("\tExecuting...")
      self.group.go(wait=True)
      return True
    else:
      return False
    rospy.sleep(2)

  def PickAndPlaceObject(self, req):
    # starting a thread that will handle the pick and place.
    self.stop = True
    if self.work_thread != None and self.work_thread.is_alive():
      self.work_thread.join()
    self.stop = False
    self.work_thread = thread.Thread(target=self.pick_place_implementation, args=[req])
    self.work_thread.start()
    return pick_and_placeResponse(True)
  def PickAndPlaceCheck(self, req):
    print "here2"
    # checks to see if the pick and place is in the final placed state
    check = self.state == STATE.PLACED
    if check:
      self.state = STATE.IDLE
    if self.state == STATE.IDLE:
      return pick_and_placeResponse(True)
    else:
      return pick_and_placeResponse(False)

  def PickAndPlaceState(self, req):
    print "here3"
    # return the state of the pick and place
    return pick_and_place_stateResponse(self.state)
  def PickAndPlaceStop(self, req):
    print "here4"
    # Stop the arm from picking and placing
    if self.work_thread == None:
      return pick_and_place_stopResponse(False)
    self.stop = True
    self.work_thread.join()
    self.stop = False
    return pick_and_place_stopResponse(True)


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

  # sound_handle.say("Thank you. Please help me to assemble the table top with me.","voice_cmu_us_clb_arctic_clunits")

  s = rospy.Service('pick_and_place_object', pick_and_place, pickplace.PickAndPlaceObject)
  s_2 = rospy.Service('pick_and_place_check', pick_and_place, pickplace.PickAndPlaceCheck)

  s_3 = rospy.Service('pick_and_place_state', pick_and_place_state, pickplace.PickAndPlaceState)

  s_4 = rospy.Service('pick_and_place_stop', pick_and_place_stop, pickplace.PickAndPlaceStop)


  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

