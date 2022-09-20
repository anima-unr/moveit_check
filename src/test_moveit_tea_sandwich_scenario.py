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

from std_msgs.msg import String
from tf.transformations import *
from unr_object_manipulation.srv import *
from moveit_check.srv import *


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


  def pick_place_implementation(self,req):
    global place_pose_cup
    print "============ picking 1"
    print req.object
    rospy.wait_for_service('object_position_service')
    self.state = STATE.NEUTRAL
    while 1:
      object_position_service = rospy.ServiceProxy('object_position_service', object_position_ar_tag)
      resp1 = object_position_service(req.object)
      if resp1.pose.position.x !=0 and resp1.pose.position.y !=0 and resp1.pose.position.z !=0:
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
        if resp1.pose.position.x !=0 and resp1.pose.position.y !=0 and resp1.pose.position.z !=0:
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
    if req.object == "Cup":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.574613148568
      self.object_place_pose_target[x].position.y = -0.708285889613
      self.object_place_pose_target[x].position.z =  -0.21847029184

      self.object_place_pose_target[x].orientation.x = 0.0946031211746
      self.object_place_pose_target[x].orientation.y = 0.993876460202
      self.object_place_pose_target[x].orientation.z = -0.056833892829
      self.object_place_pose_target[x].orientation.w = -0.00545343433902
      place_pose_cup = copy.deepcopy(self.object_place_pose_target[0])
    elif req.object == "Sugar":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.668722989827
      self.object_place_pose_target[x].position.y = -0.653921094692
      self.object_place_pose_target[x].position.z =  -0.195452017988

      self.object_place_pose_target[x].orientation.x = 0.243310548412
      self.object_place_pose_target[x].orientation.y = 0.964882539099
      self.object_place_pose_target[x].orientation.z = -0.0783193244568
      self.object_place_pose_target[x].orientation.w = 0.060561920302
    elif req.object == "Tea":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.58067400515
      self.object_place_pose_target[x].position.y = -0.61249686825
      self.object_place_pose_target[x].position.z =  -0.183082934782

      self.object_place_pose_target[x].orientation.x = 0.103390108904
      self.object_place_pose_target[x].orientation.y = 0.994421057905
      self.object_place_pose_target[x].orientation.z = -0.0125948441152
      self.object_place_pose_target[x].orientation.w = -0.0166917607825
    elif req.object == "Meat":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.54086963281
      self.object_place_pose_target[x].position.y = -0.508704546442
      self.object_place_pose_target[x].position.z =  -0.228002193798

      self.object_place_pose_target[x].orientation.x = 0.0937574076733
      self.object_place_pose_target[x].orientation.y = 0.994525764983
      self.object_place_pose_target[x].orientation.z = -0.0457005175148
      self.object_place_pose_target[x].orientation.w = 0.00628601541736
    elif req.object == "Lettuce":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.540388935625
      self.object_place_pose_target[x].position.y = -0.499655203118
      self.object_place_pose_target[x].position.z =  -0.228002193798

      self.object_place_pose_target[x].orientation.x = 0.155867191113
      self.object_place_pose_target[x].orientation.y = 0.987412604871
      self.object_place_pose_target[x].orientation.z = -0.0109181733225
      self.object_place_pose_target[x].orientation.w = 0.0245470969381
    elif req.object == "Bread1":
      self.object_place_pose_target[x] = geometry_msgs.msg.Pose()
      self.object_place_pose_target[x].position.x = 0.540388935625
      self.object_place_pose_target[x].position.y = -0.499655203118
      self.object_place_pose_target[x].position.z =  -0.228002193798

      self.object_place_pose_target[x].orientation.x = 0.0387329126595
      self.object_place_pose_target[x].orientation.y = 0.998648288978
      self.object_place_pose_target[x].orientation.z = -0.0312958668684
      self.object_place_pose_target[x].orientation.w = 0.0148971512749
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

    if req.object == "Sugar" or req.object == "Tea":
      print place_pose_cup
      place_pose_offset = copy.deepcopy(place_pose_cup)
      place_pose_offset.position.z = place_pose_offset.position.z + 0.15
      rospy.sleep(3)
      response = self.moveToPose(place_pose_offset)
      place_pose_offset_copy = copy.deepcopy(place_pose_offset)
    
      q_orig = [place_pose_offset_copy.orientation.x,place_pose_offset_copy.orientation.y,place_pose_offset_copy.orientation.z,place_pose_offset_copy.orientation.w]
      q_rot = quaternion_from_euler(0,0.349066,0)
      q_new = quaternion_multiply(q_rot, q_orig)
      place_pose_offset_copy.orientation.x = q_new[0]
      place_pose_offset_copy.orientation.y = q_new[1]
      place_pose_offset_copy.orientation.z = q_new[2]
      place_pose_offset_copy.orientation.w = q_new[3]
      self.moveToPose(place_pose_offset_copy)


    else:
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

