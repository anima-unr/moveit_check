#!/usr/bin/env python
'''
    base_scan_transform.py 

    Transforms a pose in base_scan to base_footprint

    Daniel Morris, Oct 2020
'''
import rospy
from transform_frames import TransformFrames
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from std_msgs.msg import Header

if __name__=="__main__":

    rospy.init_node('base_scan_coords') 

    tf = TransformFrames()  # This initializes frame buffer
    rospy.sleep(1.0)         # Sleep so that buffer can fill up

    # Here a pose defined in base_scan
    scan_pos = Pose(position=Point(0.783297669595,-0.0469012368112,-0.182741324213), orientation=Quaternion(-0.0003481747685,-0.0242994292008,-0.120378653421,0.99243054987))
    # Create a pose_array with frame_id='base_scan' to store it
    pose_array = PoseArray(header=Header(frame_id='base',stamp=rospy.Time(0)))
    pose_array.poses.append(scan_pos)

    new_pose_array = tf.pose_transform(pose_array=pose_array, target_frame='ar_marker_7')

    rospy.loginfo('Original pose in base_scan')
    rospy.loginfo(pose_array)
    rospy.loginfo('New pose in base_footprint')
    rospy.loginfo(new_pose_array)


