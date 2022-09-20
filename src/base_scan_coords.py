#!/usr/bin/env python
'''
    base_scan_coords.py 

    Finds base_scan coordinates relative to base_footprint

    See also: transform_frames.py

    Daniel Morris, Oct 2020
'''
import rospy
from transform_frames import TransformFrames

if __name__=="__main__":

    rospy.init_node('base_scan_coords') 

    tf = TransformFrames()  # This initializes frame buffer
    rospy.sleep(1.0)         # Sleep so that buffer can fill up

    pose = tf.get_frame_A_origin_frame_B('base','ar_marker_7')

    rospy.loginfo('Finding base_scan pose relative to base_footprint')
    rospy.loginfo(pose)


