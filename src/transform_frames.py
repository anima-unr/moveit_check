#!/usr/bin/env python
''' transform_frames.py

    Class to transform point and coordinates between robot frames

    Daniel Morris, April 2020, Nov 2021    
    Copyright 2020, 2021
'''
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, PointStamped, TransformStamped
from std_msgs.msg import Header
import tf2_ros, tf2_geometry_msgs

class TransformFrames():
    def __init__(self):
        ''' Create a buffer of transforms and update it with TransformListener '''
        self.tfBuffer = tf2_ros.Buffer()           # Creates a frame buffer
        tf2_ros.TransformListener(self.tfBuffer)   # TransformListener fills the buffer as background task
    
    def get_transform(self, source_frame, target_frame, stamp=rospy.Time(0), duration=rospy.Duration(0.2)):
        try:
            trans = self.tfBuffer.lookup_transform(target_frame, source_frame, stamp, duration )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('Cannot find transformation from {source_frame} to {target_frame}')
            raise Exception('Cannot find transformation from {source_frame} to {target_frame}')
        return trans     # Type: TransformStamped

    def pose_transform(self, pose_array, target_frame='odom'):
        ''' pose_array: will be transformed to target_frame '''
        trans = self.get_transform( pose_array.header.frame_id, target_frame, pose_array.header.stamp )
        new_header = Header(frame_id=target_frame, stamp=pose_array.header.stamp) 
        pose_array_transformed = PoseArray(header=new_header)
        for pose in pose_array.poses:
            pose_s = PoseStamped(pose=pose, header=pose_array.header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
            pose_array_transformed.poses.append( pose_t.pose )
        return pose_array_transformed

    def point_transform(self, point, target_frame='odom'):
        ''' Transform a PointStamped to a new frame '''
        trans = self.get_transform( point.header.frame_id, target_frame, point.header.stamp )
        return tf2_geometry_msgs.do_transform_point(point, trans )

    def get_frame_A_origin_frame_B(self, frame_A, frame_B, stamp=rospy.Time(0) ):
        ''' Returns the pose of the origin of frame_A in frame_B as a PoseStamped '''
        header = Header(frame_id=frame_A, stamp=stamp)        
        origin_A = Pose(position=Point(0.687166666502,-0.302388688271,-0.0385500488401), orientation=Quaternion(-0.202031220217,-0.677513391286,-0.144313390393,0.692338527051))
        origin_A_stamped = PoseStamped( pose=origin_A, header=header )
        pose_frame_B = tf2_geometry_msgs.do_transform_pose(origin_A_stamped, self.get_transform(frame_A, frame_B, stamp))
        return pose_frame_B

