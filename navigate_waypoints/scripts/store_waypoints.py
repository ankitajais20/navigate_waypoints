#!/usr/bin/env python
import rospy
import rospkg
import threading
import csv
import tf
import time
from std_msgs.msg import Empty
from tf import TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped


output_file_path = rospkg.RosPack().get_path('navigate_waypoints')+"/waypoints_file/pose.csv"
waypoints = []
def main():
        rospy.init_node('store')
        addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
        posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        poseArray_publisher = rospy.Publisher(posearray_topic, PoseArray, queue_size=1)

        global waypoints
        waypoints = [] 
        poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        topic = addpose_topic;
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)


        while (1):
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue 
                else:
                    raise e
            rospy.loginfo("Recieved new waypoint")
            waypoints.append(correctFrame(pose, "map"))
            poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
def convert_PoseWithCovArray_to_PoseArray(waypoints):
    poses = PoseArray()
    poses.header.frame_id ='map'
    poses.poses = [pose.pose.pose for pose in waypoints]
    with open(output_file_path, 'w') as file:
           for pose in waypoints:
                file.write(str(pose.pose.pose.position.x) + ',' + str(pose.pose.pose.position.y) + ',' + str(pose.pose.pose.position.z) + ',' + str(pose.pose.pose.orientation.x) + ',' + str(pose.pose.pose.orientation.y) + ',' + str(pose.pose.pose.orientation.z) + ',' + str(pose.pose.pose.orientation.w)+ '\n')
	        rospy.loginfo('poses written to '+ output_file_path)	
    return poses

# this function used for changing the position according to the new frame
def correctFrame(waypoint,target_frame):
    if waypoint.header.frame_id == target_frame:
        return waypoint

    try:
        correctFrame.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = correctFrame.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
