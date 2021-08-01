#!/usr/bin/env python
import rospy
import rospkg
import actionlib
import csv
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped


output_file_path = rospkg.RosPack().get_path('navigate_waypoints')+"/waypoints_file/pose.csv"
waypoints = []

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    poses = PoseArray()
    poses.header.frame_id = 'map'
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses
def goal_pose(pose):
   goal_pose = MoveBaseGoal()
   goal_pose.target_pose.header.frame_id = 'map'
   goal_pose.target_pose.pose.position = pose.pose.pose.position
   goal_pose.target_pose.pose.orientation = pose.pose.pose.orientation
   rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' % (pose.pose.pose.position.x, pose.pose.pose.position.y))
   return goal_pose

def main():
     rospy.init_node('patrol')
     addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
     posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
     poseArray_publisher = rospy.Publisher(posearray_topic, PoseArray, queue_size=1)

     with open(output_file_path, 'r') as file:
        reader = csv.reader(file, delimiter = ',')
        for row in reader:
            print row
            current_pose = PoseWithCovarianceStamped() 
            current_pose.pose.pose.position.x    = float(row[0])
            current_pose.pose.pose.position.y    = float(row[1])
            current_pose.pose.pose.position.z    = float(row[2])
            current_pose.pose.pose.orientation.x = float(row[3])
            current_pose.pose.pose.orientation.y = float(row[4])
            current_pose.pose.pose.orientation.z = float(row[5])
            current_pose.pose.pose.orientation.w = float(row[6])
            waypoints.append(current_pose)
            poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
     client.wait_for_server()

     for pose in waypoints:
       goal = goal_pose(pose)
       client.send_goal(goal)
       client.wait_for_result()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
     
