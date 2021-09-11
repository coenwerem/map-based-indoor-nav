#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates

#  MODELS
goal_pos = Point()
goal_pos.x = 0
goal_pos.y = 0
goal_pos.z = 0

def models_callback(model_states):
    global goal_pos    
    mod_num = len(model_states.name)# Finding the number of models in Gazebo

    # Retrieving the pose of the marker/goal location and robot using its name attribute
    for i in range(mod_num):
        if model_states.name[i] == 'goal_location':
            goal_pos = model_states.pose[i].position

models_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, models_callback) # Subscribing to Gazebo topic to btain goal location data

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    client.wait_for_server()

   # Create a new nav goal with the MoveBaseGoal object
    nav_goal = MoveBaseGoal()
    nav_goal.target_pose.header.frame_id = "map"
    nav_goal.target_pose.header.stamp = rospy.Time.now()

   # Set nav_goal location to marker pose
    nav_goal.target_pose.pose.position.x = goal_pos.x
    nav_goal.target_pose.pose.position.y = goal_pos.y
    nav_goal.target_pose.pose.position.z = goal_pos.z
    nav_goal.target_pose.pose.orientation.w = 1.0

   # Send goal to action server and wait for action to be executed.
    client.send_goal(nav_goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server is unavailable!")
        rospy.signal_shutdown("Action server is unavailable!")
    else:
        return client.get_result()   


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal sent successfully!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test completed.")