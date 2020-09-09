#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot
from std_msgs.msg import String

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub
    #pub = rospy.Publisher('move', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()
    # Call service client with path

    # Transform Setpoint from service client

    # Create Twist message from the transformed Setpoint

    # Publish Twist

    # Call service client again if the returned path is not empty and do stuff again

    # Send 0 control Twist to stop robot

    # Get new path from action server


def get_path():
    global goal_client

    # Get path from action server

    # Call move with path from action server


if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    # Init publisher
    pub = rospy.Publisher('move', String, queue_size=10)
    
    """ 
    try:
        move()
    except rospy.ROSInterruptException:
        pass
    """
    # Init simple action client
    client_next_goal = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    client_next_goal.wait_for_server()
    client_next_goal.wait_for_result()
    
           
    # Init service client
    rospy.wait_for_service('get_setpoint')

    
    # Call get path

    path,gain = client_next_goal.get_result()
    rospy.loginfo("result next goal",path,gain)
    if path is empty:
        exit()
    while path is not empty:
        try:
        get_setpoint = rospy.ServiceProxy('get_setpoint', GetSetpoint)
        path,setpoint = get_setpoint(path)
        rospy.loginfo("The frame is: %s", setpoint.header.frame_id)
        #setpoint_transformed = transform_setpoint_to_robot_frame(setpoint)
        #publish(setpoint_transformed)
        #sleep()
        
        #return resp1.sum
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
    # Spin
    rospy.spin()
