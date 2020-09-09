#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot,sqrt
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
   
   
    #hello_str = "hello world %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
    #pub.publish(hello_str)

    rospy.loginfo("The path is: %s", path)
    trans = tf_buffer.lookup_transform('base_link',setpoint, rospy.Time())
    trans = tf2_geometry_msgs.do_transform_point(setpoint,trans)
    msg = Twist()
    msg.angular.z = max_angular_velocity * atan2(trans.point.y, trans.point.x)
    msg.linear.x =  max_linear_velocity * sqrt(trans.point.x ** 2 + trans.point.y ** 2)
    pub.publish(msg)
    rospy.loginfo(msg)
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
    client_next_goal.send_goal(goal_client)
    client_next_goal.wait_for_result()
    # Call move with path from action server
    return client_next_goal.get_result()

if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')
    rate = rospy.Rate(10) # 10hz
   
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    # Init publisher
    pub = rospy.Publisher('Twist', Twist, queue_size=10)
    # Init simple action client
    client_next_goal = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    client_next_goal.wait_for_server()
    
    # Init service client
    rospy.wait_for_service('get_setpoint')
      
    while not rospy.is_shutdown():
        # Call get path
        path,gain = get_path()

        if path is None:
            exit()
        while path is not None:
            try:
                get_setpoint = rospy.ServiceProxy('get_setpoint', GetSetpoint)
                path,setpoint = get_setpoint(path)
                move(path)
            except (rospy.ServiceException):
                rospy.loginfo("Service call failed")
            
            
    # Spin
    rospy.spin()
