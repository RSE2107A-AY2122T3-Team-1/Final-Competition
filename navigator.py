#!/usr/bin/env python

import time
import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# Brings in the .action file and messages used by actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist

class Nav:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist, queue_size=1)
        self.twist = Twist()
        self.setter()

    def turn(self, val, sign):
        x = val
        while x > 0:
            self.twist.angular.z = sign*1.0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1)
            x -= 1
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def movement(self, val):
        x = val
        while x > 0:
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1)
            x -= 1
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def setter(self):    
        #orientate
        self.turn(5, -1)

        #4
        self.limo_navigator_node(-1.77, 0.0, 0.0, 0.68)
        self.turn(4, 1)
        
        #7
        self.limo_navigator_node(-1.77, -1.2, 0.0, 0.72)
        self.turn(8, 1)

        #1
        self.limo_navigator_node(-1.86, 2.1, 0.0, 0.99)
        self.turn(4, -1)

        #2
        self.limo_navigator_node(-0.11, 1.7, 0.0, 0.99)
        self.turn(3, 1)
        #self.movement(4)
	
	    #3
        self.limo_navigator_node(1.65, 1.95, 0.0, -0.23)
        self.turn(10, -1)
        self.movement(1)

        #2
        self.limo_navigator_node(-0.212, 1.7, 0.0, 0.35)
        self.turn(3, -1)
        #self.movement(3)
	
        #1
        self.limo_navigator_node(-1.95, 1.95, 0.0, 0.60)
        self.turn(5, 1)

        #4
        self.limo_navigator_node(-1.69, -0.12, 0.0, 0.68)
        self.turn(4, 1)
        
        #5
        self.limo_navigator_node(0.191, 0.0, 0.0, 0.076)
        self.turn(4, -1)
        
        #8
        self.limo_navigator_node(0.0891, -1.29, 0.0, 0.99)
        self.turn(2, -1)
        
        #8
        self.limo_navigator_node(-0.342, -1.91, 0.0, 0.99)
        self.turn(3, 1)
        
        #9
        self.limo_navigator_node(1.22, -1.8, 0.0, 0.95)
        self.turn(2, -1)
        self.movement(2)

        #end
        self.limo_navigator_node(2.32, -2.11, 0.0, 0.99)
        """  """
    
    def limo_navigator_node(self, x, y, z, w):

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()
        #rospy.loginfo('Connected to SimpleActionServer: move_base')

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = w

        # Sends the goal to the action server
        client.send_goal(goal)
        #rospy.loginfo('Executing Movement...')

        # Waits for the server to finish performing the action
        wait = client.wait_for_result()     

        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

        # If path is rejected or robot lost, reposition robot to starting position
        elif client.get_state() == GoalStatus.REJECTED or client.get_state() == GoalStatus.LOST:
            print("Coordinates REJECTED or LIMO lost")
            print("Attempting to return to origin...")
            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.w = 0.999

        # If navigation successfully executed and goal reached
        # next set of coordinates will be sent till none are left

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        #rospy.init_node('limo_navigator_node')
        rospy.init_node('Navigator')
        Navigator = Nav()
        # Buffer time to for other launch files (gazebo/rviz)
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
