#!/usr/bin/env python 
# -*- coding: utf-8 -*-
"""
@auther: yjy
add functions: mark points to dictionary `locations` by RViz

12/13
@

"""
import rospy  
import actionlib  
import collections
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose,PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt 

class MultiNav():
    n_locations = 0
    locations = collections.OrderedDict()   
    def __init__(self):  
        rospy.init_node('MultiNav', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
  
        # How long in seconds should the robot pause at each location?  
        self.rest_time = rospy.get_param("~rest_time", 0)  
  
        # Are we running in the fake simulator?  
        self.fake_test = rospy.get_param("~fake_test", False)  
  
        # Goal state return values  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED',  
                       'ABORTED', 'REJECTED','PREEMPTING', 'RECALLING',   
                       'RECALLED','LOST']  
  
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click  
        # Nav Goals in RViz when running in the simulator.  
        # Pose coordinates are then displayed in the terminal  
        # that was used to launch RViz.  
 
        
  	
        #locations['point-1'] = Pose(Point(0.41468486189842224, 0.22052276134490967, 0.00), Quaternion(0.000, 0.000, 0.9113036615504121, 0.9113036615504121))
        #locations['point-2'] = Pose(Point(1.9080162048339844,-3.650221586227417, 0.00), Quaternion(0.000, 0.000, -0.7017961470759297, 0.7123778266828495))
        #locations['point-3'] = Pose(Point(0.41468486189842224, 0.22052276134490967, 0.00), Quaternion(0.000, 0.000, 0.9113036615504121, 0.9113036615504121))
  
 
 
        # Publisher to manually control the robot (e.g. to stop it)  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  
  
        # Subscribe to the move_base action server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        rospy.loginfo("Waiting for move_base action server...")  
  
        # Wait 60 seconds for the action server to become available  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
          
        # A variable to hold the initial pose of the robot to be set by the user in RViz  
        initial_pose = PoseWithCovarianceStamped()  
        # Variables to keep track of success rate, running time, and distance traveled   
        n_goals = 0  
        n_successes = 0  
        i = 0  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""  
        # Get the initial pose from the user  
        rospy.loginfo("Click on the map in RViz to set the intial pose...")  
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
        self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose) 
        while True:
            try:
                keyinput = int(input("Input 0 to continue,or reget the initialpose!\n"))
                break
            except ValueError:
                rospy.loginfo("invalid input!")
        while keyinput != 0:
            rospy.loginfo("Click on the map in RViz to set the intial pose...")  
            rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
            rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose) 
            while True:
                try:
                    keyinput = int(input("Input 0 to continue,or reget the initialpose!"))
                    break
                except ValueError:
                    rospy.loginfo("invalid input!")
 
        # Make sure we have the initial pose  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  
        rospy.loginfo("Starting navigation test")  
        #############################################################
        #get mark points from RViz
        while True:
            rospy.set_param('take_photo', 0)
            rospy.loginfo("#Click on the map in RViz to set the #LOCATIONS#\npoint-%d",self.n_locations+1)
            location_msg = rospy.wait_for_message('move_base_simple/goal',PoseStamped)
            self.rviz2locations(location_msg)
            
            rospy.loginfo("\nInput 0 to #NEXT#\nInput 1 to #END#\nOR #REGET# this time input\n")
            while True:
                try:
                    keyinput = int(input("Choice:"))
                    break
                except ValueError:
                    rospy.loginfo("invalid input!")

            #next
            if keyinput == 0:
                rospy.loginfo("\nexit 0\n")
                while True:
                    try:
                        keyinput = int(input("if take photo,input 0\n"))
                        break
                    except ValueError:
                        rospy.loginfo("invalid input!")
                if keyinput == 0:
                    rospy.set_param('take_photo', 1)
                while True:
                    try:
                        keyinput = int(input("\n#CHECK CAM#\nif #RIGHT# input 0\nelse to retry\n"))
                        break
                    except ValueError:
                        rospy.loginfo("invalid input!")
                if keyinput == 0:
                    continue
                else :
                    self.n_locations-=1
                    rospy.loginfo("\nexit else\n")
                    continue 
            #end
            elif keyinput == 1:
                rospy.loginfo("\nexit 1\n")
                break
            #re-input
            else:
                self.n_locations-=1
                rospy.loginfo("\nexit else\n")
                continue
        #make sure what we have input
        #idel format
        #locations['point-3'] = Pose(Point(0.41468486189842224, 0.22052276134490967, 0.00), Quaternion(0.000, 0.000, 0.9113036615504121, 0.9113036615504121))
        index = 0
        rospy.loginfo("#######################################################\nlocations_log:\n")
        for location in self.locations.keys():
            index+=1
            rospy.loginfo("locations['point-%d'] = Pose(Point(%f,%f,%f),Quaternion(%f,%f,%f,%f))\n"
            ,index,self.locations[location].position.x,self.locations[location].position.y,self.locations[location].position.z,self.locations[location].orientation.x
            ,self.locations[location].orientation.y,self.locations[location].orientation.z,self.locations[location].orientation.w)
        rospy.loginfo("#######################################################")
        #############################################################
        # Begin the main loop and run through a sequence of locations
        while True:
            try:
                keyinput = int(input("input 0 to start nav,1 to exit program\n"))
                break
            except ValueError:
                rospy.loginfo("invalid input!,input"+str(keyinput))
        if keyinput == 0:  
            for location in self.locations.keys():  
    
                rospy.loginfo("Updating current pose.")  
                distance = sqrt(pow(self.locations[location].position.x  
                            - initial_pose.pose.pose.position.x, 2) +  
                            pow(self.locations[location].position.y -  
                            initial_pose.pose.pose.position.y, 2))  
                initial_pose.header.stamp = ""  
    
                # Store the last location for distance calculations  
                last_location = location  
    
                # Increment the counters  
                i += 1  
                n_goals += 1  
    
                # Set up the next goal location  
                self.goal = MoveBaseGoal()  
                self.goal.target_pose.pose = self.locations[location]  
                self.goal.target_pose.header.frame_id = 'map'  
                self.goal.target_pose.header.stamp = rospy.Time.now()  
    
                # Let the user know where the robot is going next  
                rospy.loginfo("Going to: " + str(location))  
                # Start the robot toward the next location  
                self.move_base.send_goal(self.goal)  
    
                # Allow 5 minutes to get there  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))  
    
                # Check for success or failure  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("Timed out achieving goal")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("Goal succeeded!")  
                        n_successes += 1  
                        distance_traveled += distance  
                    else:  
                        rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
    
                # How long have we been running?  
                running_time = rospy.Time.now() - start_time  
                running_time = running_time.secs / 60.0  
    
                # Print a summary success/failure, distance traveled and time elapsed  
                rospy.loginfo("Success so far: " + str(n_successes) + "/" +  
                            str(n_goals) + " = " + str(100 * n_successes/n_goals) + "%")  
                rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +  
                            " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  
                rospy.sleep(self.rest_time)  
        else :
            return
    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  
  
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
    ###################################################
    #move_base_simple/goal msg_struct
    #     pose: 
    #   position: 
    #     x: 1.9943434000015259
    #     y: -1.5142136812210083
    #     z: 0.0
    #   orientation: 
    #     x: 0.0
    #     y: 0.0
    #     z: 0.6949058535183403
    #     w: 0.7191007264256843
    def rviz2locations(self,msg):
        self.n_locations+=1
        rospy.loginfo("received point :\nx = %0.6f\ny = %0.6f\nz = %0.6f\n"
                        ,msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rospy.loginfo("received orientation :\nz = %0.6f\nw = %0.6f\n"
                        ,msg.pose.orientation.x, msg.pose.orientation.w)
        rospy.loginfo("create locations point-#%d\n",self.n_locations)
        self.locations['point -'+str(self.n_locations)] = Pose(
                                                Point(msg.pose.position.x,msg.pose.position.y , msg.pose.position.z), 
                                                Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
    ###################################################
def trunc(f, n):  
  
    # Truncates/pads a float f to n decimal places without rounding  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  
  
if __name__ == '__main__':  
    try:  
        MultiNav()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("AMCL navigation test finished.")  
