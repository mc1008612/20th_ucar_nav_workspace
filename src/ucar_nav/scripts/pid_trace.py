#!/usr/bin/env python
#-*- coding: utf-8 -*-
"""
PID Trace test node
@yjy
11/27
"""

import rospy
from geometry_msgs.msg import Twist   



class PID_Trace(object):
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #ideal guide param
        self.ideal_tomid = rospy.get_param('ideal_tomid',0.001)  # ~[0,1]
        self.ideal_area_rate = rospy.get_param('ideal_area_rate',[0.15,0.2]) #~[pram_default]
        #pid velocity param
        self.align_area_rate_velocity = 0.15
        self.align_tomid_velocity = 0.3    
        self.center_tolarance = 0.07  #comp with raw tomid
        self.find_target_velocity = 0.1
        #time out check param
        self.timeout_duration_tomid = 20  
        self.timeout_duration_area_rate = 30
        self.timeout_duration_both = 20 
        self.timeout_duration_find_target = 60
        self.start_time = rospy.Time.now().to_sec()

        rospy.loginfo("PID Trace node start!")
        rospy.loginfo("Ideal area_rate"+str(self.ideal_area_rate))
        rospy.loginfo("Ideal tomid: "+ str(self.ideal_tomid))
        rospy.loginfo("Align velocity param:" + str(self.align_area_rate_velocity))
        rospy.loginfo("Align to mid velocity:" + str(self.align_tomid_velocity)+"\n\n\n")
        
    def is_timeout(self,duration):
        if rospy.Time.now().to_sec() - self.start_time < duration:
            return False
        else:
            return True
    def convert_twist(self,x, y):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        return twist
    
    def find_target(self):
        #start time
        self.start_time = rospy.Time.now().to_sec()
        rospy.set_param('rescue',0) == 0 
        while  (not (rospy.get_param('rescue',0) == 3 or rospy.get_param('rescue',0) == 4) 
                                and abs(rospy.get_param('rescue_x'))>self.center_tolarance 
                                and not self.is_timeout(self.timeout_duration_find_target)) :
            msg = Twist()
            msg.angular.z = self.find_target_velocity
            self.pub.publish(msg)
        else :
            if not (rospy.get_param('rescue',0) == 3 or rospy.get_param('rescue',0) == 4):
                rospy.signal_shutdown("Time out find target!")
            else :
                rospy.loginfo("Successfully find target!")
    def try_align_both(self):
         #start time
        self.start_time = rospy.Time.now().to_sec()            
        lower,upper= self.ideal_area_rate[0],self.ideal_area_rate[1]
        rospy.loginfo("upper:%f,lower:%f\nideal_tomid:%f",upper,lower,self.ideal_tomid)
        while not self.is_timeout(self.timeout_duration_both):
            rescue_state = rospy.get_param('rescue',0)
            if  rescue_state == 4:                                                                         #rescue_state = 4
                break
            #if return value from param sever is @RIGHT
            elif rescue_state == 3:                                                                        #rescue_state = 3
            ############################################################
            #caculate x
                area_rate = rospy.get_param('rescue_area',0)
                #normalize area_rate
                if area_rate == 0:
                    #no result
                    nomalize_area_rate = 0
                elif area_rate < lower :
                    #require x>0,then area_rate >0
                    nomalize_area_rate = (lower - area_rate)/lower
                elif area_rate > upper:
                    #require x<0,then area_rate <0
                    nomalize_area_rate = (upper - area_rate)/upper  
                else :
                    nomalize_area_rate = 0
                    #ideal value
                pid_velocity_x = self.align_area_rate_velocity*nomalize_area_rate
                #############################################################
                #calculate y
                tomid = rospy.get_param('rescue_x')
                #normalize tomid
                normalize_tomid = tomid/0.5
                rospy.loginfo('tomid=%f,normalize_tomid=%f',tomid,normalize_tomid)
                if abs(normalize_tomid) > self.ideal_tomid:   
                    pid_velocity_y = -self.align_tomid_velocity*normalize_tomid
                else :
                    pid_velocity_y = 0 
                #############################################################
                self.pub.publish(self.convert_twist(pid_velocity_x, pid_velocity_y))
            else:                                                                                          #rescue_state = 0|1
                continue 
        else :                                                  
            rospy.loginfo("Time out Align to area_rate and tomid!")
            return False
            
        rospy.loginfo("Successfully Align to area_rate and tomid!")       
        return True

    def try_align_tomid(self):

        self.start_time = rospy.Time.now().to_sec()            
        while not self.is_timeout(self.timeout_duration_tomid):
            tomid = rospy.get_param('rescue_x')
            #normalize tomid
            normalize_tomid = tomid/0.5
            rospy.loginfo('tomid=%f,normalize_tomid=%f',tomid,normalize_tomid)
            if abs(normalize_tomid) > self.ideal_tomid:   
                pid_velocity_y = -self.align_tomid_velocity*normalize_tomid
            else :
                pid_velocity_y = 0
                break
            self.pub.publish(self.convert_twist(0, pid_velocity_y))                         
        else :                                                  
            rospy.loginfo("Time out Align to mid!")
            return False
  
        rospy.loginfo("Successfully Align to mid!")            
        return True


    def try_align_area_rate(self):
        self.start_time = rospy.Time.now().to_sec()             
        lower,upper= self.ideal_area_rate[0],self.ideal_area_rate[1]
        rospy.loginfo("upper:%f,lower:%f",upper,lower)
        while not self.is_timeout(self.timeout_duration_area_rate):
            if  rospy.get_param('rescue',3) == 4:
                break
            area_rate = rospy.get_param('rescue_area',0)
            #normalize area_rate
            if area_rate == 0:
                #no result
                nomalize_area_rate = 0
            elif area_rate < lower :
                #require x>0,then area_rate >0
                nomalize_area_rate = (lower - area_rate)/lower
            elif area_rate > upper:
                #require x<0,then area_rate <0
                nomalize_area_rate = (area_rate - upper)/upper
            else :
                nomalize_area_rate = 0
                #ideal value
            rospy.loginfo('area_rate=%f,normalize_area_rate=%f',area_rate,nomalize_area_rate)
            pid_velocity_x = self.align_area_rate_velocity*nomalize_area_rate
            self.pub.publish(self.convert_twist(pid_velocity_x, 0))
        else :                                                 
            rospy.loginfo("Time out Align to area_rate!")
            return False
        
        rospy.loginfo("Successfully Align to area_rate!")       
        return True

 
if __name__ == '__main__':  
    try:  
        rospy.init_node('pid_trace.py')
        pid_trace = PID_Trace()
        #wait for start test
        rospy.loginfo("Waiting for start test...")
        #######################################
        while not rospy.is_shutdown():
            while not rospy.get_param("start_test",0) ==1:
                pass
            else:
                rospy.set_param("start_test",0)
                rospy.loginfo("Start test!")
            ##########################block#############################
            pid_trace.find_target()
            ##########################block#############################
            if not rospy.get_param('rescue',0) == 4:
                rospy.loginfo("Align mid")
                if pid_trace.try_align_tomid():
            #         rospy.loginfo("Align both")
            # ##########################block#############################
            #         if pid_trace.try_align_both():
            # ##########################block#############################
                        pass
                    # else : 
                    #         rospy.loginfo("Align area_rate and tomid failed!")
                    #         rospy.logoinfo("PID Trace node terminated!")
                else :
                    rospy.loginfo("Align mid failed!")
            else :
                rospy.loginfo("Successfully Align to area_rate and tomid!")
            ######################################
    except rospy.ROSInterruptException:  
        rospy.signal_shutdown("PID Trace node terminated!")