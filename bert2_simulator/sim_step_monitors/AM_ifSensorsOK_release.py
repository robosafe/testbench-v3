#!/usr/bin/env python
"""
AM_ifSensorsOK_release
Assertion monitor
For use in Human-Robot Interaction simulator.

Created by David Western, July 2015.  

Implements assertion: if humanOk and sensorsOk and contact_robot_hand_and_object ==1 then assert contact_robot_hand_and_object == 0
OR: if human and sensors are OK and robot is grabbing piece, then object is released
"""

# These two imports must be here for all AMs:
from assertion_monitor_baseclass import AssertionMonitor
import rospy

# Imports specific to this AM:
from bert2_simulator.msg import *
import dist_between_links
from std_msgs.msg import Float64
from std_msgs.msg import Int8

# Make sure this name matches the name of this file.
#     vvvvvvvvvvvvvvv
class AM_ifSensorsOK_release(AssertionMonitor):

    def __init__(self,trace_label='0'):

        # Required (but adjust the list entries to reflect your own AM):
        self.precon_list = ['precon']
        self.postcon_list = ['Wait','Decide']

        # Make sure /use_sim_time is true, for use in Wait():
        use_sim_time = rospy.get_param('/use_sim_time')
        if not use_sim_time:
                rospy.set_param('/use_sim_time',True)
                print "use_sim_time changed to True"
        self.T = 3 # Wait period

        # This particular AM depends on values published on ROS topics.
        # Hence, let's set up some subscribers.
	rospy.Subscriber("human_signals",Human, self.h_signals_callback)
	rospy.Subscriber("gpl_is_ok", Int8, self.sensorsOK_callback)
	rospy.Subscriber("gpl_is_ok", Int8, self.object_robot_hand_callback) # Fudge: assumes perfect sensing.
        # Related initialisations:
        self.h_ready = 0
        self.sensorsOK = 0
        self.decision = 0



        # Required at the end of __init__:
        # super(YOUR_AM_CLASSNAME_HERE,self).__init__(trace_label) # Execute constructor of base class.
        super(AM_ifSensorsOK_release,self).__init__(trace_label) # Execute constructor of base class.


    """ 
        Define member functions representing each stage of the PREcondition below here.

        Each stage of the precondition must return 1 (satisfied), 0 (not yet satisfied),
        or -1 (invalidated)...
        1 - SATISFIED:
            The assertion monitor will progress to the next stage ON THE NEXT simulation
            time-step.  A special case is that, when the last stage of the precondition
            evaluates as 1, the first stage of the POSTcondition will be evaluated
            in the SAME time-step.  
        0 - NOT YET SATISFIED:
            The assertion monitor will evaluate the same stage again on the next
            simulation time-step.
        -1 - INVALIDATED:
            The stage has not been satisfied and cannot be satisfied in future evaluations.
            Hence the assertion monitor will reset (evaluate the first stage of the
            precondition on the next simulation time-step).

        TO DO:  Allow forking of the AM instance so that multiple independent precon (and
                then postcon) checks can run concurrently;  add a self.fork() member
                function in assertion_monitor.py, to be invoked in the stage member
                functions below?
    """

    def precon(self):
        
        dist = dist_between_links.check('object','bert2::left_wrist_flex_link')
	self.object_in_robot_hand = dist<=0.1
        if self.h_ready==1 and self.sensorsOK and self.object_in_robot_hand:
                self.t_WaitStart = rospy.get_time()
                self.object_hand = 0 # In case it's been set prematurely.
                self.decision = 0 # In case it's been set prematurely.
		return 1
        else:
		return 0


    
    """ 
        Define member functions representing each stage of the POSTcondition below here.

        Each stage of the postcondition must return 1 (satisfied), 0 (not yet satisfied),
        or -1 (violated)...
        1 - SATISFIED:
            The assertion monitor will progress to the next stage ON THE NEXT simulation
            time-step.  A special case is that, when the last stage of the postcondition
            evaluates as 1, the assertion is deemed satisfied. 
        0 - NOT YET SATISFIED:
            The assertion monitor will evaluate the same stage again on the next
            simulation time-step.
        -1 - VIOLATED:
            The stage has not been satisfied and cannot be satisfied in future evaluations.
            Hence the assertion deemed violated.  A flag will be raised to indicate this.
            Depending on mode settings (not yet implemented), either the simulation will
            end or the monitor will reset.
    """

    def Wait(self):

        now = rospy.get_time()
        if now-self.t_WaitStart>=self.T:
                return 1
        else:
                return 0
        

    def Decide(self):

        dist = dist_between_links.check('object','bert2::left_wrist_flex_link')
	self.object_in_robot_hand = dist<=0.1
	if not object_in_robot_hand:	
		rospy.loginfo('Valid assertion')
                return 1
	else:
		rospy.loginfo('Violation of property')
                return -1
        




    """
        Define callbacks for ROS subscribers.
    """

    def h_signals_callback(self,data):
	#Assuming perfect sensing and sensing delays
	if data.humanIsReady==1:
		self.h_ready = 1


    def sensorsOK_callback(self,data):
	#Assuming perfect sensing and sensing delays
	if data.data==1:
		self.sensorsOK = 1

    def object_robot_hand_callback(self,data):
	#Assuming perfect sensing and sensing delays
	if data.data==1:
		self.object_hand = 0
        else:
                self.object_hand = 1
