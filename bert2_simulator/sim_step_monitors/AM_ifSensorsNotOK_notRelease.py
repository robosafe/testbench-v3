#!/usr/bin/env python
"""
AM_ifSensorsNotOK_notRelease
Assertion monitor
For use in Human-Robot Interaction simulator.

Created by David Western, July 2015.  

Implements assertion: if human is ready and NOT sensorsOk and contact_robot_hand_and_object ==1 then assert after T, contact_robot_hand_and_object == 1
(robot does not release object within T seconds of a negative sensor reading).
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
class AM_ifSensorsNotOK_notRelease(AssertionMonitor):
    """
        A template to illustrate the generic structure of an assertion monitor
        specification, for use in the human-robot interaction simulator.  
    """

    def __init__(self,trace_label):


        """ 
            List, in order, the stages of the precondition and postcondition.
            The strings given here should name methods defined within this class.
            The assertion_monitor base class requires both these attributes to be
            defined.
        """
        # Required (but adjust the list entries to reflect your own AM):
        self.precon_list = ['precon']
        self.postcon_list = ['Wait','Decide']

        # Make sure /use_sim_time is true, for use in Wait():
        use_sim_time = rospy.get_param('/use_sim_time')
        # print "use_sim_time: ", use_sim_time
        if not use_sim_time:
                rospy.set_param('/use_sim_time',True)
                print "use_sim_time changed to True"

        self.T = 3 # Time to wait, in seconds.

        # This particular AM depends on values published on ROS topics.
        # Hence, let's set up some subscribers.
	rospy.Subscriber("human_signals",Human, self.h_signals_callback)
	rospy.Subscriber("gpl_is_ok", Int8, self.sensorsOK_callback)
        # Related initialisations:
        self.h_ready = 0
        self.sensorsOK = 0
        self.decision = 0
        self.object_in_robot_hand = 1 # Fudge.



        # Required at the end of __init__:
        # super(YOUR_AM_CLASSNAME_HERE,self).__init__(trace_label) # Execute constructor of base class.
        super(AM_decisionInTime,self).__init__(trace_label) # Execute constructor of base class.


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
        
        dist = dist_between_links.check('object_link','bert2::left_wrist_flex_link')
        obj_in_rob_hand = dist<=0.1
        #if (not self.sensorsOK) and obj_in_rob_hand:
        if self.h_ready==1 and (not self.sensorsOK) and obj_in_rob_hand:
                self.t_WaitStart = rospy.get_time()
                self.object_hand = 1 # In case it's been set prematurely.
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
	if dist<=0.1:	
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
        else:
                self.sensorsOK = 0
