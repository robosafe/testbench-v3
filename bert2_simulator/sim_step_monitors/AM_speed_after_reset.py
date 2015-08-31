"""
This monitor checks the following assertion:
IF (time since robot reset is less than 5s) 
THEN (robot hand speed does not exceed 250 mm/s)

Created by David Western, Aug 2015.
"""

# These two imports must be here for all AMs:
from assertion_monitor_baseclass import AssertionMonitor
import rospy

from bert2_simulator.msg import *
from gazebo_msgs.srv import GetLinkState

# Make sure this name matches the name of this file.
#     vvvvvvvvvvvvvvv
class AM_speed_after_reset(AssertionMonitor):

    def __init__(self,trace_label='0'):


        """ 
            List, in order, the stages of the precondition and postcondition.
            The strings given here should name methods defined within this class.
            The assertion_monitor base class requires both these attributes to be
            defined.
        """
        # Required (but adjust the list entries to reflect your own AM):
        self.precon_list = ['checkReset']
        self.postcon_list = ['checkSpeed']

        # Make sure /use_sim_time is true, for use in time checks:
        use_sim_time = rospy.get_param('/use_sim_time')
        # print "use_sim_time: ", use_sim_time
        if not use_sim_time:
                rospy.set_param('/use_sim_time',True)
        self.T = 5 # Time to wait, in seconds.

        # Attributes specific to this example:
        self.recentReset = 0
	rospy.Subscriber("robot_reset", Robot, self.reset_callback) 
        self.state_sp = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)


        # Required at the end of __init__:
        # super(YOUR_AM_CLASSNAME_HERE,self).__init__(trace_label) # Execute constructor of base class.
        super(AM_speed_after_reset,self).__init__(trace_label) # Execute constructor of base class.



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
            precondition on the next simulation time-step), unless it's a forked copy,
            in which case it just propagates its flags up towards its parent and dies.

        TO DO:  Allow forking of the AM instance so that multiple independent precon (and
                then postcon) checks can run concurrently;  add a self.fork() member
                function in assertion_monitor.py, to be invoked in the stage member
                functions below?
    """
    def checkReset(self):
        if self.recentReset==1:
            self.t_Now = rospy.get_time()
            if self.t_Now-self.t_WaitStart<self.T:
                return 1
            else:
                self.recentReset=0
                return 0
        else:
            return 0


    def reset_callback(self,data):
        if data.informedHuman==1:
            self.recentReset=1
            self.t_WaitStart = rospy.get_time()

        
    
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
            Hence the assertion is deemed violated.  A flag will be raised to indicate this.
            Depending on mode settings (not yet implemented), either the simulation will
            end or the monitor will reset.
    """
    def checkSpeed(self):
        
        state = self.state_sp('bert2::left_wrist_flex_link','world')
        vel = state.link_state.twist.linear
        spd = ( vel.x**2 + vel.y**2 + vel.z**2 )**0.5
        if spd<=0.25:
                return 1
        else:
                return -1

        
