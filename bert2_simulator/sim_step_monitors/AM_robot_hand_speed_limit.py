"""
Assertion monitor to check that the robot's hand never exceeds a specified limit.

Created by David Western, Aug 2015.
"""

# These two imports must be here for all AMs:
from assertion_monitor_baseclass import AssertionMonitor
import rospy

from gazebo_msgs.srv import GetLinkState

# Make sure this name matches the name of this file.
#     vvvvvvvvvvvvvvv
class AM_robot_hand_speed_limit(AssertionMonitor):

    def __init__(self,trace_label='0'):

        # Required (but adjust the list entries to reflect your own AM):
        self.precon_list = ['speedcheck']
        self.postcon_list = ['postcon1']

        self.state_sp = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)

        # Required at the end of __init__:
        # super(YOUR_AM_CLASS_NAME_HERE,self).__init__(trace_label) # Execute constructor of base class.
        super(AM_robot_hand_speed_limit,self).__init__(trace_label) # Execute constructor of base class.




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
    def speedcheck(self):
        state = self.state_sp('bert2::left_wrist_flex_link','world')
        #print state
        vel = state.link_state.twist.linear
        spd = ( vel.x**2 + vel.y**2 + vel.z**2 )**0.5
        #print "speed:",spd
        if spd<0.25:
                return 0
        else:
                return 1
        
    
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
    def postcon1(self):
        # They've already failed.
        return -1
