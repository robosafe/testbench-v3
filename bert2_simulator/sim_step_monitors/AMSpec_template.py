"""
Template assertion monitor specification
For use in Human-Robot Interaction simulator.

Created by David Western, May 2015.
"""

# These two imports must be here for all AMs:
from assertion_monitor_baseclass import AssertionMonitor
import rospy

# This import is only required for the AMSpec_template example.  You may
# delete it if creating your own AM.
import random

# Make sure this name matches the name of this file.
#     vvvvvvvvvvvvvvv
class AMSpec_template(AssertionMonitor):
    """
        A template to illustrate the generic structure of an assertion monitor
        specification, for use in the human-robot interaction simulator.  
    """

    def __init__(self,trace_label='0'):


        """ 
            List, in order, the stages of the precondition and postcondition.
            The strings given here should name methods defined within this class.
            The assertion_monitor base class requires both these attributes to be
            defined.
        """
        # Required (but adjust the list entries to reflect your own AM):
        self.precon_list = ['precon1']
        self.postcon_list = ['postcon1','postcon2']

        # Attributes specific to this example:
        self.example_target = 10
        self.example_count = 0
        self.printOutput = False # Flag to turn on/off debug outputs.
        # Note that you can use attributes like these to track historical values that
        # the AM needs in order to evaluate future stages.


        # Required at the end of __init__:
        # super(YOUR_AM_CLASS_NAME_HERE,self).__init__(trace_label) # Execute constructor of base class.
        super(AMSpec_template,self).__init__(trace_label) # Execute constructor of base class.




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
    def precon1(self):

        # Update attributes:
        self.example_count += 1

        if self.printOutput:
            print "Count = ", self.example_count
            print "Target = ", self.example_target

        # Check:
        if self.example_count >= self.example_target:
            
            # Use the 'fork()' method (defined in the base class) to control when a
            #   new instance of the AM is instantiated, to run concurrently with the
            #   present instance, but one step behind. IMPLEMENTATION NOT COMPLETE.
            #self.fork() 
            self.example_count = 0
            return 1 # assertion_monitor.py will trigger postcon.
        else:
            return 0 # Do nothing.

    
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

        if self.printOutput:
            print "postcon1"
        # Preliminary postcondition:
        if random.randint(0,1)==1: # Coin-toss.
            if self.printOutput:
                print "Result = ", 1
            return 1
        else:
            if self.printOutput:
                print "Result = ", 0
            return 0


    def postcon2(self):

        if self.printOutput:
            print "postcon2"
        # Preliminary postcondition:
        if random.randint(0,1)==1: # Coin-toss.
            if self.printOutput:
                print "Result = ", 1
            return 1
        else:
            if self.printOutput:
                print "Result = ", -1
            return -1
        
