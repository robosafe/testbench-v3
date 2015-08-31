"""
Assertion monitor base class
For use in Human-Robot Interaction simulator.

The specific assertion monitor is defined in a separate class (see AMSpec_template, with a
state-machine structure.  The present class (AssertionMonitor) initialises the specified AM
and steps it through its transitions.

Created by David Western, May 2015.
"""

from copy import deepcopy
import datetime
import multiprocessing
import rospkg
import rospy
import time
import types

#---------------------------------------------------------------------------------------------------
class AssertionMonitor(str):
    """
    assertion_monitor class, for implementing assertion monitors in human-robot interaction simulator.

    AssertionMonitor inherits from a type (int) to ensure that AssertionMonitor and its children are
        'new-style' classes (https://docs.python.org/2/reference/datamodel.html#newstyle).  The 
        initial motivation in this case was to enable the use of super() to call AssertionMonitor's
        constructor from within the child class.

    Important attributes:
      precon_checked (bool): Changes from 0 to 1 once the precondition's been checked.
      postcon_checked (bool): Changes from 0 to 1 once the postcondition's been checked,
                              i.e. once the precondition's been checked and satisfied.
      postcon_violated (bool: Changes from 0 to 1 once the postcondition's been violated.
    """

    allow_concurrency = 0

    def __init__(self,trace_label="0"):

        self.trace_label = trace_label
        
        # Single-change flags:
        self.precon_checked = 0 # Changes to 1 once the precondition's been checked. (for coverage)
        self.postcon_checked = 0 # Changes to 1 once the postcondition's been checked. (for coverage)

        # Other flags:
        self.precon_satisfied = 0
        self.postcon_violated = [] # Changes to 1 if the postcondition's violated, 0 if satisfied.
        self.precon_ind = 0  # Start on the first stage of the precon
        self.postcon_ind = 0 #  ""            ""            "" postcon

        #spec = AMSpec() # Instantiate the specified assertion monitor, for compliance check (below).
        #self.severity = severity
        #self.msg = msg
        self.severity=3      # Does nothing at the moment.
        self.msg='VIOLATION' # Ditto.
        """
        Not implemented yet - 
        severity (integer):    - Must be an integer, 0 to 3.  Determines the behaviour of the simulator
                               in the case that this assertion is violated.  The gradations below
                               are derived from the OVL specification provided by Accellera.
                                - 0: Information only (no improper design functionality).
                                - 1: Warning
                                - 2: Runtime error
                                - 3 (default): Fatal runtime error - simulation stops.

        msg (string):          - Message reported upon violation of this assertion. Default is
                               "VIOLATION".
        """

        # Check AMSpec has the mandatory attributes:
        mand_attrs = ['precon_list','postcon_list']
        for attr in mand_attrs:
            if not hasattr(self,attr):
                rospy.logerr("Assertion monitor specification '%s' does not define the attribute \
                              '%s', which is required by AssertionMonitor (assertion_monitor.py).",
                              self.__class__.__name__,attr)
        """ TO DO: Also make sure there aren't other attributes whose names clash with the ones defined
            here in the base class.
        """
            

        # Open file for recording signals into
        self.recordingOn = 1 # TO DO: Implement this as a ROS param or launch param.
        rospack = rospkg.RosPack()
        path = rospack.get_path('bert2_simulator')
        path = path+'/sim_step_monitors/signals/'
        filename = path+self.__class__.__name__+"_signals.txt" 
                                        # TO DO: Create a separate file
                                        #        for each simulation run?
        self.f = open(filename, 'a')
        self.f.write("\n")
        self.f.write("-----------------------------------------------\n")
        self.f.write(self.__class__.__name__)
        self.f.write("\n")
        self.f.write("Trace "+self.trace_label)
        self.f.write(" instantiated: ")
        self.f.write(datetime.datetime.now().strftime("%Y/%B/%d %I:%M%p"))
        self.f.write("\n")
        self.f.write("- BEGIN -\n")
        self.f.write("-----------------------------------------------\n")

        # Path for reports:
        report_path = rospack.get_path('bert2_simulator')
        report_path = report_path+'/sim_step_monitors/reports/'
        self.report_file = report_path+self.__class__.__name__+'.txt'

        # Initialise empty list of copies (forks) of this AM:
        self.AMCopies = []
        # ... and mark this one as the original:
        self.is_a_copy = 0

    def step(self,iteration,sim_time):

        # Step any copies of this AM:
        # Do this before stepping this parent AM, because that may create a new copy
        # that we don't won't to step until the next time-step (so that it lags its parent).
        for ind,cp in enumerate(self.AMCopies):
            cp.step(iteration,sim_time)
            # Consume flags from any children now labelled for deletion: 
            if cp.kill_me==1:
                self._consume_child_flags(ind)
        # Delete consumed copies:
        self.AMCopies[:] = [cp for cp in self.AMCopies if not cp.kill_me==1]

        # Precon stuff:
        if not self.precon_satisfied:

            # Find which precon stage we're on and evaluate it:
            stage = getattr(self, self.precon_list[self.precon_ind])
            result = stage()
            if self.recordingOn:
                self.record_sigs(self.precon_list[self.precon_ind],result,iteration,sim_time)
            if self.precon_ind==len(self.precon_list)-1:
                self.precon_checked = 1
            
            if result==1:
                if self.precon_ind==len(self.precon_list)-1:
                    self.precon_satisfied = 1
                else:
                    self.precon_ind += 1
            elif result==-1:
                if self.is_a_copy==1:
                    self.kill_me = 1 # Signal for this copy's parent to kill it; we don't want
                                     # copies to reset once invalidated.
                else:
                    self.precon_ind = 0; # Reset AM.
            # else:
                # result==0. Do nothing and let this stage repeat on next sim-step.

        
        # Postcon stuff:
        if self.precon_satisfied: # ...even if it's only just been satisfied.

            # Find which postcon stage we're on and evaluate it:
            stage = getattr(self, self.postcon_list[self.postcon_ind])
            result = stage()
            if self.recordingOn:
                self.record_sigs(self.postcon_list[self.postcon_ind],result,iteration,sim_time)
            if self.postcon_ind==len(self.postcon_list)-1:
                self.postcon_checked = 1


            if result==1:
                if self.postcon_ind==len(self.postcon_list)-1:
                    # Postcon (and, thus, assertion) satisfied (not
                    #   necessarily for the whole simulation trace).
                    self.postcon_ind = 0       # Reset.
                    self.precon_ind = 0        # ""
                    self.precon_satisfied = 0  # ""
                else:
                    self.postcon_ind += 1
            elif result==-1:
                self.postcon_checked = 1
                self.postcon_violated = 1  
                # Reset AM in case it's not fatal (i.e. sim keeps running):
                self.precon_ind = 0
                self.postcon_ind = 0
                self.precon_satisfied = 0
            # else:
                # result==0. Do nothing and let this stage repeat on next sim-step.



    def record_sigs(self,stage,result,iteration,sim_time):
        # Only the original parent AM can write to file.  So
        # first we have to gather up results from the children (copies).
        # This should work fine as long as the children are stepped before
        # record_sigs is called (see step()).
        
        self.toPassUp = []
        for ind,cp in enumerate(self.AMCopies):
            for entry in cp.toPassUp:
                self.toPassUp.append(entry)
            
        if self.is_a_copy==0:
            for kid in self.toPassUp:
                self.f.write(str(kid[0]))
                self.f.write(" ")
                self.f.write(str(kid[1]))
                self.f.write(" ")
                self.f.write(kid[2])
                self.f.write(" ")
                self.f.write(str(kid[3]))
                self.f.write("\n")

            self.f.write(str(iteration))
            self.f.write(" ")
            self.f.write(str(sim_time))
            self.f.write(" ")
            self.f.write(stage)
            self.f.write(" ")
            self.f.write(str(result))
            self.f.write("\n")
                
        else:
            # Add the signal from this instance to those of its children, to be
            # passed to the parent:
            self.toPassUp.append([iteration,sim_time,stage,result])
        


    def fork(self):
        """
            Deepcopy this AM instance and let the copy run one
                   sim-step behind the original.
        """
        self.AMCopies.append(deepcopy(self))
        # Initialise copy-specific flags:
        self.AMCopies[-1].is_a_copy = 1
        self.AMCopies[-1].kill_me = 0    



    def _consume_child_flags(self,child_ind):
        # Propagate flags up from a copy of this AM:
        if self.AMCopies[child_ind].precon_checked==1:
                self.precon_checked = 1
        if self.AMCopies[child_ind].postcon_checked==1:
                self.postcon_checked = 1
        if self.AMCopies[child_ind].postcon_violated==1:
                self.postcon_violated = 1
        


    # Destructor:
    def __del__(self):

        # Merge copies with parent:
        for ind,cp in enumerate(self.AMCopies):
            self._consume_child_flags(ind)
        self.AMCopies = [];

        if not self.is_a_copy:
            # This is the original. Give a report:
            filename = self.report_file
                                        # TO DO: Create a separate file
                                        #        for each simulation run?
            fRep = open(filename, 'a')
            """ New, compact reporting """
            fRep.write("trace "+self.trace_label)
            fRep.write(" | reporting at: ")
            fRep.write(datetime.datetime.now().strftime("%Y/%B/%d %H:%M"))
            fRep.write(" | precon_checked?: "+str(self.precon_checked))
            fRep.write(" | postcon_checked?: "+str(self.postcon_checked))
            fRep.write(" | POSTCON_VIOLATED?: "+str(self.postcon_violated)+"\n")
            """ Old, verbose reporting
            fRep.write("\n")
            fRep.write("\n------------------------------------------------\n")
            fRep.write("| Report from "+self.__class__.__name__+":")
            fRep.write("\n")
            fRep.write("| reporting at: ")
            fRep.write(datetime.datetime.now().strftime("%Y/%B/%d %H:%M"))
            fRep.write("\n------------------------------------------------\n")
            if not self.precon_satisfied:
                fRep.write("| precon stage upon termination: "+str(self.precon_ind+1)+" of "+str(len(self.precon_list)))
            else:
                fRep.write("| postcon stage upon termination: "+str(self.postcon_ind+1)+" of "+str(len(self.postcon_list)))
            fRep.write("\n| precon_checked: "+str(self.precon_checked))
            fRep.write("\n| postcon_checked: "+str(self.postcon_checked))
            fRep.write("\n| postcon_violated: "+str(self.postcon_violated))
            fRep.write("\n------------------------------------------------\n")
            """
            fRep.close()

            # Print same to terminal:
            print
            print "------------------------------------------------"
            print "| Report from", self.__class__.__name__, ":"
            print "| reporting at: ", datetime.datetime.now().strftime("%Y/%B/%d %H:%M")
            print "------------------------------------------------"
            if not self.precon_satisfied:
                print "| precon stage upon termination:", self.precon_ind+1, "of", len(self.precon_list)
            else:
                print "| postcon stage upon termination:", self.postcon_ind+1, "of", len(self.postcon_list)
            print "| precon_checked:", self.precon_checked
            print "| postcon_checked:", self.postcon_checked
            print "| postcon_violated:", self.postcon_violated
            print "------------------------------------------------"
            print

