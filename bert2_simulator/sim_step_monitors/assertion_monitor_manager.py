#!/usr/bin/env python
"""
Assertion Monitor Manager

Created by David Western, June 2015.
"""

from coverage import coverage
import imp
import rospkg
import rospy
from std_msgs.msg import UInt64
from std_srvs.srv import Empty
import sys

class AMM:

   def __init__(self,AM_list_file,trace_label):
    
      # Read list of assertion monitors to run (from file?):
      rospack = rospkg.RosPack()
      path = rospack.get_path('bert2_simulator')
      path = path+'/sim_step_monitors/'
      print("--- Assertion monitors to run:")
      self.AM_names = [line.rstrip('\n') for line in open(path+AM_list_file)]
      print(self.AM_names)

      # Instantiate assertion monitors:
      self.AMs = [] # Initialise empty list of AMs.
      for idx, class_name in enumerate(self.AM_names):
          print(class_name)
          print path+class_name+'.py'
          module = imp.load_source(class_name, path+class_name+'.py')
          #module = __import__(path+class_name) # N.B. These two lines imply that we 
          class_ = getattr(module, class_name) # require the AM to be defined in a 
                                               # file with the same name as the class.
          self.AMs.append(class_(trace_label))

          # Check AM has the mandatory attributes:
          mand_attrs = ['step']
          for attr in mand_attrs:
             if not hasattr(self.AMs[idx],attr):
                rospy.logerr("Assertion monitor specification '%s' does not define the attribute \
                              '%s', which is required by AMM (assertion_monitor_manager.py). \
                              Does %s inherite from an assertion monitor base class?",
                              self.AMs[idx].__name__, attr, self.AMs[idx].__name__)
      
      # Get service
      self.unpause_gazebo = rospy.ServiceProxy('gazebo/unpause_physics',Empty)

      # Subscriber to triggers, which come on each sim step:
      rospy.Subscriber("AM_trigger", UInt64, self.trigger_AMs)  


   def trigger_AMs(self,data):

      iteration = data.data
      sim_time = rospy.get_time()
      # Step all assertion monitors:
      for idx, AM in enumerate(self.AMs):
         AM.step(iteration,sim_time)

      # Release gazebo now we've finished the checks for this step:
      #print "unpausing"
      #self.unpause_gazebo()
      # Problem: This line prevents Gazebo's pause button from working (unless you
      #          get a lucky click). 





if __name__ == '__main__':
   try:
      if len(sys.argv) < 3:
         print("usage: rosrun [package_name] assertion_monitor_manager.py AM_list_file.txt report_file_name")
      else:

         rospy.init_node('AMM')

         AMMInst = AMM(sys.argv[1],sys.argv[2])

         rospy.spin()

   except rospy.ROSInterruptException: #to stop the code when pressing Ctr+c
      pass
