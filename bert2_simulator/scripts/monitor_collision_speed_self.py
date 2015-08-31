#!/usr/bin/env python
"""
Implements assertion: if 'robot collides with something else' then 'robot hand speed is less than 250mm/s'


Written by David Western, August 2015
"""
import re
import sys
import rospy
import smach
import smach_ros
import time
#from bert2_simulator.msg import *
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ContactsState

receivedflag1=0
globaltime=0
stats = open('assertion_collision_speed_self.txt','a')
fileno = 0
global coll1
global coll2

class Flag1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag1
	receivedflag1 = 0
	rospy.sleep(0.05)
	rospy.Subscriber("gazebo_contacts", ContactsState, callback1)
	if receivedflag1 == 1:
		return 'outcome1'
        else:
		return 'outcome2'
		
def callback1(data):
	global receivedflag1
        global coll1
        global coll2
        global maxRelSpd
        found = 0
        maxRelSpd = 0
        state_sp = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
        for contact in data.states:
            # Was it bert2 hitting itself?
            if not re.search('bert2',contact.collision1_name)==None:
                if not re.search('bert2',contact.collision2_name)==None:
                    found = 1
                    # Get relative speed:
                    state = state_sp(contact.collision1_name,'world')
                    vel1 = state.link_state.twist.linear
                    state = state_sp(contact.collision2_name,'world')
                    vel2 = state.link_state.twist.linear
                    relSpd = ( (vel1.x-vel2.x)**2 + (vel1.y-vel2.y)**2 + (vel1.z-vel2.z)**2 )**0.5
                    if relSpd>maxRelSpd:
                        maxRelSpd = relSpd
                        coll1 = contact.collision1_name
                        coll2 = contact.collision2_name
                
	if found==1:
                # Bert2 hit something else
		receivedflag1 = 1
	else:
		receivedflag1 = 0
		
class Gazebo_check1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        global coll1
        global coll2
        global maxRelSpd
        # Compare:
        if maxRelSpd<0.25:
		global stats
		stats.write('Assertion_collision_speed_self at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
        else:
        	global stats
		stats.write('Assertion_collision_speed_self at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
                print coll1,"hit",coll2,"at",str(maxRelSpd*1000),"mm/s."
    	rospy.sleep(0.1)
	return 'outcome1'
		
		
def main(number):
	rospy.init_node('assertion_collision_speed_self', anonymous=True) #Start node first
	global globaltime
	globaltime = time.time()
	global fileno
	fileno = number
	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:
		#Receive signal
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Check1','outcome2':'Flag1'})

		#Check
		smach.StateMachine.add('Check1', Gazebo_check1(), 
                transitions={'outcome1':'Flag1'})


	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
