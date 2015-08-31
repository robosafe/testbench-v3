#!/usr/bin/env python
"""
This script implements the state machine of the robot element in the simulator. The types of states available are receiving signals, sending signals, reset the position of the robot, move and grab an object, sense the human, and decide what to do about the handover. The reset position assumes the robot is not holding anything.

Format of the vector of joint commands for Gazebo-ROS:
'hipRotor', 'hipFlexor', 'neckFlexor', 'neckRotor', 'leftShoulderFlexor', 'rightShoulderFlexor', 'leftShoulderAbduction',  'leftHumeralRotation', 'leftElbowFlexor','leftWristPronation', 'leftWristAbduction', 'leftWristFlexor'

Format of the vector of joint commands for the hand:
'leftThumb1', 'leftThumb2', 'leftIndex1', 'leftIndex2', 'leftMiddle1', 'leftMiddle2', 'leftAnular1', 'leftAnular2', 'leftLittle1', 'leftLittle2'

Format of the joints used for planning:
'hipRotor', 'hipFlexor', 'leftShoulderFlexor', 'leftShoulderAbduction',  'leftHumeralRotation', 'leftElbowFlexor','leftWristPronation', 'leftWristAbduction', 'leftWristFlexor'

Written by Dejanira Araiza-Illan February-March 2015.
"""

import os
import rospy
import smach
import smach_ros
import math
import random
import time
from robot_g import set_robot_joints #Script that has the Gazebo interface to move the robot
from robot_g import move_hand #Script that has the Gazebo interface to move the hand
from interface_plan import interface #Script that has the Moveit interface for planning
from bert2_simulator.msg import *
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from coverage import coverage


#global variables
reception = 0
reception2 = 0
piece = 0
location_ok = 0
pressure_ok = 0
gaze_ok = 0
piece_location = []
the_timeout = 5.0
x_time = random.randint(0, 100000) #Specify amount of seeds available
print 'Seed: '+ str(x_time)
random.seed(x_time)
timeout_time = random.randint(0, 100)
print 'Timeout:' + str(timeout_time)
start = 0
count_timeout=0

#--------------------------------------------------------------------------------------------------------------------
class Reset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        #Announce start of Reset
	pubReset = rospy.Publisher('robot_reset', Robot, queue_size=1,latch=True)
        pubReset.publish(1)
	rospy.sleep(0.1)
	#Target position of reset: centre of hand in x=0.0, y=0.2, z=0.585
	set_robot_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	move_hand('open')
	rospy.sleep(0.5)
	#Select random initial position for the robot
	set_robot_joints([0.0, 0.0, 0.0, 0.0, random.uniform(-0.5,0.5), 0.0, 0.0, random.uniform(-0.5,0.5), 0.0, 0.0, random.uniform(-0.5,0.5),0.0])
	return 'outcome1'

#--------------------------------------------------------------------------------------------------------------------
class Receive(smach.State): #In the real robot the "human signals" node would be implemented in hardware
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'])

    def execute(self, userdata):
	global reception
	global count_timeout
	reception = 0
	rospy.sleep(0.2)
	rospy.Subscriber("human_signals", Human, callback_signals) 
	if reception == 1:
		return 'outcome1'
        elif count_timeout<500:
        	count_timeout += 1
		return 'outcome2'
	else:
		print 'Time out in loop'
		pubdone = rospy.Publisher('done', Int8, queue_size=1,latch=True)
		pubdone.publish(1)
		rospy.sleep(0.1)
		return 'outcome3'

def callback_signals(data):
	global reception
    	if data.activateRobot==1 and data.humanIsReady==0: #activateRobot signal
		reception = 1
    	elif data.activateRobot==0 and data.humanIsReady==1: #humanIsReady signal
		reception = 1

#--------------------------------------------------------------------------------------------------------------------
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):		
	#Path planning towards the piece
	theplans = interface([-0.5,0.0,-0.75,0.0,1.39,0.0,0.0,-0.5,0.0])
	pubplan = rospy.Publisher('plan_done', Int8, queue_size=1,latch=True)
	pubplan.publish(1)
	rospy.sleep(0.1)
	for i,plan in enumerate(theplans):
		set_robot_joints(plan)
	theplans = interface([-1.15,0.05,-0.75,1000.0,1.39,1000.0,1000.0,-0.5,1000.0])
	pubplan = rospy.Publisher('plan_done', Int8, queue_size=1,latch=True)
	pubplan.publish(1)
	rospy.sleep(0.1)
	for i,plan in enumerate(theplans):
		set_robot_joints(plan)
	theplans = interface([-1.15,0.05,-0.48,1000.0,1.39,1000.0,1000.0,-0.5,1000.0])
	pubplan = rospy.Publisher('plan_done', Int8, queue_size=1,latch=True)
	pubplan.publish(1)
	rospy.sleep(0.1)
	for i,plan in enumerate(theplans):
		set_robot_joints(plan)
	move_hand('close')
	rospy.sleep(0.5)
	hand = rospy.Publisher('robot_gripper', Int8, queue_size=1,latch=True)
	hand.publish(1)
	rospy.sleep(0.2)
	pubpress = rospy.Publisher('pressure_e1', Int8, queue_size=1,latch=True)
	pubpress.publish(1)
	rospy.sleep(0.2)
	hand.publish(0)
	#Path planning towards goal location
	theplans = interface([0.0,0.05,-0.75,0.0,1.39,0.0,0.0,-0.5,0.0])
	pubplan = rospy.Publisher('plan_done', Int8, queue_size=1,latch=True)
	pubplan.publish(1)
	rospy.sleep(0.1)
	#print theplans
	for i,plan in enumerate(theplans):
		set_robot_joints(plan)
	rospy.sleep(1)
	return 'outcome1'
	

#--------------------------------------------------------------------------------------------------------------------
class Send(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
	global pubsignals
	rospy.sleep(0.5)
	pubsignals = rospy.Publisher('robot_signals', Robot, queue_size=1,latch=True)
	pubsignals.publish(1)	#RobotIsReady
	rospy.sleep(0.5)
	global start
	start=time.time()
	return 'outcome1'

#--------------------------------------------------------------------------------------------------------------------
class Sense(smach.State): #In the real robot, this "sensors" node would refer to collecting and analysing info from yarp and publish results
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'])

    def execute(self, userdata):
	global reception2
	global start
	reception2 = 0
	rospy.sleep(0.2)
	rospy.Subscriber("sensors", Sensors, callback_sensors)
	if (time.time()-start) >= timeout_time:
		
		return 'outcome3'
	else:
		if reception2 == 1:
        		return 'outcome1'
		else:
			return 'outcome2'	

def callback_sensors(data):
	global reception2
	global gaze_ok
	global location_ok
	global pressure_ok
	if data.gaze==1:
		gaze_ok = 1
	else:
		gaze_ok = 0
	if data.location==1:
		location_ok = 1
	else:
		location_ok = 0
	if data.pressure == 1:
		pressure_ok = 1
	else:
		pressure_ok = 0
	reception2 = 1

#------------------------------------------------------------------------------------------------------------------
class WaitLong(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
	rospy.sleep(3)
	return 'outcome1'

#---------------------------------------------------------------------------------------------------------------------
class Timeout(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
	print 'Robot timed out'
	pubdone = rospy.Publisher('done', Int8, queue_size=1,latch=True)
	pubdone.publish(1)
	rospy.sleep(0.1)
	return 'outcome1'

#--------------------------------------------------------------------------------------------------------------------
class Decide(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
    	pubdecide = rospy.Publisher('decide', Int8, queue_size=1,latch=True)
	pubdecide.publish(1)
	rospy.sleep(0.1)
	pubgpl = rospy.Publisher('gpl_is_ok', Int8, queue_size=1,latch=True)
	if gaze_ok == 1 and pressure_ok == 1 and location_ok == 1:
		print 'GPL is OK'
		pubgpl.publish(1)
		rospy.sleep(0.1)
		return 'outcome1'

	else:
		print 'GPL is not OK'
		pubgpl.publish(0)
		rospy.sleep(0.1)
		print 'Interaction done'
		pubdone = rospy.Publisher('done', Int8, queue_size=1,latch=True)
		pubdone.publish(1)
		rospy.sleep(0.2)
		return 'outcome2'
	
#-------------------------------------------------------------------------------------------------------
class Release(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
       
    def execute(self, userdata):
    	print 'Piece is released'
    	move_hand('open')
	rospy.sleep(0.5)
	pubpress = rospy.Publisher('pressure_e1', Int8, queue_size=1,latch=True)
	pubpress.publish(0)
	rospy.sleep(0.1)
    	pubrel = rospy.Publisher('resetpiece', Int8, queue_size=1,latch=True)
    	pubrel.publish(1)
    	rospy.sleep(0.1)
    	print 'Interaction done'
    	pubdone = rospy.Publisher('done', Int8, queue_size=1,latch=True)
	pubdone.publish(1)
	rospy.sleep(0.1)
        return 'outcome1'

#--------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------
def main():
	rospy.init_node('robot', anonymous=True) #Start node first
	
	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['interactionDone'])
        
   	# Open the container
   	with sm:
		#Reset state
		smach.StateMachine.add('Reset', Reset(), 
                transitions={'outcome1':'Receive1'})

		#Receive signal
		smach.StateMachine.add('Receive1', Receive(), transitions={'outcome1':'Move_hand','outcome2':'Receive1','outcome3':'interactionDone'})
		
		#Move robot towards piece (path planning)
		smach.StateMachine.add('Move_hand', Move(), transitions={'outcome1':'Send1'})

		#Send signal informHumanOfHandover
		smach.StateMachine.add('Send1', Send(), transitions={'outcome1':'Receive2'})

		#Receive signal humanIsReady
		smach.StateMachine.add('Receive2', Receive(), transitions={'outcome1':'Wait0','outcome2':'Receive2','outcome3':'interactionDone'})

		#Wait for human
		smach.StateMachine.add('Wait0', WaitLong(), transitions={'outcome1':'Sensing'})		

		#Sense human with timeout
		smach.StateMachine.add('Sensing', Sense(), transitions={'outcome1':'Decide','outcome2':'Sensing','outcome3':'Timeout'})
		smach.StateMachine.add('Timeout',Timeout(), transitions={'outcome1':'interactionDone'}) 

		#Decide what to do
		smach.StateMachine.add('Decide', Decide(), transitions={'outcome1':'Release','outcome2':'interactionDone'})
		
		#Release piece
		smach.StateMachine.add('Release', Release(), transitions={'outcome1':'interactionDone'})

	# Execute SMACH plan
    	outcome = sm.execute()
	

#---------------------------------------------------------------------------------------------------
if __name__ == '__main__':
	cov = coverage()
	cov.start()
	try:
    		main()
    		cov.stop()
		cov.html_report(directory='covhtml')
	except rospy.ROSInterruptException: #to stop the code when pressing Ctr+c
        	pass
