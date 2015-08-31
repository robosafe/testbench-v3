#!/usr/bin/env python

# This script checks the distance between two links in Gazebo, and is used by various
# assertion monitors.
#
# Created by David Western, April 2015.

import rospy
import sys
from gazebo_msgs.srv import GetLinkState

def get_link_loc(link_name):
    
    rospy.wait_for_service('gazebo/get_link_state')
    try:
        state_sp = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
        state = state_sp(link_name,'world')
        return state.link_state.pose.position
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def compare_locs(loc1,loc2):

    xDif = loc1.x-loc2.x
    yDif = loc1.y-loc2.y
    zDif = loc1.z-loc2.z

    if zDif<0 and zDif>-0.3:
        zOK = 1
    else:
        zOK = 0
    xOK = 1
    yOK = 1
    print loc1.z
    print loc2.z
    print zDif

    if xOK and yOK and zOK:
        return 1
    else:
        return 0

def get_dist(loc1,loc2):

    return ((loc1.x-loc2.x)**2+(loc1.y-loc2.y)**2+(loc1.z-loc2.z)**2)**0.5

def check(link1,link2):

    # Get hand location
    loc1 = get_link_loc(link1)

    # Get object location
    loc2 = get_link_loc(link2)

    # Compare
    dist = get_dist(loc1,loc2)

    return dist
