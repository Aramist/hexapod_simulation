#! /usr/bin/env python

import rospy
from hexapod_msgs.msg import HexapodState, Leg


def set_state(setpoint, leg, thigh, knee, ankle):
    
    if leg == 'rf':  # Right front
        setpoint.right_front.thigh = thigh
        setpoint.right_front.knee = knee
        setpoint.right_front.ankle = ankle
    elif leg == 'rm':
        setpoint.right_middle.thigh = thigh
        setpoint.right_middle.knee = knee
        setpoint.right_middle.ankle = ankle
    elif leg == 'rr':
        setpoint.right_rear.thigh = thigh
        setpoint.right_rear.knee = knee
        setpoint.right_rear.ankle = ankle
    elif leg == 'lf':
        setpoint.left_front.thigh = thigh
        setpoint.left_front.knee = knee
        setpoint.left_front.ankle = ankle
    elif leg == 'lm':
        setpoint.left_middle.thigh = thigh
        setpoint.left_middle.knee = knee
        setpoint.left_middle.ankle = ankle
    else:  # Left rear
        setpoint.left_rear.thigh = thigh
        setpoint.left_rear.knee = knee
        setpoint.left_rear.ankle = ankle



def run():
    rospy.init_node('console_control')

    recent = raw_input().strip().lower()
    publisher = rospy.Publisher('/hexapod_interface/setpoint', HexapodState)
    setpoint = HexapodState()

    while not recent == 'quit':
       split = recent.split(' ')
       leg = split[0]
       #  Convert to radians from degrees
       thigh, knee, ankle = [3.14159265 * float(a) / 180.0 for a in split[1:]]
       set_state(setpoint, leg, thigh, knee, ankle)
       publisher.publish(setpoint)
       
       recent = raw_input().strip().lower()
