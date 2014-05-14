#!/usr/bin/env python
import hubo_ach as ha
import ach
import sys
import time
from ctypes import *
import math
# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#Below are the length's of the Legs of the robot
l1=340.03
l2=340.38
l3=114.97
anglee=0
# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()
for arg in sys.argv:
    argsteplength=arg
# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)
#This uses two IK formulas to lean the robot depending on the leg it has moved forwards The name's within the method may
#be missleading
#Main first gets the robot in the ready position and begins the initial lean and then it runs a loop of the walking
#code
def main():
	ref.ref[ha.LKN] = 0
	ref.ref[ha.RKN] = 0
	ref.ref[ha.LAP] = 0
	ref.ref[ha.RAP] = 0
	ref.ref[ha.LHP] = 0
	ref.ref[ha.RHP] = 0
	# Print out the actual position of the LEB
	#print "Joint = ", state.joint[ha.LEB].pos
	l1=340.03
	l2=340.38
	l3=114.97
	xe=(l1+l2+l3)*.85
	ye=0
	anglee=0
	leg=0
	i=0
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	#print xw
	#print yw
	angle2=math.pi-math.acos((l1**2+l2**2-xw**2-yw**2)/(2*l1*l2))
	#angle2=2*math.atan(math.sqrt(((l1+l2)**2-(xw**2+yw**2))/(-1*(l1-l2)**2+(xw**2+yw**2))))
	angle1=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3=anglee-angle1-angle2
	# Print out the Left foot torque in X
	#print "Mx = ", state.ft[ha.HUBO_FT_L_FOOT].m_x
	#print angle1
	#print angle2
	# Write to the feed-forward channel This is the set up phase
	for l in range(0,1200):
		time.sleep(.001)
		ref.ref[ha.LKN]=ref.ref[ha.LKN]+angle2/1200
		ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2/1200
		ref.ref[ha.LAP]=ref.ref[ha.LAP]+(angle3)/1200
		ref.ref[ha.RAP]=ref.ref[ha.RAP]+(angle3)/1200
		ref.ref[ha.LHP]=ref.ref[ha.LHP]+(angle1)/1200
		ref.ref[ha.RHP]=ref.ref[ha.RHP]+(angle1)/1200
		r.put(ref)
	#Prepare for lean manuever
	height=l1*math.cos(angle1)+l2*math.cos(angle2+angle1)+l3*math.cos(angle3+angle1+angle2)
	leanangle=math.asin(83.43/height)
	ref.ref[ha.LHR] = 0
	ref.ref[ha.RHR] = 0
	ref.ref[ha.LAR] = 0
	ref.ref[ha.RAR] = 0
	ref.ref[ha.LSR] = (math.pi)*.05
	ref.ref[ha.RSR]	= -(math.pi)*.05
	#get angles for .3 from the legs current bending state
	xe=((l1+l2+l3)*.85)-300
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	angle2lift=math.pi-math.acos((l1**2+l2**2-xw**2-yw**2)/(2*l1*l2))
	angle1lift=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3lift=anglee-angle1-angle2
	angle1liftdiff=angle1lift-angle1
	angle2liftdiff=angle2lift-angle2
	angle3liftdiff=angle3lift-angle3
	#I am making the foot lift and fall between .1m and .3m
	#So we are finding the angles so it is .1m from ground
	xe=((l1+l2+l3)*.85)-100
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	angle2down=math.pi-math.acos((l1**2+l2**2-xw**2-yw**2)/(2*l1*l2))
	angle1down=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3down=anglee-angle1-angle2
	angle1downdiff=angle1down-angle1lift
	angle2downdiff=angle2down-angle2lift
	angle3downdiff=angle3down-angle3lift
	angle1upcont=angle1lift-angle1down
	angle2upcont=angle2lift-angle2down
	angle3upcont=angle3lift-angle3down
	angle1downcont=-angle1lift+angle1down
	angle2downcont=-angle2lift+angle2down
	angle3downcont=-angle3lift+angle3down
	#All the for loops are there so the system remains stable
	for l in range(0,1500):
		time.sleep(.001)
		ref.ref[ha.LHR]=ref.ref[ha.LHR]-leanangle/1500
		ref.ref[ha.RHR]=ref.ref[ha.RHR]-leanangle/1500
		ref.ref[ha.LAR]=ref.ref[ha.LAR]+leanangle/1500
		ref.ref[ha.RAR]=ref.ref[ha.RAR]+leanangle/1500
		r.put(ref)
	#Below is a loop which makes the foot go up and down 5 times
	for loop in range(0,6):
		if loop==0:
			for looper1 in range(0,1200):
				time.sleep(.006)
				ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2liftdiff/1200
				ref.ref[ha.RAP]=ref.ref[ha.RAP]+angle3liftdiff/1200
				ref.ref[ha.RHP]=ref.ref[ha.RHP]+angle1liftdiff/1200
				r.put(ref)
			for looper2 in range(0,1200):
				time.sleep(.006)
				ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2downdiff/1200
				ref.ref[ha.RAP]=ref.ref[ha.RAP]+angle3downdiff/1200
				ref.ref[ha.RHP]=ref.ref[ha.RHP]+angle1downdiff/1200
				r.put(ref)
		else:
			for looper2 in range(0,1200):
				time.sleep(.006)
				ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2upcont/1200
				ref.ref[ha.RAP]=ref.ref[ha.RAP]+angle3upcont/1200
				ref.ref[ha.RHP]=ref.ref[ha.RHP]+angle1upcont/1200
				r.put(ref)
			for looper2 in range(0,1200):
				time.sleep(.006)
				ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2downcont/1200
				ref.ref[ha.RAP]=ref.ref[ha.RAP]+angle3downcont/1200
				ref.ref[ha.RHP]=ref.ref[ha.RHP]+angle1downcont/1200
				r.put(ref)
			print loop
	xe=(l1+l2+l3)*.85
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	angle2over=math.pi-math.acos((l1**2+l2**2-xw**2-yw**2)/(2*l1*l2))
	angle1over=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3over=anglee-angle1-angle2
	angle1overdiff=angle1over-angle1down
	angle2overdiff=angle2over-angle2down
	angle3overdiff=angle3over-angle3down
	#Below puts the foot back down
	for l in range(0,1200):
		time.sleep(.003)
		ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2overdiff/1200
		ref.ref[ha.RAP]=ref.ref[ha.RAP]+angle3overdiff/1200
		ref.ref[ha.RHP]=ref.ref[ha.RHP]+angle1overdiff/1200
		r.put(ref)
	#Below makes the robot lean
	for l in range(0,1500):
		time.sleep(.003)
		ref.ref[ha.LHR]=ref.ref[ha.LHR]+leanangle/1500
		ref.ref[ha.RHR]=ref.ref[ha.RHR]+leanangle/1500
		ref.ref[ha.LAR]=ref.ref[ha.LAR]-leanangle/1500
		ref.ref[ha.RAR]=ref.ref[ha.RAR]-leanangle/1500
		r.put(ref)
	ref.ref[ha.LSR] = 0
	ref.ref[ha.RSR]	= 0
	angle1overdiff=0-angle1over
	angle2overdiff=0-angle2over
	angle3overdiff=0-angle3over
	#Below makes the robot stand up
	for l in range(0,1200):
		time.sleep(.003)
		ref.ref[ha.LKN]=ref.ref[ha.LKN]+angle2overdiff/1200
		ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2overdiff/1200
		ref.ref[ha.LAP]=ref.ref[ha.LAP]+angle3overdiff/1200
		ref.ref[ha.RAP]=ref.ref[ha.RAP]+angle3overdiff/1200
		ref.ref[ha.LHP]=ref.ref[ha.LHP]+angle1overdiff/1200
		ref.ref[ha.RHP]=ref.ref[ha.RHP]+angle1overdiff/1200
		r.put(ref)
	# Close the connection to the channels
	r.close()
	s.close()
main()
