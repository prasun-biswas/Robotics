#!/usr/bin/env python

#Note: Open this file with Text Editor to copy paste code from and to your computer

import rospy
import brickpi3 
import numpy as np
import math
import time
from std_msgs.msg import String, Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class

# TO DO. Set your number of joints here
nrOfJoints=3
qNow=np.array([0.0] * nrOfJoints) # This will be our joints state, global access in this script
# This int can be used to check wether there are pending requests to be attended by the planner or not
pendantRequest = 0
# TO DO. Student definitions, customize this with your data
#Specify ports you are using for your JOINTS in order, size must be the same than nrOfJoints
jointsOrder = [BP.PORT_A, BP.PORT_B, BP.PORT_C]
# TO DO. Again, same size than your kinematic model in the planner or failure will be upon you, conversion from radians or mm to degrees of motors
jointsScale = np.array([3*180/math.pi, 4*180/math.pi, 36]) #This values are a random example

# callbackPlanner(data) This function is triggered when a ROS message is being received while in ratePendant.sleep()
# This function takes what the trajectory planner sends, according to the requests you do from this python code
# Also, it uses that information to command the motors, iteratively until they reach the jointPoints given a degrees precision and advances to the next
# This function can be modified with your own control way, and parameters.
# TO DO. If you don't feel a good control over your robot, you should modify this function
def callbackPlanner(data):
	global BP # We need to specify the globals to be modified
	global qNow 
	global pendantRequest 

	# TO DO. How many degrees of precision for each motor do you want?
	degreesThreshold = np.array([5, 5, 5, 5]) #You can also specify the same for every joint as done in kp and ki
	kp = [15] * len(data.points[0].positions) #You can also specify a kp for each joint as done in degreesThreshold
	ki = [25] * len(data.points[0].positions) 
	kd = [20] * len(data.points[0].positions) #kp ki and kd are proportional, integral and derivative constants for control respectively
	h = len(data.points) # h contains the number of jointPoints
	w = len(data.points[0].positions) # w contains number of joints	
	Matrix = [[0 for x in range(w)] for y in range(h)]
	for idy, point in enumerate(data.points):
		# Flag for waiting until motors reach each point
		allReached = False
		# This array of flags indicates that each single joint arrived to the destination
		qReached=[False] * len(data.points[0].positions)
		timeBase= time.time()		
                while not allReached:
                        if rospy.is_shutdown():
                                BP.reset_all()
                                return
                        for idx, jointPos in enumerate(point.positions):
                                rospy.loginfo("Joint %s position to send %s", idx, jointPos)
                                #Tell the motors to move to the desired position
                                K = kp[idx] + ki[idx] * (time.time()-timeBase)
                                if K > 100: K = 100
                                BP.set_motor_position_kp(jointsOrder[idx], K) # We can cheat the control and add an integrator varying kp over time
                                BP.set_motor_position_kd(jointsOrder[idx], kd[idx])
                                BP.set_motor_position(jointsOrder[idx], jointPos*jointsScale[idx]) # Note how jointsScale is used!
                                for idx, jointPort in enumerate(jointsOrder):
                                        if abs(BP.get_motor_encoder(jointPort) - int(round(data.points[idy].positions[idx]*jointsScale[idx]))) < degreesThreshold[idx]:
                                                qReached[idx]=True					
                                        if all(flag == True for flag in qReached):
                                                allReached=True

	# Now the request has been attended, let's refresh qNow values
	for idx, q in enumerate(jointsOrder):
                 # TO DO: Maybe it is better to put in this line below the last jointPoint given by the trajectory planner
		qNow[idx] = float(BP.get_motor_encoder(q))/float(jointsScale[idx]) 
	#Request has been attended, success!
	pendantRequest = pendantRequest - 1
	rospy.loginfo("Request attended")

#TO DO: Define the callback of the variant topic subscription here

# ROS publisher and subscriber
rospy.init_node('task', anonymous=True)
ratePendant = rospy.Rate(0.5) #Hz
rateLoop = rospy.Rate(100) #Hz
pub = rospy.Publisher('request', String, queue_size=100)
subP = rospy.Subscriber('trajectory', JointTrajectory, callbackPlanner)

#TO DO: Subscribe to the variant topic in a similar way as the line before specyfing its callback, check the MATLAB script for the message type
# Subscribe to the variant topic in a line, below, in a similar way that it subscribes to trajectory topic upwards

# Write the callback code here, (define the callback function), you can take the other callback upwards as example. 

####################################################


# This function sends the ros message (your request)
	#Format of the request : 
	# A char , L or J for traj type, "space" 
	# Current state of the joints, separated by commas
	# Target position in world frame, separated by commas "space" 
	#rotation in EulerZYX, separated by commas "space" 
	# Number of points to interpolate
def sendRequest(typeTraj, qNow, xyz, eulerzyx, points = 1):
	global pendantRequest
	# Trajectory type, pass it as a 	
	request_str = typeTraj + ' ' 
	# QNow, pass it as a numpy array
	for q in qNow:
		request_str = request_str + str(q) + ','
	request_str = request_str[:-1] + ' ' # Remove last comma, add space for next word
	# XYZ, pass it as a numpy array
	for u in xyz:
		request_str = request_str + str(u) + ','
	request_str = request_str[:-1] + ' ' 
	# EulerZYX, pass it as a numpy array
	for a in eulerzyx:
		request_str = request_str + str(a) + ','
	request_str = request_str[:-1] + ' ' 
	# joint Points to be returned by the planner, pass it as an int number
	request_str = request_str + str(points)
	# Send the request_str
	pub.publish(request_str)
	# If we send request, it is pendant to be answered
	pendantRequest = pendantRequest + 1 

# This is the main function
def task():
        global BP #Initialize the state of our joints, let's them be global to be refreshed on every movement #This variable should have their value in radians or as interpreted by your model in the planner
                
        # TO DO. Student code should start here:                
        BP.reset_all() #Ensure the moveposset_motor_positionition previous command is not working       
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) #Reset encoder A
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #Reset encoder B
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #Reset encoder C
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) #Reset encoder C
        # Tell the motors not to be in a hurry, slowly is more accurate
        BP.set_motor_limits(BP.PORT_A + BP.PORT_B + BP.PORT_C + BP.PORT_D, 30) 
        #TO DO:
        # Request the planner to move our robot, you can start adding code here to make the first trials, but at the end, these movement requests
        # should be send in the callback listening matlab commands.
        # Be sure that the point is reachable, orientation can be ignored, as always, an example is given.
                
        ratePendant.sleep() # Wait a while before requesting
                
        xyz=np.array([20.0, 20.0, 25.0])
        eulerzyx=np.array([0.0, 0.0, 0.0]) # Don't care about rotation now
        sendRequest('J', qNow, xyz, eulerzyx, 1) #In theory one point for joint trajs is ok
        while pendantRequest > 0:
                ratePendant.sleep()
                pass
                
        xyz=np.array([20.0, -20.0, 15.0])
        sendRequest('L', qNow, xyz, eulerzyx, 20) #The most points the most accurate it should be
        while pendantRequest > 0:
                ratePendant.sleep()
                pass
                
        xyz=np.array([-20.0, -20.0, 5.0])
        sendRequest('L', qNow, xyz, eulerzyx, 20) 
        while pendantRequest > 0:
                ratePendant.sleep()
                pass
                
        xyz=np.array([-20.0, 20.0, 25.0])
        sendRequest('L', qNow, xyz, eulerzyx, 20) 
        while pendantRequest > 0:
                ratePendant.sleep()
                pass
                
        xyz=np.array([20.0, 20.0, 15.0])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while not rospy.is_shutdown():
                ratePendant.sleep()
                rospy.loginfo("Pendant requests: %s", pendantRequest)




if __name__ == '__main__':
	try:
		task()
	except (rospy.ROSInterruptException, KeyboardInterrupt):
                pass
        finally:
                rospy.loginfo("KeyboardInterrupt received")
                BP.reset_all()		
