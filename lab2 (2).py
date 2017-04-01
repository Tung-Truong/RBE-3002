#!/usr/bin/env python
import rospy, tf
from kobuki_msgs.msg import BumperEvent
# Add additional imports for each of the message types used
import numpy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


#function that just sends linear and angular veloctiy to the turtlebot on a twist message
def SendMoveMsg(linearVelocity, angularVelocity):
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)
    
#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global currentX
    global currentY
    global theta
    #find desired x and y position
    whereY = goal.pose.position.y
    whereX = goal.pose.position.x
	
    #find desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    whereT = yaw * (180.0/math.pi)
	
    #compute distance to goal with quadratic formula
    distance = math.sqrt(math.pow((whereX - currentX), 2) + math.pow((whereY - currentX), 2))
    adjustedX = goal.pose.position.x -currentX
    adjustedY = goal.pose.position.y -currentY
    print goal.pose.position.x, goal.pose.position.y
    print currentX, currentY
    print adjustedX, adjustedY
    print "spin!"
	#find initial turn amount
    initialTurn = (math.atan2(adjustedY, adjustedX) * (180 /math.pi)) - theta
    print str(initialTurn)
	#rotates to face goal
    rotateDegrees(initialTurn)
    print "move!"
	
	#drive to goal
    driveSmooth( .30, distance)
    print "spin!"
	
	#calculates remaining degrees to turn to go to final pose
    remainingTurn = whereT - theta
    rotateDegrees(remainingTurn)
    print "done"


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	driveStraight(1, .6)
	rotate(-90)
	driveStraight(1, .5)
	rotate(135)





#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	global pub
	
	r = 3.5/100.0  #wheel_radius
	b = 23.0/100.0 #wheel_base
	
	#calculates linear velocity of the base based on wheel speed
	linear_vel = (r / .5) * (u1 + u2) 
	
	#calculates angular velocity of the base based on the wheels
	angular_vel = (r / b) * (u1 + u2)
	
	#creates two messages
	move_msg = Twist(); 
	stop_msg = Twist();
	
	#fills message with data
	move_msg.linear.x = linear_vel
	move_msg.angular.z = angular_vel
	
	#fills stop message with data
	stop_msg.linear.x = 0
	stop_msg.angular.z = 0
	
	#while specified time has not passed, send twist messages and make stuff happen
	now = rospy.Time.now().secs
	while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
		pub.publish(move_msg)
	pub.publish(stop_msg)



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
   global pose
   #set origin
   xInitial =  pose.position.x
   yInitial =  pose.position.y
	
   #flag to keep track if robot is at the position
   atTarget = False
   while (not atTarget and not rospy.is_shutdown()):
       	currentX = pose.position.x
   	currentY = pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - xInitial), 2) + math.pow((currentY - yInitial), 2))
        print currentDistance
        if (currentDistance >= distance):
           	atTarget = True
            	SendMoveMsg(0, 0)
        else:
            	SendMoveMsg(speed, 0)
           	rospy.sleep(0.15)

	           
#this function works like driveStraight except it slowly speeds up and down
def driveSmooth(speed, distance):
    global pose
    
    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False
    rampSpeed = 0.0
    sleepTime = 0.05
    rampPercentage = 0.3
    step = speed / ((rampPercentage * (distance / speed)) / sleepTime)
    print "Step size: " + str(step)
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        if (currentDistance >= distance):
            atTarget = True
            SendMoveMsg(0, 0)
        else:
            if ((distance - currentDistance) <= distance * rampPercentage and rampSpeed >= 0):
                rampSpeed -= step
                SendMoveMsg(rampSpeed, 0)
            elif ((distance - currentDistance) >= distance * (1.0 - rampPercentage) and rampSpeed <= speed):
                rampSpeed += step
                SendMoveMsg(rampSpeed, 0)
            else:
                SendMoveMsg(speed, 0)
            rospy.sleep(sleepTime)
	
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global pose
    global odom_list
    if (angle > 180 or angle < -180):
        print "invalid angle"
    vel = Twist();
    done = True 
    
    error = angle-math.degrees(pose.orientation.z)
    if (angle >= 0):
        vel.angular.z = .35
    if (angle < 0):
        vel.angular.z = -.35
    
    angle = angle + math.degrees(pose.orientation.z)
    if (angle >180):
        angle = angle -360
    if (angle < -180):
        angle = angle +360
    while ((abs(error) >= 2) and not rospy.is_shutdown()):
        error = angle-math.degrees(pose.orientation.z)
        rospy.sleep(.01)
    vel.angular.z = 0.0

#converts an angles into degrees for use in rotate(angle)
def rotateDegrees(angle):
    rotate(angle* (math.pi/180))

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	w = speed / radius
	Vr = w * (radius + .5 * length)
	Vl = w * (radius - .5 * length)
	arcTime = angle / w
	spinWheels(Vr, Vl, arcTime)
	#kinematics of turtle bot go here to turn in arc





#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        print "Bumper was pressed"
	executeTrajectory()


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def readOdom(msg):
    global pose
    global currentX
    global currentY
    global theta
    global odom_list
    global odom_tf
    try:
        #converts to global coordiantes
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
        roll, pitch, yaw = euler_from_quaternion(rot)
        theta = yaw * (180.0/math.pi)
        currentX = trans[0]
        currentY = trans[1]
    except:
        print "Waiting for tf..."


def tCallback(event):
    global pose
    global currentX
    global currentY
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    currentX=position[0]
    currentY=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('ttruong)

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
    pose = Pose()
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub =rospy.Subscriber('move_base_simple/goal', PoseStamped, navToPose, queue_size=1)
    sub = rospy.Subscriber('/odom', Odometry, readOdom)
    rospy.Timer(rospy.Duration(.01), tCallback)
    odom_list = tf.TransformListener()
    rospy.sleep(2000)
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    #make the robot keep doing something...
    while not rospy.is_shutdown():
        rospy.sleep()

    # Make the robot do stuff...
    print "Lab 2 complete!"

