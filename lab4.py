import rospy, math, tf, numpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid,  Path, Odometry
from tf.transformations import euler_from_quaternion

class Node:
	def __init__(self, x, y, gCost, fCost, cameFrom):
		self.x = x
		self.y = y
		self.gCost = gCost
		self.fCost = fCost
		self.cameFrom = cameFrom

	def calcgCost(self):
		return manhattan(self)

	def calchCost(self):
		return eucl(self)
	
	def calcfCost(self):
		return self.gCost + eucl(self)

	def isValid(self):
		ret = bool(getCellValue(self.x, self.y) < 50)
		return ret

#returns data (probability) of map cell given (x,y) coord
def getCellValue(x,y):
    global mapdata
    
    #get map info
    cols = mapdata.info.width
    rows = mapdata.info.height
   
    index = (y * cols) + x #zero indexed, (y * cols) represents first number of each row, then add x (the column)
    return mapdata.data[index] 

def pointToIndex(x,y):
    global mapdata
    
    #get map info
    cols = mapdata.info.width
    rows = mapdata.info.height
    #res = grid.info.resolution #m/cell, might come in handy later
   
    index = (y * cols) + x #zero indexed, (y * cols) represents first number of each row, then add x (the column)
    return index 


def indexToPoint(index):
	global mapdata
	#gets map info
	cols = mapdata.info.width
	rows = mapdata.info.height

	y = index % cols
	x = index // cols
	node = node()
	node.x = x
	node.y = y
	return node


def astar(start, goal):													# returns zero if no path from start to goal otherwise returns path from start to goal
	global openSet
	global closedSet

	current = start														
	
	closedSet = []     													# Already Evaluated Nodes initialized to zero
	openSet = [start]												# Unevaluated but discovered nodes 
	
	neighbors = getNeighbors(start)

	#add all nodes to openset that open?

	while openSet:

		#get lowest cost from openSet
		current = lowestFcost(openSet)														# goes to the node in openset having the lowest fCost

		 										# then return the path back to the start

		openSet.remove(current) 										# update each of the sets
		

		neighbors = getNeighbors(current)
		

		for neighbor in neighbors:
			if(current.x is goal.x) and (current.y is goal.y):											# if current is goal ->
				return repath(current)

			neighbor.gCost = current.gCost + dist_between(current,neighbor)
			
			neighbor.fCost = neighbor.gCost + eucl(neighbor)

			if inOpenSet(neighbor):
				continue
			if inClosedSet(neighbor): 									# ignores already evaluated nodes
				continue

			else:
				openSet.append(neighbor) 	


		closedSet.append(current)
	return "Error in ASTAR"
	##pass

def inOpenSet(n):
	#true = skip
	global openSet
	for q in openSet:
		if (q.x is n.x) and (q.y is n.y) and (q.fCost <= n.fCost):
			return True
	return False

def inClosedSet(n):
	#true = skip
	global closedSet
	for q in closedSet:
		if (q.x is n.x) and (q.y is n.y) and (q.fCost <= n.fCost):
			return True
	return False

#manhtn dist from start to node
def manhattan(node):												# returns the manhattan distance
	global start
	x = abs(start.x - node.x)
	y = abs(start.y - node.y)
	return x + y

def dist_between(nodeA, nodeB):
	x = abs(nodeA.x - nodeB.x)
	y = abs(nodeA.y - nodeB.y)
	dist = x + y
	return dist

def repath(node):
	global path
	path = [node]
	n = node
	last = n.cameFrom
	while(last is not 0):
		path = [last] + path
		n = n.cameFrom 
		last = n.cameFrom
	return path


def drawPath(nodelist):
	p = Path()
	p.header.frame_id = "map"
	for n in nodelist:
		posestp = PoseStamped()
		posestp.header.frame_id = "map"
		posestp.pose.position.x = n.x
		posestp.pose.position.y = n.y
		p.poses.append(posestp)
	pub_path.publish(p)



#straight dist from node to goal
def eucl(node):												# distance between node and goal
	global goal
	ix = node.x
	iy = node.y
	nx = goal.x
	ny = goal.y
	dist = math.sqrt((nx-ix)**2 + (ny - iy)**2)
	return dist

def eucl2(node, end):												# distance between node and end
	ix = node.x
	iy = node.y
	nx = end.x
	ny = end.y
	dist = math.sqrt((nx-ix)**2 + (ny - iy)**2)
	return dist


def getNeighbors(anode):
	global openSet
	global closedSet
	#gets list of neighboring nodes (4connected)

	#get x and y
	x = anode.x
	y = anode.y

	#make a node for ea direction
	nodeN = Node(x, y + 1, 0, 0, anode)
	nodeN.calcgCost
	nodeN.calcfCost

	nodeS = Node(x, y - 1, 0, 0, anode)
	nodeS.calcgCost
	nodeS.calcfCost

	nodeW = Node(x - 1, y, 0, 0, anode)
	nodeW.calcgCost
	nodeW.calcfCost

	nodeE = Node(x + 1, y, 0, 0, anode)
	nodeE.calcgCost
	nodeE.calcfCost

	nodelist = [nodeN, nodeS, nodeW, nodeE]

	for n in nodelist:
		val = getCellValue(n.x, n.y)
		if ((val > 20) or (val < 0)):
			nodelist.remove(n)
			#openSet.remove(n)
			#closedSet.append(n)
			print "OBSTACLE @ ", n.x, ",", n.y

	return nodelist


def lowestFcost(nodes):
	#iterate through node list and get lowest
	lowest_cost = 999999
	lowest_node = 0

	for n in nodes:
		if (n.fCost < lowest_cost):
			lowest_cost = n.fCost
			lowest_node = n


	#all other nodes to closed set and take off open set to not recalculate
	#for m in nodes:
	#	if m is not lowest_node:
	#		closedSet.append(m)
	#		openSet.remove(m)
	return lowest_node

def gl(msg):															# stores goal pose as global			
	global gl
	if msg:
		gl = msg.pose
		print gl

def ipose(msg):															# stores start pose as global
	global ipose
	if msg:
		ipose = msg.pose
		
		print ipose

def gcells(msg):															# stores the elements of GridCells as a global
	global cell_width
	global cell_height
	global cells
	if msg:
		cell_width = msg.cell_width
		cell_height = msg.cell_height
		cells = msg.cells

def ogrid(msg):															# stores the elements of GridCells as a global
	global mapdata

	if msg:
		mapdata = msg
		print "GOT MAP!"
		print getCellValue(166,258)
		#print str(Node(166,258,0,0,0).isValid)

def toRes(x):
	return int(x) 

###############################################

def main():	
	global ipose
	global gl
	#global pub_pose
	global start
	global goal															# main function
	


	#get start and goal from rviz
	print "ipose - x rnd", toRes(ipose.pose.position.x)
	print "ipose - x raw", ipose.pose.position.x
	#round x and y to grid resolution
	start = Node(toRes(ipose.pose.position.x), toRes(ipose.pose.position.y), 0,0,0)
	start.calcfCost
	
	#print "goal - x rnd", toRes(gl.position.x)
	goal = Node(toRes(gl.position.x), toRes(gl.position.y),0,0,0)
	#pub_pose.publish(ipose)
	path = astar(start,goal)

	printPath(path)
	drawPath(path) #publishes

	wp = makeWaypoints(path)
	printPath(wp)
	pubWaypoints(wp)

	#travel to waypoints
	for w in wp:
		res_scale = 20
		ps = PoseStamped()
		ps.header.frame_id = "map"
		ps.pose.position.x = w.x / res_scale
		ps.pose.position.y = w.y / res_scale 
		print "going to pose: ", "(",w.x,",", w.y,")"
		navToPose(ps)
		print "at goal!"

		#sense when @ goal

	print "DONE!"




#################################################

def makeWaypoints(apath):
	waypoints = []
	prev1 = apath[1] # start of path...prev2, prev1, current,...
	prev2 = apath[0]
	curr = apath[2]

	for x in range(2,len(apath)):
		curr = apath[x]
		prev1 = apath[x - 1]
		prev2 = apath[x - 2]
		d = eucl2(curr, prev2)
		if d < 2:
			print "TURN!"
			waypoints.append(prev1)
	return waypoints

def pubWaypoints(nodes):
	global pub_points
	g = GridCells()
	g.header.frame_id = "map"
	g.cell_height = 1
	g.cell_width = 1


	for n in nodes:
		point = Point()
		point.x = n.x
		point.y = n.y
		point.z = 0
		g.cells.append(point)
	pub_points.publish(g)


# Add additional imports for each of the message types used
def publishTwist(linearVelocity, angularVelocity):
    #send a twist message
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)
    print "PUBLISH!"

#This function accepts a speed and a distance for the robot to move in a straight line (IN METERS)
def driveStraight(speed, distance):

    global pose
    
    print "DRIVING!"
    
    initialX = pose.pose.position.x
    initialY = pose.pose.position.y
    atTarget = False
    
    #loop until distance between robot and initial frme is distance specified
    
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.pose.position.x
        currentY = pose.pose.position.y
        #calc current distance
        deltaX = currentX - initialX
        deltaY = currentY - initialY
        currentDist = math.sqrt((deltaX ** 2) + (deltaY ** 2))
       
        print "curX: " + str(currentX) + " dX: " + str(deltaX)
       #check if distance reached yet
        if (currentDist >= distance):
            atTarget = True
            publishTwist(0,0) #stop
        else:
            publishTwist(speed, 0) #keep driving
            rospy.sleep(0.15)
            print "GO!"
              
#Accepts an angle RADIANS and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose
    
    speed = 0.2 #rad/s
    done = True
    #get current angular position
    theta = pose.pose.orientation.z
    error = angle - theta #calc difference
    
    
    #determine direction to turn
    if (error < 0):
        #turn CW by reversing ang speed
        speed = speed * (-1)
    
    while ((abs(error) >= 0.035) and not rospy.is_shutdown()):
        publishTwist(0,speed)
        rospy.sleep(0.15)
        #for debug
        print "pose: " + str(math.degrees(pose.pose.orientation.z))
        #update error
        error = angle - pose.pose.orientation.z
        
    publishTwist(0,0)
    
#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    #gather some intial info
    global pose
    
    dx = goal.pose.position.x - pose.pose.position.x
    dy = goal.pose.position.y - pose.pose.position.y
    c = math.sqrt((dx ** 2) + (dy ** 2)) #straight distance
    
    print "spin!"
    #first angle to rotate to
    theta1 = math.atan(dy / dx)
    rotate(theta1)
    
    print "move!"
    driveStraight(0.3, c)

    print "spin!"
    theta2 = goal.pose.orientation.z #get angle from goal
    rotate(theta2)
    #note: angles are all absolute

    print "done"


#updates global variables for odometry
def timerCallback(event):
    global pose
    pose = PoseStamped()
    
    #note: odom for stage, map for real turtlebot?
    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(2.0))
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    
    #store position info into pose
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]

    
    odomW = orientation
    q = [odomW[0],odomW[1],odomW[2],odomW[3]]
    roll,pitch,yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.pose.orientation.z = yaw
	


def printPath(path):
	for p in path:
		print "(",p.x,",",p.y,")->"


if __name__ == '__main__':
	global goal
	global start
	global spose
	#global GridCells
	global mapdata

	global pub_path
	global pub_points
	global pub_pose
	global pub_goal
	#from lab 2:
	global pub
	global pose
	global odom_tf
	global odom_list
	global length
    
    
    #pose = pose()
	length = 0.23 #m

	spose = PoseStamped()
	#GridCells = GridCells()

	rospy.init_node("lab4")

########PUBLISHERS
	pub_path = rospy.Publisher('/astarpath', Path, queue_size=1)
	rospy.sleep(rospy.Duration(1, 0))
	pub_points = rospy.Publisher('/ourwaypoints', GridCells, queue_size=1)
	rospy.sleep(rospy.Duration(1, 0))
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10) # Publisher for commanding robot motion
	pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
	#pub_pose = rospy.Publisher('/arrow', PoseStamped, queue_size = 1)
 
########SUBSCRIBERS
	goal_sub = rospy.Subscriber('/goalpose', PoseStamped, gl, queue_size=1)
	init_sub = rospy.Subscriber('/startpose', PoseWithCovarianceStamped, ipose, queue_size=1)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, ogrid, queue_size=1)	
    #bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
	#goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()

    #make the robot keep doing something...
	rospy.Timer(rospy.Duration(0.01), timerCallback)
    # Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(8, 0))

	print "begin main"
	
	main()

	print "end main"
