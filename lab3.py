import rospy, math, tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid, GridCells

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
		ret = (getCellValue(self.x, self.y) < 20)
		return ret

#returns data (probability) of map cell given (x,y) coord
def getCellValue(x,y):
    global mapdata
    
    #get map info
    cols = mapdata.info.width
    rows = mapdata.info.height
   
    index = (y * cols) + x #zero indexed, (y * cols) represents first number of each row, then add x (the column)
    return mapdata[index] 

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

		if(current.x is goal.x) and (current.y is goal.y):											# if current is goal ->
			return repath(current) 										# then return the path back to the start

		openSet.remove(current) 										# update each of the sets
		closedSet.append(current)

		neighbors = getNeighbors(current)

		for neighbor in neighbors:
			
			if (neighbor in closedSet): #or (not neighbor.isValid): 									# ignores already evaluated nodes
				continue

			tentativegCost = current.gCost + 1	# the cost from start to the neighbor

			if neighbor not in openSet: 								# discovered a new node
				openSet.append(neighbor) 	

			elif (tentativegCost >= neighbor.gCost): 					# this is not a better path
				continue

			neighbor.cameFrom = current
			neighbor.gCost = tentativegCost
			neighbor.fCost = neighbor.gCost + eucl(neighbor)

	return "Error in ASTAR"
	##pass

#manhtn dist from start to node
def manhattan(node):												# returns the manhattan distance
	global start
	x = abs(start.x - node.x)
	y = abs(start.y - node.y)
	return x + y

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

#straight dist from node to goal
def eucl(node):												# distance between node and goal
	global goal
	ix = node.x
	iy = node.y
	nx = goal.x
	ny = goal.y
	dist = math.sqrt((nx-ix)**2 + (ny - iy)**2)
	return dist


def getNeighbors(anode):
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

	return nodelist


def lowestFcost(nodes):
	#iterate through node list and get lowest
	lowest_cost = 999999
	lowest_node = 0

	for n in nodes:
		if n.fCost < lowest_cost:
			lowest_cost = n.fCost
			lowest_node = n


	#all other nodes to closed set and take off open set to not recalculate
	for m in nodes:
		if m is not lowest_node:
			closedSet.append(m)
			openSet.remove(m)

	return lowest_node






# def mkGraph(rows, columns, inits): 										# makes a graph with specified number of rows and columns and initializes it to (inits)
# 	# Define blank list
# 	graph = []

# 	#Generate rows with length of 10
# 	for row in range(rows):
# 	  # Appen a blank list to each row cell
# 	  graph.append([])
# 	  for column in range(columns):
# 	    # Assign 0.5 to each row initially
# 	    graph[row].append(inits)
# 	return graph


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
		mapdata = msg.data




###############################################

def main():	
	global ipose
	global gl

	global start
	global goal															# main function
	


	#get start and goal from rviz
	start = Node(ipose.pose.position.x, ipose.pose.position.y, 0,0,0)
	start.calcfCost
	
	goal = Node(gl.position.x, gl.position.y,0,0,0)

	path = astar(start,goal)

	printPath(path)

#################################################

def printPath(path):
	for p in path:
		print "(",p.x,",",p.y,")->"


if __name__ == '__main__':
	global goal
	global start
	global spose
	global GridCells
	global mapdata
	spose = PoseStamped()
	GridCells = GridCells()

	rospy.init_node("lab3")

########PUBLISHERS
	

########SUBSCRIBERS
	goal_sub = rospy.Subscriber('/goalpose', PoseStamped, gl, queue_size=1)
	init_sub = rospy.Subscriber('/startpose', PoseWithCovarianceStamped, ipose, queue_size=1)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, ogrid, queue_size=1)

	rospy.sleep(rospy.Duration(10, 0))

	#pub = rospy.Publisher('/arrow', PoseStamped, queue_size = 1)

	print "begin main"
	
	main()

	print "end main"
