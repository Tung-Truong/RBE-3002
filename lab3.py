import rospy, math, tf
from geometry_msgs.msg import PoseStamped
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
		return manhattan(self) + eucl(self)

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
	pass
	global current

	current = start														
	
	closedSet = []     													# Already Evaluated Nodes initialized to zero
	openSet = [start]													# Unevaluated but discovered nodes 
	cameFrom = []														# Will contain most efficient previous steps
	gCost = []															# For each node, the cost of getting from the start node to that node
	fCost = []															# For each node, the cost of getting from that node to goal
	camFrom[start] = 0
	gCost[start] = 0       												# Have not travelled when at start, thererfore inits as zero
	fCost[start] = eucl(start, goal)									# The cost of going start to goal by passing that node

	while openSet:

		current = start														# goes to the node in openset having the lowest fCost

		if(current is goal):											# if current is goal ->
			return repath(current) 										# then return the path back to the start

		openSet.remove(current) 										# update each of the sets
		closedSet.add(current)

		for neighbor in neighbors:
			
			if neighbor in closedSet: 									# ignores already evaluated nodes
				continue

			tentativegCost = manhattan(current, start) +  eucl(current, neighbor)	# the cost from start to the neighbor

			if neighbor not in openSet: 								# discovered a new node
				openSet.add(neighbor) 	

			elif (tentativegCost >= gCost[neighbor]): 					# this is not a better path
				continue

			cameFrom[neighbor] = current
			gCost[neighbor] = tentativegCost
			fcost[neighbor] = gCost[neighbor] + eucl(neighbor, goal)

		return "Error in ASTAR"
	##pass
def manhattan(node):												# returns the manhattan distance
	global start
	x = abs(start.x - node.x)
	y = abs(start.y - node.y)
	return x + y

def repath(node):
	global path
	path = []
	last = node.CameFrom
	while(last is not 0):
		path.add(last)
	return path

def eucl(node):												# distance between node and goal
	global goal
	ix = node.x
	iy = node.y
	nx = goal.x
	ny = goal.y
	dist = math.sqrt((nx-ix)**2 + (ny - iy)**2)
	return dist





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

def ipose(msg):															# stores start pose as global
	global ipose
	if msg:
		ipose = msg.pose

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






def main():																# main function
	start = Node(0, 0, 0, 0, 0)
	start.calcfCost
	var = Node(1, 1, 2, 0, start)
	var.calcfCost
	goal = Node(2, 2, 4, 4, var)
	print "start fcost", start.fCost
	print "gCost", var.gCost
	print "fCost", var.fCost





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
	goal_sub = rospy.Subscriber('/navGoal', PoseStamped, gl, queue_size=1)
	init_sub = rospy.Subscriber('/initialPose', PoseStamped, ipose, queue_size=1)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, ogrid, queue_size=1)

	rospy.sleep(rospy.Duration(1, 0))


	print "begin main"
	
	main()

	print "end main"
