import rospy, math
from geometry_msgs.msg import PoseStamped

class Node:

	def __init__(self, x, y, gCost, fCost, cameFrom):
		self.x = x
		self.y = y
		self.gCost = gCost
		self.fCost = fCost
		self.cameFrom = cameFrom


def astar(start, goal):													# returns zero if no path from start to goal otherwise returns path from start to goal
	
	global current

	current = start														
	
	closedSet = []     													# Already Evaluated Nodes initialized to zero
	openSet = [start]													# Unevaluated but discovered nodes 
	cameFrom = []														# Will contain most efficient previous steps
	gCost = []															# For each node, the cost of getting from the start node to that node
	gCost[start] = 0       												# Have not travelled when at start, thererfore inits as zero
	fCost[start] = ###### IDK !!! astar(current, goal) 								# The cost of going start to goal by passing that node

	while(openSet is not None):

		current = min()													# goes to the node in openset having the lowest fCost

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

			else if tentativegCost >= gCost[neighbor]: 					# this is not a better path
				continue

			cameFrom[neighbor] = current
			gCost[neighbor] = tentativegCost
			fcost[neighbor] = gCost[neighbor] + eucl(neighbor, goal)

		return "Error in ASTAR"

def manhattan(node, goal):												# returns the manhattan distance
	x = abs(goal.x - node.x)
	y = abs(goal.y - node.y)
	return x + y

def repath(node):
	global path
	path = p[]
	last = node.CameFrom
	while(last is not 0):
		p.add(last)
	return path

def eucl(node, neighbor):												#distance between 2 nodes
	ix = node.x
	iy = node.y
	nx = neighbor.x
	ny = neighbor.y
	dist = math.sqrt((nx-ix)**2 + (ny - iy)**2)
	return dist


def mkGraph(rows, columns, inits): 		# makes a graph with specified number of rows and columns and initializes it to (inits)
	# Define blank list
	graph = []

	#Generate rows with length of 10
	for row in range(rows):
	  # Appen a blank list to each row cell
	  graph.append([])
	  for column in range(columns):
	    # Assign 0.5 to each row initially
	    graph[row].append(inits)
	return graph



def main():
	while not rospy.isShutdown():
		rospy.sleep()


if __name__ == '__main__':
	global spose
	global GridCells
	spose = PoseStamped()
	GridCells = GridCells()


	rospy.init_node("lab3")

########PUBLISHERS
	

########SUBSCRIBERS
	goal_sub = rospy.Subscribe('/navGoal', PoseStamped, queue_size=1)
	init_sub = rospy.Subscribe('/initialPose', PoseStamped, queue_size=1)
	map_sub = rospy.Subscribe('/map', GridCells, queue_size=1)

	rospy.sleep(rospy.Duration(1, 0))


	print "begin main"
	
	main()

	print "end main"