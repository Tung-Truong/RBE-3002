#group frontier cells into lines with their neighbors
def linearizeFrontier():
	global frontier_cells
	global finalset
	global blobset

	while frontier_cells.size() > 0:  # runs until frontier_cells is empty
		first = frontier_cells.pop()  # grabs a random object and pops it from frontier_cells
		finalset.clear()  # clears finalset after every loop
		blob = makeBlob(first) # starts the recursion of grouping frontier cells into blobs
		blobset.add(blob) # adds blob to a set of frontier blobs
		print "blobset: ", blobset

def makeBlob(node):
	global frontier_cells
	global finalset

	finalset.add(node)
	frontier_cells.remove(node)
	
	x = node.x # get x,y
	y = node.y

	#check each neighboring cell (4 connected check)
	if makePoint(x + 1, y) in frontier_cells: # if this cell is in frontier_cells, meaning it is a frontier cell
		newNode = makePoint(x+1,y) # creates a new node with the point and marks it as unexplored
		print "newNode: ", newNode
		finalset.add(newNode) 
		makeBlob(newNode) # recursively checks all the neighbors of this neighbor until all neighbors have been checked

	if makePoint(x - 1, y) in frontier_cells:
		newNode = makePoint(x - 1, y)
		print "newNode: ", newNode
		finalset.add(newNode)
		makeBlob(newNode)

	if makePoint(x, y + 1) in frontier_cells:
		newNode = makePoint(x, y + 1)
		print "newNode: ", newNode
		finalset.add(newNode)
		makeBlob(newNode)

	if makePoint(x, y - 1) in frontier_cells:
		newNode = makePoint(x, y-1)
		print "newNode: ", newNode
		finalset.add(newNode)
		makeBlob(newNode)

	print "final set: ", finalset
	
	return finalset # returns the final blob


	if __name__ == '__main__':
	global mapdata

	#these will be sets of Point() objects
	global frontier_cells
	global free_cells
	global occupied_cells
	global unknown_cells
	global frontier_cells
	global blobset
	global finalset



	#intitialize some globals
	frontier_cells = set()
	free_cells = set()
	occupied_cells = set()
	unknown_cells = set()
	frontier_cells = set()
	finalset = set()
	blobset = set()


	#init node
	rospy.init_node('frontier_operations_node')

	#subscribers


	#publishers

	print "BEGIN MAIN..."

	main()

	print "DONE!"