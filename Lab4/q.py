import Queue as queue

class Node:
	def __init__(self, x, y, cameFrom):
		self.x = x
		self.y = y
		self.gCost = cameFrom.gCost + 1
		self.fCost = fCost
		self.cameFrom = cameFrom

	def getNeighbors(self):
	frontier.push(Node(x, y + 1, self))
	frontier.push(Node(x, y - 1, self))
	frontier.push(Node(x - 1, y, self))
	frontier.push(Node(x + 1, y, self))


class myPriorityQueue():
	
	def __init__(self):
		self.pq = queue.PriorityQueue()

	def push(self, node):
		val = getCellValue(n.x, n.y)
		if node is in visited:
			continue
		elif ((val > 20) or (val < 0)):
			continue
		else:
			self.pq.put((node.fCost, node))

	def pop(self):
		return self.pq.get()[1]

def astar(start, goal):##need to create start and end node
	global frontier
	global visited

	current = start
	frontier = myPriorityQueue()
	visited = [start]
	start.gCost = 0
	start.cameFrom = 0
	
	while frontier:
		if (current.x is goal.x) and (current.y is goal.y)
			return repath(current)

		current = frontier.pop()
		visited.append(current)
		current.gCost = current.cameFrom.gCost + 1

		



	print "Error in A*"
	return

def getNeighbors(anode):
	global frontier
	#get x and y
	x = anode.x
	y = anode.y

	#make a node for ea direction
	frontier.push(Node(x, y + 1, anode))
	frontier.push(Node(x, y - 1, anode))
	frontier.push(Node(x - 1, y, anode))
	frontier.push(Node(x + 1, y, anode))

#manhattan dist from start to node
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


def main():
	frontier = myPriorityQueue()
	

	# A = Node(1, 2, 3, 4, 5)
	# B = Node(2, 3, 4, 5, 6)
	# C = Node(3, 4, 5, 6, 7)
	# D = Node(4, 5, 6, 2, 8)


	# frontier.push(A)
	# frontier.push(B)
	# frontier.push(C)
	# frontier.push(D)

	# for i in range(4):
	# 	print frontier.pop()

if __name__ == '__main__':
	print "begin main"
	main()
	print "end main"
