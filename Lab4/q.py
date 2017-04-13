import Queue as queue

class Node:
	global goal
	global visited
	def __init__(self, x, y, gCost, fCost, cameFrom):
		self.x = x
		self.y = y
		self.gCost = gCost
		self.fCost = fCost
		self.cameFrom = cameFrom

	def	expand(self):
		lis = ()
		A = Node(x, y+1, gCost + 1, heur(goal.x-x, goal.y-y+1), self)
		B = Node(x, y-1, gCost + 1, heur(goal.x-x, goal.y-y-1), self)
		C = Node(x+1, y, gCost + 1, heur(goal.x-x+1, goal.y-y), self)
		D = Node(x-1, y, gCost + 1, heur(goal.x-x-1, goal.y-y), self)
		lis.append(A)
		lis.append(B)
		lis.append(C)
		lis.append(D)
		for i in range(4):
			if(getCellValue(i.x, i.y) not == -1) and not in visited:
				push(i)
	
class myPriorityQueue():
	global visited
	def __init__(self):
		self.pq = queue.PriorityQueue()

	def push(self, node):
			self.pq.put((node.fCost, node))

	def pop(self):
		visited.add(self.pq.get()[1])
		return self.pq.get()[1]

def astar(start, goal):													
	global frontier
	global path

	start.expand()

	while frontier.pop():
		#get lowest cost Node from frontier
		current = frontier.pop()														# goes to the node in openset having the lowest fCost

		if(abs(current.x - goal.x) + abs(current.y - goal.y)) <= 1:											# if current is goal ->
			return repath(current)
		else:
			current.expand()
	return "Error in ASTAR"

#returns data (probability) of map cell given (x,y) coord
def getCellValue(x,y):
    global mapdata
    #get map info
    cols = mapdata.info.width
    rows = mapdata.info.height
   
    index = (y * cols) + x #zero indexed, (y * cols) represents first number of each row, then add x (the column)
    return mapdata.data[index]

# def repath(node):
# 	global path
# 	path = [node]
# 	n = node
# 	last = n.cameFrom
# 	while(last is not 0):
# 		path = [last] + path
# 		n = n.cameFrom 
# 		last = n.cameFrom
# 	return path

def heur(x, y):
	return sqrt(x** + y**)

def main():
	global path
	global frontier
	global visited

	frontier = myPriorityQueue()


if __name__ == '__main__':
	print "begin main"
	main()
	print "end main"
