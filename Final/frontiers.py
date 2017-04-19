#imports
import Queue as queue
import rospy, math, tf, numpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid,  Path, Odometry
from tf.transformations import euler_from_quaternion




#returns true if cell is immediately bordered by unexplored cell on map
def isFrontier(index):
	global mapdata

	#get map info
	cols = mapdata.info.width
	d = mapdata.data

	val = d[index]

	#get values of neighboring cells
	val_L = d[index - 1]
	val_R = d[index + 1]
	val_U = d[index - cols]
	val_D = d[index + cols]
	a = (val_D > -1)
	b = (val_R < 40) and (val_R > -1)
	c = (val_L < 40) and (val_U > -1)
	d = (val_D < 40) and (val_D > -1)
	e = (val > 60) or (val is -1)
	return ((a or b or c or d) and e)

#map sub callback
def updateMap(msg):
	global mapdata
	if msg:
		#save map into global variable
		mapdata = msg

#returns a point object given coords
def makePoint(x,y):
	p = Point()
	p.x = x
	p.y = y
	p.z = 0
	return p


#breaks the global map down into cell types
def parseMap():
	global mapdata
	#these will be sets of Point() objects
	global frontier_cells
	global free_cells
	global occupied_cells
	global unknown_cells

	#get general map info
	num_rows = mapdata.info.height
	num_cols = mapdata.info.width
	data = mapdata.data

	#sort all map cells by prob value
	for y in range(0, num_rows):
		for x in range(0, num_cols):
			#get value of cell
			val = data[pointToIndex(x,y)]

			if val is -1: # or 50?
				unknown_cells.add(makePoint(x,y))
				continue
			elif val < 40:
				free_cells.add(makePoint(x,y))
				continue
			elif val > 60:
				occupied_cells.add(makePoint(x,y))

	#get frontier values
	for p in free_cells:
		x = p.x
		y = p.y

		#check ea neighboring cell (4 connected check)
		if makePoint(x + 1, y) in unknown_cells:
			frontier_cells.add(p) #add current point to frontier and skip to next
			continue
		if makePoint(x - 1, y) in unknown_cells:
			frontier_cells.add(p)
			continue
		if makePoint(x, y + 1) in unknown_cells:
			frontier_cells.add(p)
			continue
		if makePoint(x, y - 1) in unknown_cells:
			frontier_cells.add(p)
			continue

#make gridcellsobjects out of cell type sets and publish
def makeGrids():
	pass

#group frontier cells into lines with their neighbors
def linearizeFrontier():
	pass






#takes in x and y in col and row NUMBER
def pointToIndex(x,y):
    global mapdata
    
    #get map info
    cols = mapdata.info.width
    index = ((y) * cols) + (x) #zero indexed
    return int(index)


def main():
	pass


if __name__ == '__main__':
	global mapdata

	#these will be sets of Point() objects
	global frontier_cells
	global free_cells
	global occupied_cells
	global unknown_cells

	#intitialize some globals
	frontier_cells = set()
	free_cells = set()
	occupied_cells = set()
	unknown_cells = set()

	#init node
	rospy.init_node('frontier_operations_node')

	#subscribers


	#publishers

	print "BEGIN MAIN..."

	main()

	print "DONE!"


