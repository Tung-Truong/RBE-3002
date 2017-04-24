#imports
import Queue as queue
import rospy, math, tf, numpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid,  Path, Odometry
from tf.transformations import euler_from_quaternion


#returns a point object given x,y in METERS
def makePoint(x,y):
	p = Point()
	p.x = x #round(x,2)
	p.y = y #round(y,2)
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
	global map_res

	#get general map info
	num_rows = mapdata.info.height
	num_cols = mapdata.info.width
	map_res = round(mapdata.info.resolution,2)

	#actual cell data
	data = mapdata.data

	#sort all map cells by prob value
	for y in range(0, num_rows):
		for x in range(0, num_cols):

			#get value of cell
			index = pointToIndex(x,y)
			val = data[index]

			print "Checking cell (",x,y,") of value: ",val, " #",index," of ",(num_rows*num_cols)

			#convert x and y to METERS
			xm = x * map_res
			ym = y * map_res

			if val is -1: # or 50?
				unknown_cells.add(makePoint(xm,ym))
				continue
			elif val < 40:
				pnt = makePoint(xm,ym)
				free_cells.add(pnt)
				#also check if frontier
				if isFrontier(index):
					frontier_cells.add(pnt)
				continue
			elif val > 60:
				occupied_cells.add(makePoint(xm,ym))

	print "MAP PARSED!!!"

#returns true if cell is immediately bordered by open cell on map given index (of mapdata.data)
def isFrontier(index):
	global mapdata

	#get map info
	cols = mapdata.info.width
	d = mapdata.data


	val_L = d[index - 1]
	val_R = d[index + 1]
	val_U = d[index - cols]
	val_D = d[index + cols]
	a = (val_D is -1)
	b = (val_R is -1)
	c = (val_L is -1)
	d = (val_D is -1)
	return (a or b or c or d)

#return gridcells object given set of points
def setToGrid(points):
	global map_res

	#create starting GridCells object
	gc = GridCells()
	gc.header.frame_id = "map"
	gc.cell_width = map_res
	gc.cell_height = map_res

	#add each point to GridCells object
	for p in points:
		gc.cells.append(p)

	return gc

#group frontier cells into lines, returns list of lists
def linearizeFrontier():
	global frontier_cells

	lines = list() #list of lists
	
	#S...

	return lines


#function to refactor for loops, to return when done checking lines to break outer for loop
def scanLines(lines,p):
	#check against existing lines
	for i in range(0,len(lines)):
		line = lines[i]
		for q in line:

			if areNeighbors(p,q):
				#add p to line in lines
				lines[i].append(p) 
				#print "JOINED TOGETHER LINE!"
				return lines #will prompt outer for loop to continue

	#if point is not near any existing line, make it a new line
	new_line = list()
	new_line.append(p)
	lines.append(new_line)
	return lines

#prints out lines made to debug via CLI
def printLines(lines):
	global line_ends
	count_lines = 0
	print "GENERATED ",len(lines)," FRONTIER LINES"
	for l in lines:
		p1 = l[0]
		p2 = l[-1]
		line_ends.add(p1)
		line_ends.add(p2)
		print "LINE #",count_lines," FROM (",p1.x,",",p1.y,") TO (",p2.x,",",p2.y,") W/ LEN ",len(l)
		count_lines += 1

#returns true if 1 point is neighbor of second point (8-connected)
def areNeighbors(p1,p2):
	global map_res
	dx = abs(p1.x - p2.x)
	dy = abs(p1.y - p2.y)
	#print "CHECKING <",dx,",",dy,"> RESULT = ", str((dx <= map_res) or (dy <= map_res))
	return ((dx <= (2*map_res)) and (dy <= (2*map_res)))


#takes in x and y in col and row NUMBER (NOT meters!!)
def pointToIndex(x,y):
    global mapdata
    
    #get map info
    cols = mapdata.info.width
    index = ((y) * cols) + (x) #zero indexed
    return int(index)

#returns larget list in a list of lists
def largestLine(lol):
	largest_length = 0
	largest_list = list()
	for i in range(0,len(lol)):
		l = lol[i]
		curr_len = len(l)
		if curr_len > largest_length:
			largest_length = curr_len
			largest_list = l
	return largest_list




#map sub callback
def setMap(msg):
	global mapdata
	global got_map
	#global map_res
	if msg:
		print "GOT MAP!!!"
		mapdata = msg
		got_map = True
		map_res = round(msg.info.resolution,2) #<-- already done in parseMap
		print "RESOLUTION = ",map_res," METERS"

#############################---MAIN---##################################

def main():
	global map_res
	global mapdata
	global frontier_cells
	global got_map

	got_map = False

	#for initial testing only
	map_res = 0.05


	######################
	#    NOTE:           #
	# POINTS X Y Z       #
	#  ARE ALL IN        #
	#   METERS!!!        #
	######################

	print "WAITING FOR MAP..."

	while True:
		if got_map:
			print "DONE WAITING FOR MAP!"
			break


	print "PARSING MAP..."
	
	parseMap()	

		

	print "LINEARIZING FRONTIER..."

	start_len = len(frontier_cells)
	lines = linearizeFrontier()
	end_len = len(lines)

	print "RESULTS: ",start_len," POINTS GROUPED INTO ",end_len," LINES"
	printLines(lines)
	pub_ends.publish(setToGrid(line_ends))
	print "PUBLISHED END CELLS!"	

	print "PUBLISHING FRONTIER CELLS..."
	pub_frontier.publish(setToGrid(frontier_cells))

	rospy.sleep(rospy.Duration(0.5, 0))
	pub_largest.publish(setToGrid(largestLine(lines)))

########################---END MAIN---################################33

if __name__ == '__main__':
	global mapdata #occupancyGrid


	#these will be sets of Point() objects
	global frontier_cells
	global free_cells
	global occupied_cells
	global unknown_cells
	global line_ends

	global map_res #in meters per cell

	#intitialize some globals
	mapdata = OccupancyGrid()
	frontier_cells = set()
	free_cells = set()
	occupied_cells = set()
	unknown_cells = set()
	line_ends = set()

	#init node
	rospy.init_node('frontier_ops_node')

	#subscribers
	sub_map = rospy.Subscriber('/map', OccupancyGrid, setMap, queue_size=1)

	#publishers
	pub_frontier = rospy.Publisher('/frontier_cells/all', GridCells, queue_size = 10)
	pub_ends = rospy.Publisher('/frontier_cells/ends', GridCells, queue_size = 10)
	pub_largest = rospy.Publisher('/frontier_cells/largest', GridCells, queue_size = 10)

	print "BEGIN MAIN..."

	main()

	print "DONE!"


