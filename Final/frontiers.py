#imports
import Queue as queue
import rospy, math, tf, numpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid,  Path, Odometry
from tf.transformations import euler_from_quaternion


#returns a point object given x,y in METERS
def makePoint(x,y):
	global mapdata
	global mapoffsetx
	global mapoffsety
	
	p = Point()
	p.x = round(x,2) + mapoffsetx
	p.y = round(y,2) + mapoffsety
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

			#print "Checking cell (",x,y,") of value: ",val, " #",index," of ",(num_rows*num_cols)

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

	if (index - 1 > 0):
		val_L = d[index - 1]
	else:
		val_L = False

	if (index + 1 < len(d)):
		val_R = d[index - 1]
	else:
		val_R = False

	if (index - cols > 0):
		val_U = d[index - cols]
	else: 
		val_U = False

	if (index + cols < len(d)):
		val_D = d[index + cols]
	else: 
		val_D = False

	a = (val_D is -1)
	b = (val_R is -1)
	c = (val_L is -1)
	d = (val_D is -1)
	return (a or b or c or d)

#return gridcells object given set of points
def toGrid(points):
	global map_res

	#create starting GridCells object
	gc = GridCells()
	gc.header.frame_id = "map"
	gc.cell_width = map_res
	gc.cell_height = map_res

	#add each point to GridCells object
	gc.cells.extend(points)

	return gc

def linearizeFrontier():
	global frontier_cells
	global finalset
	global blobset

	while len(frontier_cells) > 0:  # runs until frontier_cells is empty
		first = frontier_cells.pop()  # grabs a random object and pops it from frontier_cells
		frontier_cells.add(first)
		#print "FIRST NODE: ",first
		finalset = list()  # clears finalset after every loop
		makeBlob(first)
		blobset.append(finalset) # adds blob to a set of frontier blobs
		#print "BLOB ADDED TO BLOBSET" #, blobset


	return blobset

def makeBlob(node):
	global frontier_cells
	global finalset
	global map_res

	finalset.append(node)
	removeFromFrontier(node)
	
	x = node.x # get x,y
	y = node.y

	#check each neighboring cell (4 connected check)
	newNode1 = makePoint(round(x +  map_res,2), y) # creates a new node with the point and marks it as unexplored
	if inFrontier(newNode1): # if this cell is in frontier_cells, meaning it is a frontier cell
		#print "newNode: ", newNode1
		#finalset.add(newNode1) 
		makeBlob(newNode1) # recursively checks all the neighbors of this neighbor until all neighbors have been checked

	newNode2 = makePoint(round(x - map_res,2), y)
	if inFrontier(newNode2):
		#print "newNode: ", newNode2
		#finalset.add(newNode2)
		makeBlob(newNode2)

	newNode3 = makePoint(x, round(y + map_res,2))
	if inFrontier(newNode3):
		#print "newNode: ", newNode3
		#finalset.add(newNode3)
		makeBlob(newNode3)

	newNode4 = makePoint(x, round(y - map_res,2))
	if inFrontier(newNode4):	
		#print "newNode: ", newNode4
		#finalset.add(newNode4)
		makeBlob(newNode4)

	#print "FINAL SET FOUND!"
	
#removes given point from frontier cells
def removeFromFrontier(point):
	global frontier_cells

	for p in list(frontier_cells):
		#if p is this point remove it
		if (abs(p.x - point.x) < 0.02) and (abs(p.y - point.y) < 0.02):
			frontier_cells.remove(p)
			break

#returns true if given point is in frontier 
def inFrontier(point):
	global frontier_cells

	for p in list(frontier_cells):
		if (abs(p.x - point.x) < 0.02) and (abs(p.y - point.y) < 0.02):
			return True
	return False

#prints out lines made to debug via CLI
def printLines(lines):
	global line_ends
	count_lines = 0
	#print "GENERATED ",len(lines)," FRONTIER LINES"
	for l in lines:
		if len(l) > 1:
			print "LINE #",count_lines," W/ LEN ",len(l)
			count_lines += 1

#takes in x and y in col and row NUMBER (NOT meters!!)
def pointToIndex(x,y):
    global mapdata
    
    #get map info
    cols = mapdata.info.width
    index = ((y) * cols) + (x) #zero indexed
    return int(index)

#returns larget list in a list of lists
def largestLine(lines):
	largest_length = 0
	largest_line = set()
	for i in range(0,len(lines)):
		l = list(lines[i])
		curr_len = len(l)
		if curr_len > largest_length:
			largest_length = curr_len
			largest_line = l
	#print "LARGEST LENGTH = ",largest_length
	#print "LARGEST BLOB = ", largest_line
	return largest_line

#map sub callback
def setMap(msg):
	global mapdata
	global got_map
	global mapoffsetx
	global mapoffsety
	#global map_res
	if msg:
		print "GOT MAP!!!"
		mapdata = msg
		got_map = True
		map_res = round(msg.info.resolution,2) #<-- already done in parseMap
		mapoffsetx = msg.info.origin.position.x
		mapoffsety = msg.info.origin.position.y
		print "RESOLUTION = ",map_res," METERS"

#finds xy of center of line
def center(points):
	x_sum = 0
	y_sum = 0

	#cumulative sum of x and y
	for p in points:
		x_sum += p.x
		y_sum += p.y

	#divide to get average
	n = len(points)
	x_avg = x_sum / n
	y_avg = y_sum / n

	x_avg = round(x_avg / 0.05) * 0.05
	y_avg = round(y_avg / 0.05) * 0.05

	return makePoint(x_avg,y_avg)

#sequence of operations for evaluating map and frontier lines
def frontierOps():
	global dest_line
	global dest
	global frontier_cells

	parseMap()

	print "PUBLISHING FRONTIER CELLS..."
	pub_frontier.publish(toGrid(frontier_cells))	

	print "LINEARIZING FRONTIER..."

	start_len = len(frontier_cells)
	lines = linearizeFrontier()
	end_len = len(lines)

	#print "RESULTS: ",start_len," POINTS GROUPED INTO ",end_len," LINES"
	printLines(lines)

	rospy.sleep(rospy.Duration(0.5, 0))

	dest_line = largestLine(lines)
	pub_largest.publish(toGrid(dest_line))

	dest = center(dest_line)
	pub_ends.publish(toGrid([dest]))

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
        #print "pose: " + str(math.degrees(pose.pose.orientation.z))
        #update error
        error = angle - pose.pose.orientation.z
        
    publishTwist(0,0)

# Add additional imports for each of the message types used
def publishTwist(linearVelocity, angularVelocity):
    #send a twist message
    global pub_move
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub_move.publish(msg)
    #print "PUBLISH!"

#updates global variables for odometry
def timerCallback(event):
    global pose
    pose = PoseStamped()
    pose.header.frame_id = "map"


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

    #pub_pose.publish(pose)



#############################---MAIN---##################################


def main():
	global pose
	global map_res
	global mapdata
	global frontier_cells
	global got_map
	global dest_line
	global dest
	global mapoffsetx
	global mapoffsety



	got_map = False

	#for initial testing only
	#map_res = 0.05


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
	
	########_BEGIN MAIN LOOP_#########
	while True:
		#spin 360 degrees
		#rotate(math.pi)
		#rotate(math.pi)

		#evaluate maps and frontiers
		frontierOps()

		#chech if any big frontiers remain
		# if len(dest_line) < 2:
		# 	break
		
		#publish navigation goal
		goal = PoseStamped()
		goal.header.frame_id = "map"
		#goal.pose.position = dest
		goal.pose.position.x = dest.x - mapoffsetx
		goal.pose.position.y = dest.y - mapoffsety
		#goal.pose.orientation = pose.pose.orientation
		#goal.pose.orientation.z = 0
		#goal.pose.orientation.y = 0
		#goal.pose.orientation.x = 0
		goal.pose.orientation.w = 1.0
		print "Curr pose = ",pose
		print "dest = ",dest
		print "goal = ",goal
		pub_goal.publish(goal)

		#wait until is at destination
		break
	print"dest = ", dest

	

########################---END MAIN---################################33

if __name__ == '__main__':
	global mapdata #occupancyGrid


	#these will be sets of Point() objects
	global frontier_cells
	global free_cells
	global occupied_cells
	global unknown_cells
	global line_ends
	global blobset
	global finalset

	
	global map_res #in meters per cell
	global mapoffsetx
	global mapoffsety

	#intitialize some globals
	mapdata = OccupancyGrid()
	frontier_cells = set()
	free_cells = set()
	occupied_cells = set()
	unknown_cells = set()
	line_ends = set()
	finalset = list()
	blobset = list()

	#init node
	rospy.init_node('frontier_ops_node')

	#subscribers
	sub_map = rospy.Subscriber('/map', OccupancyGrid, setMap, queue_size=1)

	#publishers
	pub_frontier = rospy.Publisher('/frontier_cells/all', GridCells, queue_size = 10)
	pub_ends = rospy.Publisher('/frontier_cells/ends', GridCells, queue_size = 10)
	pub_largest = rospy.Publisher('/frontier_cells/largest', GridCells, queue_size = 10)
	pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
	pub_move = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10)

	# odometry stuff
	odom_list = tf.TransformListener()
	rospy.Timer(rospy.Duration(0.01), timerCallback)
	rospy.sleep(rospy.Duration(1, 0))

	print "BEGIN MAIN..."

	main()

	print "DONE!"


