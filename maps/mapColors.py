import rospy
from std_msgs.msg import String
from nav_msgs.msg import GridCells, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point


def pubCells(cells):
    global pub
    pub.publish(cells)

def pubExpCells(cells):
	global pub_exp_cells
	pub_exp_cells.publish(cells)

def getCells(msg):
    global colored_cells #cell map for obstacles
    global mapdata
    
    if msg: 
		colored_cells = GridCells()
		

		#populate header
		#print msg.header.frame_id
		colored_cells.header.frame_id = "map" #msg.header.frame_id
		cell_size = msg.info.resolution #resolution in m/cell
		#set cell width and height to resolution
		colored_cells.cell_width = cell_size
		print "CELL WIDTH: " + str(colored_cells.cell_width)
		colored_cells.cell_height = cell_size
		print str(msg.info.origin)

	

		data = msg.data
		mapdata = msg
		print len(data)
		#print data
		#get size of map
		numCols = msg.info.width
		print numCols
		numRows = msg.info.height
		print numRows
		numCells = numCols * numRows
		print numCells

		#iterate through 2d matrix
		for x in range(0,numCells):
		    #if unexplored cell
		    #print x
		    if data[x] > (60):
		        #create point
		        point = Point()
		        #print "found a cell! " + str(x)
		        point.x = float(x % numCols) * cell_size #column number * meters/cell
		        #print str(point.x)
		        point.y = float(x // numCols) * cell_size #row number * meters/cell
		        #print str(point.y)
		        point.z = 0 #or 1 needed???
		        #add point to colored_cells
		        colored_cells.cells.append(point)

#function that takes in a point, and makes a 4x4 grid around it to 
def expandPoint(x,y):
    global expanded_cells
    for i in range(-4, 4):
        for j in range(-4, 4):
			pnt = Point()
			pnt.x = (x + i)  
			pnt.y = (y + j)
			pnt.z = 0
			#if (pnt not in expanded_cells.cells):
			expanded_cells.cells.append(pnt)

#expands given GridCells.cells and updates expanded_cells
def expansion(cells):
	global expanded_cells

	starting_cells = cells #save local as to not affect for loop
	n = len(starting_cells)
	print "# cells: ", str(n)
	count = 0

	expanded_cells = GridCells()
	expanded_cells.header.frame_id = "map"
	expanded_cells.cell_width = 1
	expanded_cells.cell_height = 1

	for c in starting_cells:
		#print "Expanding: ", "(",c.x,",",c.y,")"
		print "Cell #: ",count, " out of ", n
		expandPoint(c.x, c.y)
		count += 1

# This is the program's main function
if __name__ == '__main__':
    global pub
    global pub_exp_cells
    global colored_cells
    global expanded_cells
    global mapdata

    # Change this node name to include your username
    rospy.init_node('color_cells_node')
    
    #pub to gridcells topic in rviz display
    pub = rospy.Publisher('/move_base/local_costmap/obstacles', GridCells, queue_size = 10)
    
    #pub to gridcells topic 
    pub_exp_cells = rospy.Publisher('/expanded_cells', GridCells , queue_size = 10)
    
    #sub to map, get cells
    sub = rospy.Subscriber('/map', OccupancyGrid, getCells, queue_size=1)
    
    #put cells into pubCells 
    rospy.sleep(rospy.Duration(5, 0))
    
    
    
    #expand all the found points
    expansion(colored_cells.cells)
    pubExpCells(expanded_cells)

    rospy.sleep(rospy.Duration(0.5, 0))

    #pub coored_cells after so its on top?
    pubCells(colored_cells)
    rospy.sleep(rospy.Duration(0.5, 0))

    print "DONE"
    
    
    
    
