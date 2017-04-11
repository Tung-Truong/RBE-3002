import rospy
from std_msgs.msg import String
from nav_msgs.msg import GridCells, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point


def colorCells(cells):
    global pub
    #msg = GridCells()
    #msg.cells = cells
    pub.publish(cells)

def getCells(msg):
    global colored_cells #cell map for obstacles
    global buffer_cells #cell map for increasing the size of obstacles
    
    colored_cells = GridCells()
    buffer_cells = GridCells()
    
    #populate header
    #print msg.header.frame_id
    colored_cells.header.frame_id = msg.header.frame_id
    cell_size = msg.info.resolution #resolution in m/cell
    #set cell width and height to resolution
    colored_cells.cell_width = cell_size
    print "CELL WIDTH: " + str(colored_cells.cell_width)
    colored_cells.cell_height = cell_size
    print str(msg.info.origin)
    
    #populate header
    #print msg.header.frame_id
    buffer_cells.header.frame_id = msg.header.frame_id
    cell_size = msg.info.resolution #resolution in m/cell
    #set cell width and height to resolution
    buffer_cells.cell_width = cell_size
    print "CELL WIDTH: " + str(colored_cells.cell_width)
    buffer_cells.cell_height = cell_size
    print str(msg.info.origin)
    
    if msg:
        data = msg.data
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
            if data[x] is (-1):
                for all points that are at most 40 cells away 
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
                #expands the obstacle
                makeBuffer(x)
            #print data[x]

#function that takes in a point, and makes a 4x4 grid around it to 
def makeBuffer(point)
    for i in range (-4, 4)
        for j in range(-4, 4)
            buff = Point()
            buff.x = float((x+i) % numCols) * cell_size 
            buff.y = float((x+j) //numCols) * cell_size
            buff.z = 0
            buffer_cells.cells.append(buff)
            
# This is the program's main function
if __name__ == '__main__':
    global pub
    global colored_cells
    

    # Change this node name to include your username
    rospy.init_node('color_cells_node')
    
    #pub to gridcells topic in rviz display
    pub = rospy.Publisher('/move_base/local_costmap/obstacles', GridCells, queue_size = 10)
    
    #pub to gridcells topic 
    pub = rospy.Publisher('/move_base/local_costmap/buffer', BufferCells , queue_size = 10)
    
    #sub to map, get cells
    sub = rospy.Subscriber('/map', OccupancyGrid, getCells, queue_size=1)
    
    #put cells into colorCells (on timer?)
    rospy.sleep(rospy.Duration(2, 0))
    colorCells(colored_cells)
    
    rospy.sleep(rospy.Duration(30, 0))
    print "DONE"
    
    
    
    
