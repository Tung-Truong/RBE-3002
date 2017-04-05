import rospy
from std_msgs.msg import String
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point


def colorCells(cells):
    global pub
    msg = GridCells()
    msg.cells = cells
    pub.publish(msg)



# This is the program's main function
if __name__ == '__main__':
    global pub

    # Change this node name to include your username
    rospy.init_node('color_cells_node')
    
    #pub to gridcells topic in rviz display
    pub = rospy.Publisher('/move_base/local_costmap/obtacles', GridCells, queue_size = 10)
    
    #sub to map, get cells
    
    #put cells into colorCells (on timer?)
    
    rospy.sleep(rospy.Duration(20, 0))
    print "DONE"
    
    
    
    
