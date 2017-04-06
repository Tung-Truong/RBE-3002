#!/usr/bin/env python

#astar service client written for lab 3 rbe 3002 extra credit
import sys
import rospy
from nav_msgs.srv import GetPlan

def astar_client(x, y):
    rospy.wait_for_service('astar')
    try:
        astar = rospy.ServiceProxy('astar', GetPlan)
        resp1 = astar(x, y)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]



class Node:
    def __init__(self, x, y, gCost, fCost, cameFrom):
        self.x = x
        self.y = y
        self.gCost = gCost
        self.fCost = fCost
        self.cameFrom = cameFrom

    def calcgCost(self):
        return manhattan(self)

    def calchCost(self):
        return eucl(self)

    def calcfCost(self):
        return self.gCost + eucl(self)

    def isValid(self):
        ret = bool(getCellValue(self.x, self.y) < 20)
        return ret

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        #x = Node(200,200,0,0,0)
        #y = Node(225,225,0,0,0)
    else:
        print usage()
        sys.exit(1)

    x = Node(200,200,0,0,0)
    y = Node(225,225,0,0,0)

    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, astar_client(x, y))


