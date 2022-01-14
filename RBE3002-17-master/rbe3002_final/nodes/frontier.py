#!/usr/bin/env python

import math
import time
import rospy
from path_planner import PathPlanner
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import subprocess
import random

class Frontier: 

    def __init__(self):
        rospy.init_node('frontier', anonymous=True)
        self.first = True
        self.last_run=0
        self.last_run_centroid = 0
        self.px = 0
        self.py = 0
        self.pth = 0
        self.zCount = 0
        self.odom = Odometry()
        self.cspace = OccupancyGrid()
        self.mapdata = OccupancyGrid()
        ### Needs to subscribe to C-Space to create frontiers
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.create_frontiers)
        self.cspace_sub = rospy.Subscriber('/path_planner/cspace/map', OccupancyGrid, self.update_cspace)
        ### Publishes the unexplored information in hierarchical need
        self.list_pub = rospy.Publisher('Frontiers', GridCells, queue_size=1)
        self.big_square = rospy.Publisher('BigSquare', GridCells, queue_size=1)
        self.return_pub = rospy.Publisher('Return', PoseStamped, queue_size=1)
        self.cent_pub = rospy.Publisher('Centroids', GridCells, queue_size=1)
        self.next_loc = rospy.Publisher('Centroids/Best', PoseStamped, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.update_odometry,  queue_size=100)
        print("Frontier init")
        rospy.sleep(1.0)

    def update_cspace(self,msg):
        self.cspace = msg

    def update_odometry(self, msg):
        #print("update_odometry")
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.odom = msg
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw
        if self.first:
            self.startX = self.px
            self.startY = self.py
            self.first = False
        
    def create_frontiers(self, msg):
        #msg is the configuration space of the map
        #Function looks for spaces that are unknown that are next to walkable spaces
        #returns a list of centroids of frontiers as a list of Points
        #frontiers = Point[]
        print("msg?", msg is None)
        if time.time() - self.last_run > 5 and not msg is None:
            curr_map = msg
            self.mapdata = msg
            frontiers = GridCells()
            centCells = GridCells()

            size = msg.info.resolution
            #size = 50
            frontiers.cell_width = size
            frontiers.cell_height = size
            centCells.cell_width = size
            centCells.cell_height = size

            frontiers.header.frame_id = 'map'
            centCells.header.frame_id = 'map'

            points = set()
            cpoints = set()

            unexplored_N = []
            counter = 0
            for i in curr_map.data:
                if i < 10 and i != -1:
                    #print("i", i)
                    #If the map believes the spot is most likely traversible it needs to check if its neighbors have been explored
                    #The list of unexplored neighbors is used to create the frontiers
                    #I think this needs an intermediate step so that centroid_generator has access to the whole list of unexplored neighbors
                    (x,y) = PathPlanner.index_to_grid(curr_map.info, counter) 
                    for item in self.unexplored_neighbors(curr_map,x,y):
                        unexplored_N.append(item)
                counter += 1
            #print("Unexplored", unexplored_N)
            centroids = (self.centroid_generator(unexplored_N))
            print("centroids generated")
            for centroid in centroids:
                p = Point() 
                p.z = 0.0
                (p.x, p.y) = PathPlanner.grid_to_world(msg, float(centroid[0]),float(centroid[1]))
                cpoints.add(p)
            for unex in unexplored_N:
                p = Point() 
                p.z = 0.0
                (p.x, p.y) = PathPlanner.grid_to_world(msg, float(unex[0]),float(unex[1]))
                points.add(p)
            points_list = list(points)
            cpoints_list = list(cpoints)

            #print("points", points_list)
            frontiers.cells = points_list
            centCells.cells = cpoints_list

            self.list_pub.publish(frontiers)
            self.cent_pub.publish(centCells)
            self.last_run = time.time()
            return frontiers

    def closest_walkable(self,x,y,allowable_dist):
        closest_dist = None
        closest_point = None
        bigsquare = GridCells()
        size = self.mapdata.info.resolution
        #size = 50
        bigsquare.cell_width = size
        bigsquare.cell_height = size
        bigsquare.header.frame_id = 'map'
        cells = []
        cellpoints = set()

        for i in range(allowable_dist*2+1):
            for j in range(allowable_dist*2+1):
                currX = (x-allowable_dist)+i
                currY = (y-allowable_dist)+j
                #print("x",((x-allowable_dist)+i),"y",((y-allowable_dist)+j))
                cells.append((currX,currY))
                if PathPlanner.is_cell_walkable(self.cspace,currX,currY):
                    walkable_point = (currX,currY)
                    print("walkable", walkable_point)
                    if PathPlanner.euclidean_distance(x,y,walkable_point[0],walkable_point[1]) < closest_dist or closest_dist == None:
                        closest_dist = PathPlanner.euclidean_distance(x,y,walkable_point[0],walkable_point[1])
                        closest_point = walkable_point
        for cell in cells:
            p = Point() 
            p.z = 0.0
            (p.x, p.y) = PathPlanner.grid_to_world(self.mapdata, cell[0],cell[1])
            cellpoints.add(p)
        bigsquare.cells = list(cellpoints)
        #self.big_square.publish(bigsquare)
        if closest_point == None:
            return False
        print("Closest", closest_point)
        return closest_point
        
    def unexplored_neighbors(self, mapdata, x, y):
        """
        Returns the unexplored 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of unexplored 4-neighbors.
        """
        neighbors = [(x,y-1),(x+1,y),(x,y+1),(x-1,y)]
        unexplored_neighbors = []
        for neighbor in neighbors:
            if not self.is_cell_explored(mapdata, neighbor[0],neighbor[1]):
                unexplored_neighbors.append(neighbor)
        return unexplored_neighbors

    def is_cell_explored(self, mapdata, x, y):
        """
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is explored, False otherwise
        """
        mapMD = mapdata.info
        wBound = mapMD.width
        hBound = mapMD.height
        if x>wBound or y>hBound:
            return True
        i = PathPlanner.grid_to_index(mapdata,x,y)
        if i >= len(mapdata.data):
            return True
        if  mapdata.data[i] == -1:
            return False
        return True

    def clumped_cells(self, cell, cells, explored, depth):
        #Recursive function that creates a set of cells that are adjacen to each other or a "clump"
        x = cell[0]
        y = cell[1]
        #print("cell",x,y)
        visited = list(explored)
        clump = set()
        clump.add(tuple(cell))
        visited.append(cell)
        neighbors = [(x,y-1),(x+1,y-1),(x+1,y),(x+1,y+1),(x,y+1),(x-1,y+1),(x-1,y),(x-1,y-1)]
        depth += 1
        #print("cells", len(cells))
        if depth < 10:
            for neighbor in neighbors:
                #print("explored", len(visited))
                if (neighbor not in visited) and (neighbor in cells):
                    clump.update(set(self.clumped_cells(neighbor,cells,visited,depth)))
        #print("Clump", len(clump), depth)
        return(clump)

    def average_position(self, cells):
        #Calculates the average position when given a list of cells in order to determine the centroid of a clump
        x=0
        y=0
        for cell in cells:
            x+=cell[0]
            y+=cell[1]
        position = (int(x/len(cells)), int(y/len(cells)))
        return position

    def centroid_generator(self, unexplored_cells):
        #Takes the current list of unexplored cells and returns a list of points that are the unexplored centroids
        clump_list = []
        centroid_list = set()
        explored = []
        maxY = None
        minY = None
        maxX = None
        minX = None
        bestScore = -1000
        currScore = 0
        bestCentroid = PoseStamped()
        #print("Unexplored Cells", unexplored_cells)
        #print("total unexplored", len(unexplored_cells))
        loop_count = 0
        for cell in unexplored_cells:
            #print("loop count", loop_count)
            clump_list.append(self.clumped_cells(cell,unexplored_cells,explored,0))
            #print("loop count +1", loop_count)
            loop_count += 1
        for clump in clump_list:
            clumpSize = len(clump)
            print("clumpsize", clumpSize)
            if clumpSize > 1: 
                clumpPos = self.average_position(clump)
                #print(clumpPos)
                #centroid_list.add(clumpPos)
                closest_to_clump = self.closest_walkable(clumpPos[0], clumpPos[1], 2)
                print("closest", closest_to_clump)
                if closest_to_clump:
                    centroid_list.add(closest_to_clump)
                    clumpDistance = PathPlanner.euclidean_distance(closest_to_clump[0],closest_to_clump[1],self.px,self.py)
                    #currScore = (clumpSize) - ((clumpDistance**2)/1550) + random.randint(0,20)
                    currScore = 1/clumpDistance
                    #currScore = clumpSize - clumpDistance/100
                    #print(clumpSize,(clumpDistance**2)/1800)
                    #print("Score", currScore, bestScore)
                    if currScore > bestScore:
                        bestScore = currScore
                        goalPosition = closest_to_clump
                        goalPosition = PathPlanner.grid_to_world(self.mapdata, goalPosition[0], goalPosition[1])
                        bestCentroid.pose.position.x = goalPosition[0]
                        bestCentroid.pose.position.y = goalPosition[1]
                        bestCentroid.pose.position.z = 0
                        goalTh = 0
                        quat2 = quaternion_from_euler(0,0,goalTh)
                        bestCentroid.pose.orientation.x = quat2[0]
                        bestCentroid.pose.orientation.y = quat2[1]
                        bestCentroid.pose.orientation.z = quat2[2]
                        bestCentroid.pose.orientation.w = quat2[3]  
                ## TODO if the list is empty of reachable goals then make the best centroid the starting position 
                ## After it gets back to the start it should save the map
                ##not 100% on the exact implementation but something like this
        #print("best centroid", bestCentroid.pose.position) 
        print("Length", len(centroid_list))
        #print("Home", self.startX, self.startY)
        if time.time() - self.last_run_centroid > 5:
            self.next_loc.publish(bestCentroid)
            self.last_run_centroid = time.time()
        if len(centroid_list)==0:
            self.zCount += 1
            print("zeros", self.zCount)
            if self.zCount > 1:
                return_point = PoseStamped()
                return_point.pose.position.x = self.startX
                return_point.pose.position.y = self.startY
                return_point.pose.position.z = 0
                goalTh = 0
                quat2 = quaternion_from_euler(0,0,goalTh)
                return_point.pose.orientation.x = quat2[0]
                return_point.pose.orientation.y = quat2[1]
                return_point.pose.orientation.z = quat2[2]
                return_point.pose.orientation.w = quat2[3] 
                print("Goin home")
                self.return_pub.publish(return_point)
            #print("save")
            #subprocess.run("rosrun", "map_server", "map_saver", "-f explored_map")   
        return centroid_list
        
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """        
        rospy.spin()


        
if __name__ == '__main__':
    Frontier().run()