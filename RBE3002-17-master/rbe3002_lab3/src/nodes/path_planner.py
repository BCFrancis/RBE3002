#!/usr/bin/env python

import math
import rospy
from priority_queue import PriorityQueue
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
import copy

from tf.transformations import quaternion_from_euler


class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        ## rospy.wait_for_service('plan_path')
        self.pathServ = rospy.Service('plan_path', GetPlan, self.plan_path)        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCellss
        self.cpub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.fpub = rospy.Publisher('frontier', GridCells, queue_size=10)
        self.excpub = rospy.Publisher('expanded_cells', GridCells, queue_size=10)
        ### Tell ROS that this node publishes Path messages on the 'path_publisher' topic
        self.pathpub = rospy.Publisher('path_publisher', Path, queue_size=10)

        ## Initialize the request counter
        self.rcounter = 0
        ## Create goal message type
        self.goal = PoseStamped()
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        


    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        index = (mapdata.info.width * y) + x
        return int(index)
        

    @staticmethod
    def index_to_grid(mapdata, i):
        """
        Returns the given (x,y) coordinates corresponding to the index in the occupancy grid.
        :param i  [int] The index.
        :return x [int] The cell X coordinate.
        :return y [int] The cell Y coordinate.
        """

        x = i%mapdata.width
        y = math.floor(i/mapdata.width)

        return [x,y]


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        return math.sqrt((y2-y1)**2 + (x2-x1)**2)
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        xCell = ((x+.5)*mapdata.info.resolution) + mapdata.info.origin.position.x
        yCell = ((y+.5)*mapdata.info.resolution) + mapdata.info.origin.position.y
        point = (xCell,yCell)
        return point


        
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        xCell = math.floor((wp.x-mapdata.info.origin.position.x)/mapdata.info.resolution)
        yCell = math.floor((wp.y-mapdata.info.origin.position.y)/mapdata.info.resolution)
        position = (xCell,yCell)
        return position


        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        coordlist = []
        for waypoints in path:
            coordlist.append(PathPlanner.grid_to_world(mapdata,waypoints[0],waypoints[1]))
        return coordlist
    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        mapMD = mapdata.info
        wBound = mapMD.width
        hBound = mapMD.height
        if x>wBound or y>hBound:
            return False
        i = PathPlanner.grid_to_index(mapdata,x,y)
        if i >= len(mapdata.data):
            return False
        if mapdata.data[i] >= 50 or mapdata.data[i] == -1:
            return False
        return True


               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        neighbors = [(x,y-1),(x+1,y),(x,y+1),(x-1,y)]
        walkeable_neighbors = []
        for neighbor in neighbors:
            if PathPlanner.is_cell_walkable(mapdata, neighbor[0],neighbor[1]):
                walkeable_neighbors.append(neighbor)
        return walkeable_neighbors

        

    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        neighbors = [(x,y-1),(x+1,y-1),(x+1,y),(x+1,y+1),(x,y+1),(x-1,y+1),(x-1,y),(x-1,y-1)]
        walkeable_neighbors = []
        for neighbor in neighbors:
            if PathPlanner.is_cell_walkable(mapdata, neighbor[0],neighbor[1]):
                walkeable_neighbors.append(neighbor)
        return walkeable_neighbors

    
    
    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        try:
            map_serv = rospy.ServiceProxy('/dynamic_map', GetMap)
            amap = map_serv()
            return amap
        except:
            return None



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        data = mapdata.data
        metadata = mapdata.info
        newData = OccupancyGrid()
        newData.info = metadata
        setOfObs = set()
        setOfNewObs = set()
        totalObs = list(copy.copy(data))
        points = []
        i=0
        for item in data:
            if item != 0:
                setOfObs.add(i)
            i+=1

        #runs as many times as amount of padding specified, default padding is 1
        for pad in range(padding):
            for ind in sorted(setOfObs):
                obstacleCell = PathPlanner.index_to_grid(metadata,ind)
                for neighbor in PathPlanner.neighbors_of_8(mapdata, obstacleCell[0], obstacleCell[1]):
                    setOfNewObs.add(PathPlanner.grid_to_index(mapdata, neighbor[0],neighbor[1]))
            print('before',len(setOfObs))    
            setOfObs = setOfObs.union(setOfNewObs)
            print('after',len(setOfObs))    


        for index in setOfObs:
            totalObs[index] = 100
            p = Point()
            cell = PathPlanner.index_to_grid(metadata,index)
            (p.x, p.y) = PathPlanner.grid_to_world(mapdata, cell[0],cell[1])
            points.append(p)
        newData.data = totalObs
        ## Create a GridCells message and publish it
        msg = GridCells()
        msg.header = newData.header
        msg.header.frame_id = 'map'
        msg.cell_width = newData.info.resolution
        msg.cell_height = newData.info.resolution
        #list of cells that are now obstcles in world coordinates (thats what the write-up says)
        msg.cells = points 
        self.cpub.publish(msg)
        ## Return the C-space
        return newData


    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        print('a*')
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        seen = []

        graph = mapdata.data
        while not frontier.empty():
            current = frontier.get()
            ## Create a GridCells message and publish it so RVis knows what we are about to visit
            msg = GridCells()
            msg.header = mapdata.header
            msg.header.frame_id = 'map'
            msg.cell_width = mapdata.info.resolution
            msg.cell_height = mapdata.info.resolution
            #list of Points in the world coordinates that are on the edge of the wavefront (frontier)
            msg.cells = PathPlanner.gridCords_to_Points(mapdata, copy.copy(frontier)) 

            self.fpub.publish(msg)

	        #all of the nodes that we have seen so far
            seen.append(PathPlanner.coords_to_Points(mapdata, current)) #add the current node to seen
            msgSeen = GridCells()
            msgSeen.header = mapdata.header
            msgSeen.header.frame_id = 'map'
            msgSeen.cell_width = mapdata.info.resolution
            msgSeen.cell_height = mapdata.info.resolution
            #list of Points in the world coordinates that we have seen
            msgSeen.cells = seen
            self.excpub.publish(msgSeen)

            # If goal has been reached
            if current == goal:
                break
            for next in PathPlanner.neighbors_of_8(mapdata,current[0],current[1]): #graph.neighbors(current):
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current[0],current[1],next[0], next[1]) #graph.cost(current, next)
                if (next not in came_from.keys()) or (new_cost < cost_so_far[next]):
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal[0], goal[1], next[0], next[1]) #heuristic(goal, next)
                    frontier.put(next,priority)

                    came_from[next] = current
        return PathPlanner.createPath(mapdata,came_from,start,goal)


    @staticmethod
    def createPath(mapdata, came_from, start, goal):
        currNode = goal
        nextNode = came_from[currNode]
        waypoints = []
        while nextNode != start:
            nextNode = came_from[currNode]
            waypoints.append(nextNode)
            currNode = nextNode
        return waypoints

    @staticmethod
    def coords_to_Points(mapdata, coordinate):
        """
        Converts a priority queue of x,y pairs into an array of Points
        :param mapdata   [OccupancyGrid]    The map data.
        :param coordinate [[int,int]]       the coordinate
        :return          [Point]          The point
        """
        p = Point()
        (p.x, p.y) = PathPlanner.grid_to_world(mapdata, coordinate[0], coordinate[1])
        return p


    @staticmethod
    def gridCords_to_Points(mapdata, gridCords):
        """
        Converts a priority queue of x,y pairs into an array of Points
        :param mapdata   [OccupancyGrid]    The map data.
        :param gridCords [PriorityQueue]    The x,y pairs
        :return          [Point[]]          The array of points
        """
        points = []
        for i in gridCords.get_queue():
            p = Point()
            (p.x, p.y) = PathPlanner.grid_to_world(mapdata, i[1][0], i[1][1])
            points.append(p)
        return points

    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        same_direction = []
        ### EXTRA CREDIT
        ## if there are multiple waypoints in a row, remove the middle points
        ## check if point orientation is in the same direction, determined if waypoints in line. 
        ## Do for diagonals movement also? 
        ## only delete the intermediate points A and B, still need a start and stop location

        for i in range(len(path)-2):
            print("DEBUG",range(len(path)-2),i)
            if path[i].pose.orientation == path[i+1].pose.orientation:
                same_direction.append(path[i+1])

        for item in same_direction:
            path.remove(item)
        
        rospy.loginfo("Optimizing path")
        return path

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        aPath = Path()
        aPath.header.frame_id = 'map'
        aPoses = []
        ## path list in reverse order
        path = list(reversed(path))
        for waypoint in path:
            curPose = PoseStamped()
            curPose.pose.position = PathPlanner.coords_to_Points(mapdata,[waypoint[0],waypoint[1]])
            if path.index(waypoint) < (len(path)-1):
                ydiff = path[path.index(waypoint)+1][1] - waypoint[1]
                xdiff = path[path.index(waypoint)+1][0] - waypoint[0]
                orientation = quaternion_from_euler(0,0,math.atan2(ydiff,xdiff))
                curPose.pose.orientation.x = orientation[0]
                curPose.pose.orientation.y = orientation[1]
                curPose.pose.orientation.z = orientation[2]
                curPose.pose.orientation.w = orientation[3]
            else:
                orientation = self.goal.pose.orientation
                curPose.pose.orientation = orientation
            aPoses.append(curPose)
        ## Optimize waypoints
        #aPoses = PathPlanner.optimize_path(aPoses)
        aPath.poses = aPoses
        
        rospy.loginfo("Returning a Path message")
        self.pathpub.publish(aPath)
        return aPath


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the request.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        self.goal = msg.goal
        mapdata = PathPlanner.request_map()
        mapdata = mapdata.map
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        
        ## Return a Path message
        retPath = self.path_to_message(mapdata, path)
        return retPath


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        mapdata = PathPlanner.request_map()
        print(mapdata)
        self.calc_cspace(mapdata.map, 1)
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
