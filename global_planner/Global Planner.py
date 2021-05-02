##Code assumes robot is in Northern and Western hemispheres

from math import sin, cos, sqrt, atan2, radians, inf, ceil
import heapq
import time

##GPS coordinate calclations
from geographiclib.geodesic import Geodesic

##SQl stuff
import sqlite3
from sqlite3 import Error

##ROS stuff only if in ros
if __name__ != "__main__":
    import rospy

##For testing
import random


## HELPER FUNCTIONS ###################################################
def dist(coord1, coord2):
    """
    Gets distance between 2 coordinates in meters
    """
    # approximate radius of earth in km
    R = 6373.0

    # get angle of coordinates
    lat1 = radians(coord1[0])
    lon1 = radians(coord1[1])
    lat2 = radians(coord2[0])
    lon2 = radians(coord2[1])

    # get difference of angles
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    # get distance in km based onarc length
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c

    # convert to m
    return distance * 1000
def node_expanded(list, node):
    """
    Tests whether a particular graph node has been expanded previously
    Returns true if node has been expanded according to list of astar_nodes
        -node is an astar_node
        -list is a list of astar_nodes
    """
    # iterate through list
    for n in list:
        # if find the graph node we're looking for
        # (looking for the actual node itself)
        if n.node == node.node:
            return True
    return False
# maximum distance between gps waypoints in meters
MAX_DIST_BETWEEN_GPS_WAYPOINTS = 10
def fill_gap(coord1, coord2):
    """
    Splits path between coord1 and coord2 into a path of subnodes based
    on MAX_DIST_BETWEEN_GPS_WAYPOINTS. Resulting path does not include coord2.
    """
    # Get dist between coordinates
    total_dist = dist(coord1, coord2)
    # Get the number of sub waypoints there should be
    waypoint_number = ceil(total_dist / MAX_DIST_BETWEEN_GPS_WAYPOINTS) - 1
    if waypoint_number == -1:
        return [coord1]
    # Get the dist between each sub waypoint
    waypoint_dist = total_dist / (waypoint_number + 1)
    # Start of path
    sub_path = [coord1]

    # If there needs to be a sub waypoints
    if waypoint_number > 0:
        # Get a line along a geodesic from coord1 to coord2
        gd = Geodesic.WGS84.Inverse(coord1[0], coord1[1], coord2[0], coord2[1])
        line = Geodesic.WGS84.Line(gd['lat1'], gd['lon1'], gd['azi1'])

        # For number of sub waypoints
        for i in range(1, waypoint_number):
            # Get point along line
            point = line.Position(gd['s12'] / waypoint_number * i)
            # Add point to list
            sub_path.append((point['lat2'], point['lon2']))

    return sub_path

class node:
    """
    Holds coordinates and node that there is a path to
    """
    def __init__(self, lat, long):
        self.coordinate = (lat, long)
        self.connections = []

    def dist(self, other):
        """
        Gets distance between to nodes in meters
        """
        return dist(self.coordinate, other.coordinate)
class astar_node():
    """
    Hold information for performing A*
    """
    def __init__(self, node, parent, cost, goal):
        self.node = node
        self.parent = parent
        self.cost = cost
        # Heuristic
        self.priority = cost + node.dist(goal)

    def __lt__(self, other):
        """
        For heap operations
        """
        return self.priority < other.priority
class graph:
    """
    Handles graph operations
    """
    def __init__(self):
        self.nodes = []

    ##For testing
    def get_min_lat(self):
        """
        Get the smallest latitude value in the graph
        """
        lat = inf
        for n in self.nodes:
            if n and n.coordinate[0] < lat:
                lat = n.coordinate[0]
        return lat
    def get_max_lat(self):
        """
        Get the largest latitude value in the graph
        """
        lat = -inf
        for n in self.nodes:
            if n and n.coordinate[0] > lat:
                lat = n.coordinate[0]
        return lat
    def get_min_long(self):
        """
        Get the smallest longitude value in the graph
        """
        long = inf
        for n in self.nodes:
            if n and n.coordinate[1] < long:
                long = n.coordinate[1]
        return long
    def get_max_long(self):
        """
        Get the largest longitude value in the graph
        """
        long = -inf
        for n in self.nodes:
            if n and n.coordinate[1] > long:
                long = n.coordinate[1]
        return long

    def add_node(self, node):
        """
        Add node to graph
        """
        self.nodes.append(node)
    def add_connection(self, node1, node2):
        """
        Add a connection to the graph
            node1 and node2 are indices, not node objects
        """
        self.nodes[node1].connections.append(self.nodes[node2])
        self.nodes[node2].connections.append(self.nodes[node1])

    def get_closest_node(self, coord):
        """
        Gets the node in the graph that is the closest to coord
        """
        closest = None
        closest_dist = inf
        for n in self.nodes:
            if n and dist(n.coordinate, coord) < closest_dist:
                closest = self.nodes.index(n)
                closest_dist = dist(n.coordinate, coord)
        return closest
    def path_between_nodes(self, node1, node2):
        """
        A* path finding. Finds path of nodes.
            node1 and node2 are indices, not node objects
        """
        # A* priority queue
        # Starts at node1, with no parent, no cost
        queue = [astar_node(self.nodes[node1], -1, 0, self.nodes[node2])]
        # Nodes expanded
        expanded = []
        # what the goal node is
        end = None

        # While the end has not been found and there are nodes to explore
        while len(queue) > 0 and not end:
            # Pop next node from priority queue
            n = heapq.heappop(queue)
            # If it's a node that hasn't been explored.
            # By the nature of A*, the first time a node is expanded
            # is through the shortest path to that node
            if not node_expanded(expanded, n):
                # Add to expanded list
                expanded.append(n)
                # If this is the goal
                if n.node == self.nodes[node2]:
                    # Set goal and exit
                    end = n
                    continue
                # For each connection
                for neighbor in n.node.connections:
                    # Push new node into priority queue
                    # Parameters are:
                    #                                node,
                    #                                          index of parent node (currently expanded node) in expanded
                    #                                                             Cost of the new node (cost of parent + distance from parent)
                    #                                                                                             Goal node (to calculate priority value)
                    heapq.heappush(queue, astar_node(neighbor, len(expanded) - 1, n.cost + n.node.dist(neighbor), self.nodes[node2]))
        # If the end was found
        if end:
            # Reverse engineer path starting at end
            path = [self.nodes.index(end.node)]
            current = end
            # While the starting node hasn't been found
            while current.parent != -1:
                path.insert(0, self.nodes.index(expanded[current.parent].node))
                current = expanded[current.parent]
            return path
        else:
            # Else return empty list
            return []
    def coordinate_path(self, current_position, destination):
        """
        Finds path of gps waypoints between two gps coordinates.
            current_position and destination are gps coordinates
        """
        # Get the nodes closest to the coordinates of interest
        starting_node = self.get_closest_node(current_position)
        ending_node = self.get_closest_node(destination)

        # Get the nodes that form a higher level path
        path = self.path_between_nodes(starting_node, ending_node)
        print(path)

        # If a valid path was found
        if path:
            # Transform path from node indices to gps coordinates
            coord_path = [current_position]
            coord_path += [self.nodes[i].coordinate for i in path]
            coord_path.append(destination)

            ##Create a finer detailed path based on MAX_DIST_BETWEEN_GPS_WAYPOINTS
            new_coord_path = []
            for i in range(len(coord_path) - 1):
                new_coord_path += fill_gap(coord_path[i], coord_path[i + 1])
            new_coord_path.append(coord_path[-1])
            ##Return
            return new_coord_path
        else:
            # Else return empty list
            return []

##For testing
class movement_simulator():
    """
    Simulates the robot moving
    """
    def __init__(self, min_lat, max_lat, min_long, max_long):
        self.pos = (random.uniform(min_lat, max_lat), random.uniform(min_long, max_long))
        self.speed = 2

        self.goal = None

    ##Change during integration
    def get_current_gps_coords(self):
        """
        Gets the gps coordinations of the robot
        """
        return self.pos
    ##Change during integration
    def set_goal(self, coord):
        """
        Sets the goal to new coordinates
        """
        self.goal = coord
        print("###### Goal updated to: ", self.goal, "######\n\t\tRobot is", str(dist(self.goal, self.pos)) + "m from goal" )
    ##For testing
    def move(self):
        """
        Simulates movement
        """
        # If goal
        if goal:
            # Get line in desired direction
            gd = Geodesic.WGS84.Inverse(self.pos[0], self.pos[1], self.goal[0], self.goal[1])
            line = Geodesic.WGS84.Line(gd['lat1'], gd['lon1'], gd['azi1'])
            # If too close
            if dist(self.goal, self.pos) < self.speed:
                # Don't overshoot
                point = line.Position(dist(self.goal, self.pos))
            else:
                # Move in direction by speed
                point = line.Position(self.speed)
            # Set pos
            self.pos = (point['lat2'], point['lon2'])

            print("GOAL:", self.goal, " POS:", self.pos)

##For testing
def load_graph_from_file(file):
    """
    Loads graph data from file
    """
    g = graph()
    f = open(file, "r")
    text = f.read().split('\n')

    coords_in = False
    for line in text:
        if not coords_in:
            if line != '':
                c = line.split(', ')
                n = node(float(c[0]), float(c[1]))
                g.add_node(n)
            else:
                coords_in = True
        else:
            i = line.split(' ')
            g.add_connection(int(i[0]), int(i[1]))

    return g

def create_connection(db_file):
    """ create a database connection to a SQLite database """
    conn = None
    try:
        conn = sqlite3.connect(db_file)
        return conn
    except Error as e:
        print(e)
    return None
def load_graph_from_database(db_file):
    """
    Loads graph data from db file
    """
    ##Create graph
    g = graph()

    ##Connect to database (or create if it doesn't exist)
    connection = create_connection(db_file)
    cursor = connection.cursor()

    ##Create nodes
    nodes = cursor.execute("SELECT * FROM Nodes").fetchall()
    g.nodes = [None] * (len(nodes) + 1)
    for row in nodes:
        g.nodes[row[0]] = node(float(row[1]), float(row[2]))

    ##Create conections
    connections = cursor.execute("SELECT * FROM Connections").fetchall()
    for row in connections:
        g.add_connection(int(row[0]), int(row[1]))

    return g

##For testing
def random_node_in_graph(graph):
    """
    Gets a random coordinate from graph
    """
    return graph.nodes[random.randint(0, len(graph.nodes) - 1)].coordinate
def print_path(path):
    """
    Displays gps path
    """
    print("Path is", len(path), "waypoints long")
    for w in path:
        print("\t", w)

##Defines how close the robot needs to be to its destination in m for it to be considered there (tolerence)
CLOSE_ENOUGH = 0.5

def newDelivery_cb(msg):
    """
    This function will be called whenever there will be change in the new_delivery topic
    """
    cur = (msg.current_lat, msg.current_long)
    start = (msg.start_lat, msg.start_long)
    goal = (msg.goal_lat, msg.goal_long)

    # Get graph from database
    g = load_graph_from_database("test_lite.db")

    # Find path
    path = g.coordinate_path(cur, goal)

    # Output to txt file
    with open("delivery_waypoints.txt", "w+") as file:
        for p in path:
            file.write(str(p[0]) + " " + str(p[1]) + "\n")
##For testing
def test_newDelivery_cb():
    """
    Test version of newDelivery_cb
    """
    # Get graph from database
    g = load_graph_from_database("test_lite.db")

    cur = random_node_in_graph(g)
    start = random_node_in_graph(g)
    goal = random_node_in_graph(g)

    # Find path
    path = g.coordinate_path(cur, goal)

    # Output to txt file
    with open("delivery_waypoints.txt", "w+") as file:
        for p in path:
            file.write(str(p[0]) + " " + str(p[1]) + "\n")



##For testing
if __name__ == "__main__":
    # Get graph from database
    ##g = load_graph_from_file("test coords.txt")
    g = load_graph_from_database("test_lite.db")
    # Create simulator (for testing)
    robot = movement_simulator(g.get_min_lat(), g.get_max_lat(), g.get_min_long(), g.get_max_long())

    # Run until power off
    while True:
        goal = random_node_in_graph(g)
        path = g.coordinate_path(robot.get_current_gps_coords(), goal)
        print_path(path)
        current_sub_goal = 0
        robot.set_goal(path[current_sub_goal])

        while True:
            while(dist(robot.get_current_gps_coords(), path[current_sub_goal]) > CLOSE_ENOUGH):
                ##Simulate robot moving over time
                robot.move()
                ##Wait before checking again
                time.sleep(1)
            current_sub_goal += 1
            if current_sub_goal >= len(path):
                break
            robot.set_goal(path[current_sub_goal])

        print("###### ROBOT ARRIVED AT DESTINATION ######")
        break
else:   ##ROS stuff
    rospy.init_node('mpdr/global_planner')
    gpsSub = rospy.Subscriber('new_delivery', DeliveryInfo, newDelivery_cb)
    rospy.spin()
