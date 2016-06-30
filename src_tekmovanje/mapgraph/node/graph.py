#!/usr/bin/env python
#import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from actionlib_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from math import degrees, radians

FRAME_ID = 'map'

# ============================================================================ #
class MapNode:

    def __init__(self, data, seq):
        '''
        data : x, y, color
        seq  : node index
        '''
        self.xpos, self.ypos, self.color = data
        self.seq = seq
        self.conns = []
        self.vstd = False
        self.dist = 0
        self.prev = None

    def __repr__(self):
        return "Node " + str(self.seq) + ": " + str(self.xpos) + ", " + str(self.ypos)

    def distance_to(self, other):
        '''
        Euclidean distance to node or coords other
        '''
        if type(other) == tuple:
            return math.sqrt((self.xpos - other[0])**2 + (self.ypos - other[1])**2)
        else:
            return math.sqrt((self.xpos - other.xpos)**2 + (self.ypos - other.ypos)**2)

    def to_marker(self):
        '''
        Create marker from node.
        '''
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.pose = Pose(Point(self.xpos, self.ypos, 0.5),
                           Quaternion(0, 0, 0, 0))
        if self.color == 'blue':
            marker.color = ColorRGBA(0, 0, 1, 0.2)
        elif self.color == 'green':
            marker.color = ColorRGBA(0, 1, 0, 0.2)
        elif self.color == 'red':
            marker.color = ColorRGBA(1, 0, 0, 0.2)
        elif self.color == 'yellow':
            marker.color = ColorRGBA(1, 1, 0, 0.2)

        marker.action          = Marker.ADD
        marker.header.frame_id = FRAME_ID
        marker.header.stamp    = rospy.Time.now()
        marker.header.seq      = self.seq * 1000
        marker.id              = self.seq * 1000
        marker.frame_locked    = False
        marker.scale           = Vector3(0.5, 0.5, 0.5)

        return marker

    def conn_markers(self):
        '''
        Create markers from connections.
        '''
        markers = []
        for i in range(len(self.conns)):
            marker = Marker()
            marker.type = Marker.LINE_STRIP

            marker.points.append(Point(self.xpos, self.ypos, 0.6))
            marker.points.append(Point(self.conns[i].xpos, self.conns[i].ypos, 0.4))

            if self.color == 'blue':
                marker.color = ColorRGBA(0, 0, 1, 0.2)
            elif self.color == 'green':
                marker.color = ColorRGBA(0, 1, 0, 0.2)
            elif self.color == 'red':
                marker.color = ColorRGBA(1, 0, 0, 0.2)
            elif self.color == 'yellow':
                marker.color = ColorRGBA(1, 1, 0, 0.2)

            marker.action          = Marker.ADD
            marker.header.frame_id = FRAME_ID
            marker.header.stamp    = rospy.Time.now()
            marker.header.seq      = self.seq * 1000 + i + 1 #GRDO
            marker.id              = self.seq * 1000 + i + 1 #GRDO
            marker.frame_locked    = False
            marker.scale           = Vector3(0.1, 0.1, 0.1)

            markers.append(marker)

        return markers

# ============================================================================ #
class MapGraph:

    def __init__(self, nodes, conns):
        '''
        nodes : list of triples of (x, y, color)
        conns : list of tuples of (node_index, node_index)
        '''
        self.nodes = []
        self.markers = MarkerArray()

        # Create array of nodes and MarkerArray
        for i in range(len(nodes)):
            self.nodes.append(MapNode(nodes[i], i))
            self.markers.markers.append(self.nodes[i].to_marker())

        for i in range(len(conns)):
            self.nodes[conns[i][1]].conns.append(self.nodes[conns[i][0]])
            self.nodes[conns[i][0]].conns.append(self.nodes[conns[i][1]])
            self.markers.markers += self.nodes[conns[i][0]].conn_markers()
            self.markers.markers += self.nodes[conns[i][1]].conn_markers()

        # Init topics, etc.
        markers_topic = rospy.get_param('~markers_topic',
                                        rospy.resolve_name('%s/mapnodes' %
                                                           rospy.get_name()))
        self.markers_pub = rospy.Publisher(markers_topic,
                                           MarkerArray, queue_size = 300)
        self.dijkstra(self.nodes[0])

    def drop_connection(self, frm ,to):
        '''
        conn : tuple representing the conn to drop.
        '''
        #frm_n = self.node(frm)
        #to_n  = self.node(to)
        if to in frm.conns:
            frm.conns.remove(to);
        else:
            print "couldn't drop."

    # ------------------------------------------------------------------------ #
    # Markers
    # ------------------------------------------------------------------------ #
    def show_markers(self):
        '''
        Publish all markers.
        '''
        self.markers_pub.publish(self.markers)

    def highlight_path(self, path):
        '''
        Color all the nodes in path white.
        '''
        self.reset_marker_colors()

        for n in path:
            i = n.seq
            self.markers.markers[i].color = ColorRGBA(1, 1, 1, 0.1)

        self.show_markers()

    def reset_marker_colors(self):
        '''
        Reset the colors of all the nodes to their default.
        '''
        for i in range(len(self.nodes)):
            n = self.nodes[i]
            m = self.markers.markers[i]
            if n.color == 'blue':
                m.color = ColorRGBA(0, 0, 1, 0.1)
            elif n.color == 'green':
                m.color = ColorRGBA(0, 1, 0, 0.1)
            elif n.color == 'red':
                m.color = ColorRGBA(1, 0, 0, 0.1)
            elif n.color == 'yellow':
                m.color = ColorRGBA(1, 1, 0, 0.1)

        self.show_markers()

    # ------------------------------------------------------------------------ #
    # Pathfinding
    # ------------------------------------------------------------------------ #
    def dijkstra(self, start):
        '''
        set dist and prev to all nodes in graph.
        start : node.
        '''
        start = self.node(start)
        Q     = self.nodes[:]

        # init distances, visited and previous nodes
        for n in Q:
            n.dist = float('inf')
            n.vstd = False
            n.prev = None

        start.dist = 0

        while len(Q) > 0:

            dists = map(lambda n: n.dist, Q)
            min_i = dists.index(min(dists))
            curr  = Q[min_i]
            Q.remove(curr)

            for n in curr.conns:
                alt = curr.dist + curr.distance_to(n)
                if alt < n.dist:
                    n.dist = alt
                    n.prev = curr

    def node(self, nd):
        '''
        nd : index, node, position or color
        '''
        if type(nd) == int:
            return self.nodes[nd]

        if type(nd) == tuple:
            dists = map(lambda n: n.distance_to(nd), self.nodes)
            return self.nodes[dists.index(min(dists))]

        if type(nd) == str:
            targets = filter(lambda n: n.color == nd, self.nodes)
            t_dists = map(lambda n: n.dist, targets)
            return targets[t_dists.index(min(t_dists))]

        return nd

    def find_path(self, start, finish):
        '''
        Find path from start index to finish index, node, position or color.
        '''

        start = self.node(start)
        self.dijkstra(start)
        curr  = self.node(finish)
        path  = []

        # Build path
        while curr != start:
            if curr.prev == None:
                rospy.loginfo("Can't find path!")
                return []

            path.append(curr)
            curr = curr.prev

        path.append(start)

        # Result
        path.reverse()
        return path, self.path_angles(path)

    def path_angles(self, path):
        '''
        Get yaw for each node in path.
        '''
        qs = []

        for i in range(1,len(path)):
            a = {'x' : path[i-1].xpos, 'y' : path[i-1].ypos}
            b = {'x' : path[i].xpos,   'y' : path[i].ypos}

            c = {'x' : a['x'] - b['x'], # c = a-b
                 'y' : a['y'] - b['y']}

            nc = c['x']**2 + c['y']**2  # norm of a

            #print c['x']/nc
            #qs.append(math.acos(c['x'] / nc))
            qs.append(math.atan2(b['y']-a['y'], b['x']-a['x']))

        qs.append(0)

        #qs.reverse()
        return qs

# ============================================================================ #


if __name__ == '__main__':

    rospy.init_node('mapgraph')

    # TEST
    try:
        graph = MapGraph([(0,0,'red'),    # 0
                          (0,1,'blue'),   # 1
                          (1,0,'blue'),   # 2
                          (1,1,'blue'),   # 3
                          (1,2,'green'),  # 4
                          (2,1,'green'),  # 5
                          (2,2,'green')], # 6
                         [(0,1), (1,2), (2,3), (3,4), (4,5), (5,6), (0,2), (3,5)])

        rospy.sleep(5)
        graph.show_markers()

        p = graph.find_path(0, (2,2))
        graph.highlight_path(p)
        rospy.sleep(2)
        print p
        p = graph.find_path(0, "green")
        graph.highlight_path(p)
        print p

        rospy.sleep(5)
        graph.reset_marker_colors()

    except rospy.ROSInterruptException:
        pass
