#!/usr/bin/env python
import roslib
roslib.load_manifest('navigacija')
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from math import degrees, radians

import math

global last_map
global last_face

"""
def callback_pose(data):
        Callback for the topic subscriber.
    Prints the current received data on the topic.
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                                  data.pose.pose.orientation.y,
                                                  data.pose.pose.orientation.z,
                                                  data.pose.pose.orientation.w])
        #rospy.loginfo("Current robot pose: x=" + str(x) + "y=" + str(y) + " yaw=" + str(degrees(yaw)) + "deg")

def callback_map_saver(data):
    global last_map
    last_map = data
    #rospy.loginfo(last_map.data)

def callback_face_saver(data):
    global last_face
    last_face = data
    #rospy.loginfo(data)

def filter_data(data):
    ar = data.ranges
    mx = data.range_max
    mn = data.range_min
    ar =filter(lambda x: not math.isnan(x) and mx > x > mn, ar)

    return list(ar)
    """

class Goto():

        def __init__(self):
                self.face_count = 0
                self.faces = []
                self.goal_sent = False

                #What to do if shut down (e.g. Ctrl-C or failure)
                #rospy.on_shutdown(self.shutdown)

                # Tell the action client that we want to spin a thread by default
                self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
                rospy.loginfo("Wait for the action server to come up")

                #Allow up to 5 seconds for the action server to come up
                self.move_base.wait_for_server(rospy.Duration(5))
                #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
                #rospy.Subscriber("/map", OccupancyGrid, callback_map_saver)
                #rospy.Subscriber("/facemapper/face_loc", MarkerArray, callback_face_saver)
                #rospy.Subscriber('/scan', LaserScan, test_cb)
                rospy.Subscriber("/facemapper/face_loc", MarkerArray, self.face_cb)
                rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_cb)

                markers_topic = rospy.get_param('~markers_topic', rospy.resolve_name('%s/approach_points' % rospy.get_name()))
                self.markers_pub = rospy.Publisher(markers_topic, MarkerArray, queue_size = 300)
                self.markers = MarkerArray()

        def goto(self, pos, quat):

                #Send a goal
                self.goal_sent = True
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                             Quaternion(quat[0],quat[1],quat[2],quat[3]))

                # Start moving
                self.move_base.send_goal(goal)

                # Allow TurtleBot up to 60 seconds to complete task
                success = self.move_base.wait_for_result()

                state = self.move_base.get_state()
                result = False

                if success and state == GoalStatus.SUCCEEDED:
                        result = True
                else:
                        self.move_base.cancel_goal()

                self.goal_sent = False

        def goto2(self, x, y, yaw):
                position = {'x': x, 'y' : y}
                quaternion = quaternion_from_yaw(yaw)
                self.goto(position, quaternion)

        def spinAt(self, x, y):
                print "roaming"
                self.goto2(x, y, 0)
                self.checkFaces()
                self.goto2(x, y, math.pi/2)
                self.checkFaces()
                self.goto2(x, y, math.pi)
                self.checkFaces()
                self.goto2(x, y, 3 * math.pi/2)
                self.checkFaces()

        def pose_cb(self, data):
            self.pos = data.pose.pose.position

        def face_cb(self, data):
            self.faces.append((self.pos, data.markers[len(data.markers) - 1]))

        def checkFaces(self):
            while self.face_count < len(self.faces):
                self.visit(self.faces[self.face_count])
                self.face_count+=1

        def visit(self, tup):
            print "aaaaaaaaproaaaachhhh"

            sel, mar = tup

            mar_x = mar.pose.position.x
            mar_y = mar.pose.position.y
            sel_x = sel.x
            sel_y = sel.y

            phi = math.atan2(mar_y - sel_y, mar_x - sel_x)

            theta = math.pi / 2 - phi

            dist = 0.4
            a_to_b = math.sqrt((sel_y - mar_y)**2 + (sel_x - mar_x)**2)

            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            #marker.pose = Pose()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Time(0)
            marker.id = 111
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA(1, 0, 0, 1)

            if a_to_b < dist:
                print "from " + str((sel_x, sel_y)) + " to + " + str((sel_x, sel_y))
                #marker.pose.position.x = sel_x
                #marker.pose.position.y = sel_y
                #marker.pose.position.z = 0
                #marker.pose.orientation = quaternion_from_yaw(phi)
                #self.markers.markers.append(marker)
                #self.markers_pub.publish(self.markers)
                self.goto2(sel_x, sel_y, phi)
            else:
                #marker.pose.position.x = math.sin(theta) * dist + mar_x
                #marker.pose.position.y = math.cos(theta) * dist + mar_y
                #marker.pose.position.z = 0
                #marker.pose.orientation = quaternion_from_yaw(phi)
                #self.markers.markers.append(marker)
                #self.markers_pub.publish(self.markers)
                print "from " + str((sel_x, sel_y)) + " to + " + str((math.sin(theta) * dist + mar_x, math.cos(theta) * dist + mar_y))
                self.goto2((sel_x - mar_x) / a_to_b * dist + mar_x, (sel_y - mar_y) / a_to_b * dist + mar_y, phi)

            print "zzzzzz(" + str(math.sqrt((sel_y - mar_y)**2 + (sel_x - mar_x)**2)) + ")"
            #rospy.sleep(2)
            print "dobro jutro"

            

        def search(self):
                """ sequence of goals """

                """
                #self.goto2(0.07,  -0.66, 0.2)
                self.spinAt(0.07, -0.06)
                rospy.loginfo("tri")
                #self.goto2(-0.55, -0.11, 0.87)
                self.spinAt(-0.55, -0.11)
                rospy.loginfo("ster")
                #self.goto2(-1.86, -0.2, 1-74)
                self.spinAt(-1.86, -0.2)
                rospy.loginfo("pet")
                #self.goto2(-3.02, -0.72, 1.63)
                self.spinAt(-3.02, -0.72)
                rospy.loginfo("sest")
                #self.goto2(-3.53, -0.43, -3.09)
                self.spinAt(-3.53, -0.43)
                rospy.loginfo("sedm")
                #self.goto2(-3.2, -1.05, -0.35)
                self.spinAt(-3.2, -1.05)
                rospy.loginfo("osm")
                #self.goto2(-1.71, -0.68, -1.64)
                self.spinAt(-1.71, -0.68)
                rospy.loginfo("devet")
                #self.goto2(0.64, -3.42, -1.28)
                self.spinAt(0.64, -3.42)
                rospy.loginfo("ena")
                #self.goto2(0.75, -3.19, 0)
                self.spinAt(0.75, -3.19)
                rospy.loginfo("dva")
                """

                #self.spinAt(-1.87,-0.61)

                
                self.spinAt(-4.66,-0.52)
                self.spinAt(-3.02,-0.35)
                self.checkFaces()
                self.spinAt(-3.34,-1.41)
                self.checkFaces()
                self.spinAt(-2.05,-1.12)
                self.checkFaces()
                self.spinAt(2.31,-0.22)
                self.checkFaces()
                self.spinAt(-1.77,-0.93)
                self.checkFaces()
                self.spinAt(-0.88,-0.89)
                self.checkFaces()
                self.spinAt(-0.32,0.14)
                self.checkFaces()
                self.spinAt(0.34,-1.57)
                self.checkFaces()
                self.spinAt(0.76,-3.13)
                self.checkFaces()
                


        # pos current position
        # marker marker recieved
        def approach(self, pos, marker):
                pass

def quaternion_from_yaw(yaw):
        """yaw in degrees to quaternion"""
        return quaternion_from_euler(0, 0, yaw)

if __name__ == '__main__':
        rospy.init_node('approach')
        try:
                position = {'x': 0.1, 'y' : -0}
                quaternion = quaternion_from_yaw(1.5)

                navigator = Goto()

                navigator.search()

                # Sleep to give the last log messages time to be sent
                rospy.sleep(1)
                rospy.spin()
        except rospy.ROSInterruptException:
                pass
