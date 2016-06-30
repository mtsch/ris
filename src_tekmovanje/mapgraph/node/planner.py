#!/usr/bin/env python
from graph import *
#import roslib
#roslib.load_manifest('planner.py')
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
import random
import numpy as np

FRAME_ID = 'map'

class Planner:

    def __init__(self, mapgraph):
        # Subscribers
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print 'Waiting for move_base server...'
        self.move_base.wait_for_server() #rospy.Duration(3))
        print '[OK]'

        # Variables
        self.actions = []
        self.mapgraph = mapgraph
        self.curr_pos = None
        self.last_target_pos = None
        self.mapgraph.show_markers()

        # List of known faces and their positions.
        self.known_buildings = {}
        self.known_faces     = {}
        self.signs = []
        self.visited_signs = []

        # Resposnes
        self.ok_responses = ['Affirmative.', 'Your wish is my command.', 'Ok.',
                             'Sure.', 'Okie dokie.', 'Why not?', 'Deal.']
        self.what_responses = ['What?', 'Say again?', "I didn't get that.",
                               'Huh?', 'Please repeat.', "I don't understand you.",
                               'What what what what?', 'Wat?', 'Wut?', 'Eh?']
        self.repeat_responses = ['Repeat your command.', 'What then?',
                                 'What do you want from me?', 'Ok then. What do you want me to do?']
        self.last_voice_command = None

        self.last_action = None

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb)
        rospy.Subscriber('/voice_control/output', String, self.voice_cb)
        rospy.Subscriber('/rec_faces', MarkerArray, self.face_cb)
        rospy.Subscriber('/markers_cilindrov', MarkerArray, self.buil_cb)
        #rospy.Subscriber('/todo', MarkerArray, self.buil_cb)
        #rospy.Subscriber('/sign_detector/markers', MarkerArray, self.sign_cb)
        rospy.Subscriber('/sign_detector/markers_tmp', Marker, self.sign_cb)

        # Publishers
        self.voice_pub = rospy.Publisher('recognizerStartStop', Bool, queue_size = 3)
        self.speak_pub = rospy.Publisher('speak/text', String, queue_size = 3)
        self.street_pub = rospy.Publisher('planner/street_color', String, queue_size = 3)


    # ------------------------------------------------------------------------ #
    # Subscribers
    # ------------------------------------------------------------------------ #

    def pose_cb(self, data):
        '''
        Pose callback - save current position to curr_pos.
        '''
        position = data.pose.pose.position
        self.curr_pos = (position.x, position.y)
        dists   = map(lambda n: n.distance_to(self.curr_pos), self.mapgraph.nodes)
        least_i = np.argmin(dists)
        self.street_pub.publish(String(self.mapgraph.nodes[least_i].color))

    def voice_cb(self, data):
        '''
        React to voice commands.
        '''
        self.last_voice_command = data.data.lower().split(' ')
        self.voice_onoff(False)
        # topic =
        # recognizerStartStop <- true, false
        # voice_control/output <- sub
        # 'street ime barva'
        # 'building barva'

    def sign_cb(self, data):
        '''
        React to new signs.
        '''
        self.signs = []

        if data.text == 'stop':
            print 'stop detected'
            self.actions.insert(0, ('stop',))
        elif data.text == 'one_way':
            print 'Removing connection.'
            self.remove_connection(data)
            self.recalculate_route()

    def face_cb(self, data):
        '''
        Add new detected faces to known_faces.
        '''
        print 'Found a face!'
        #faces = ['Forrest', 'Harry', 'Scarlett', 'Tina',
        #         'Peter', 'Kim', 'Filip', 'Matthew', 'Ellen']
        for f in data.markers:
            #face = faces[f.id]
            face = f.text.lower()
            if not face in self.known_faces:
                print face + " detected."
                self.known_faces[face] = (f.pose.position.x, f.pose.position.y)

        print 'Known:' +  str(self.known_faces) + '.'

    def buil_cb(self, data):
        '''
        Add new detected buildings to known_buildings.
        '''
        print "Found a building!"

        for b in data.markers:
            col = b.text
            if not col in self.known_buildings:
                print col + " building detected."
                self.known_buildings[col] = (b.pose.position.x, b.pose.position.y)

        print 'Known: ' + str(self.known_buildings)

    # ------------------------------------------------------------------------ #
    # Misc
    # ------------------------------------------------------------------------ #
    def voice_onoff(self, opt):
        '''
        Turn voice commands on or off by setting opt to true or false.
        '''
        self.voice_pub.publish(opt)
        pass

    def say(self, text):
        '''
        Speak the text.
        '''
        self.speak_pub.publish(text)

    def ask_yesno(self, text):
        '''
        Repeat command and ask for yes or no
        '''
        self.say(text)
        while True:
            resp = self.get_voice_command()
            if resp[0] == 'yes':
                return True
            elif resp[0] == 'no':
                return False


    def get_voice_command(self):
        '''
        Get a single voice command.
        '''
        self.last_voice_command = None
        self.voice_onoff(True)
        while not self.last_voice_command:
            pass
        return self.last_voice_command

    def remove_connection(self, sign):
        '''
        Remove a connection from graph.
        '''

        dist = lambda x1, y1, x2, y2: math.sqrt((x1-x2)**2 + (y1-y2)**2)

        # Get nearest connection to sign
        sign_x = sign.pose.position.x
        sign_y = sign.pose.position.y

        nearest1 = None
        nearest1_d = np.inf
        nearest2 = None
        nearest2_d = np.inf
        for n in self.mapgraph.nodes:
            d = dist(n.xpos, n.ypos, sign_x, sign_y)
            if d < nearest1_d:
                nearest2 = nearest1
                nearest2_d = nearest1_d
                nearest1 = n
                nearest1_d = d
            elif d < nearest2_d:
                nearest2 = n
                nearest2_d = d

        print 'From ' + str(nearest1) + ' to ' + str(nearest2)

        while not self.curr_pos:
            pass

        cpx, cpy = self.curr_pos
        if dist(nearest1.xpos, nearest1.ypos, cpx, cpy):
            self.mapgraph.drop_connection(nearest1, nearest2)
        else:
            self.mapgraph.drop_connection(nearest2, nearest1)

    def last_pos(self):
        '''
        Get last position in queue.
        '''
        if self.last_target_pos == None:
            while self.curr_pos == None:
                pass
            pos = self.curr_pos
        else:
            pos = self.last_target_pos
        return pos

    def quaternion_from_yaw(self, yaw):
        '''
        Yaw in radians to quaternion
        '''
        return quaternion_from_euler(0, 0, yaw)


    # ------------------------------------------------------------------------ #
    # Main
    # ------------------------------------------------------------------------ #
    def run(self):
        '''
        Repeatedly execute actions.
        '''
        self.add_say('Hello.')
        self.add_wait_for_command('from_operator')
        #self.add_wait_for_command('from_person')

        waiting = False
        while True and not rospy.is_shutdown():
            waiting = self.perform_next_action(waiting)

    # ------------------------------------------------------------------------ #
    # Navigation
    # ------------------------------------------------------------------------ #
    def clear_actions(self):
        '''
        Clear all actions.
        '''
        self.last_target_pos = None
        self.actions = []

    def perform_next_action(self, waiting):

        # TODO tuki:
        # interrupti. mora cekirat spremenljivke ce je kej novga

        if len(self.actions) == 0:
            if not waiting:
                print 'Out of actions.'

            rospy.sleep(0.1)
            return True

        action = self.actions.pop(0)

        # ......................................................................
        if action[0] == 'goto':
            target = action[1]
            sys.stdout.write('Going to ' + str(target))
            q = self.quaternion_from_yaw(target['q'])

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = FRAME_ID
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(target['x'], target['y'], 0.000),
                                         Quaternion(q[0], q[1], q[2], q[3]))

            self.move_base.send_goal(goal)

            success = self.move_base.wait_for_result()
            print ' :^(' if not success else ' :^)'
            state = self.move_base.get_state()

            result = False

            if success and state == GoalStatus.SUCCEEDED:
                result = True
            else:
                self.move_base.cancel_goal()


        # ......................................................................
        elif action[0] == 'found?':
            if action[1] == 'building' and action[2] in self.known_buildings:
                print 'Found goal: ' + action[2] + ' building.'
                self.clear_actions()
                self.add_approach(self.known_buildings[action[2]])
                self.add_say('You have arrived at your destination.')
                self.add_wait_for_command('from_operator')

            elif action[1] in self.known_faces:
                print 'Found goal: ' + action[1] + '.'
                self.clear_actions()
                self.add_approach(self.known_faces[action[1]])
                self.add_say('Hello, ' + action[1] + '. Where do you want to go?')
                self.add_wait_for_command('from_person')

        # ......................................................................
        elif action[0] == 'say':
            print 'Saying "' + action[1] + '"'
            self.say(action[1])
            rospy.sleep(1)

        elif action[0] == 'stop':
            self.say('I stopped.')
            print 'Stopping.'
            rospy.sleep(3)

        # ......................................................................
        elif action[0] == 'wait_for_command':
            print 'Waiting for command ' + action[1]
            got_command = False
            while not got_command:
                data = self.get_voice_command()

                if len(data) == 3 and \
                   data[0] == 'street' and \
                   action[1] == 'from_operator':

                    resp = self.ask_yesno('Should I find ' + data[1] + \
                                          ' on ' + data[2] + ' street?')
                    if resp:
                        print 'Finding ' + data[1] + ' on ' + data[2] + ' street.'
                        self.add_search_for(data[1], data[2])
                        self.say(self.ok_responses[
                            random.randint(0,len(self.ok_responses)-1)])
                        got_command = True
                    else:
                        self.say(self.repeat_responses[
                            random.randint(0,len(self.repeat_responses)-1)])

                elif len(data) == 2 and \
                     data[0] == 'building' and \
                     action[1] == 'from_person':

                    resp = self.ask_yesno('To the ' + data[1] + ' building?')
                    if resp:
                        print 'Finding ' + data[1] + ' building.'
                        self.add_search_for(data[0], data[1])
                        self.say(self.ok_responses[random.randint(0,len(self.ok_responses)-1)])
                        print 'Finding ' + data[1] + ' building.'
                        got_command = True
                    else:
                        self.say(self.repeat_responses[
                            random.randint(0,len(self.repeat_responses)-1)])

                else:
                    self.say(self.what_responses[random.randint(0,len(self.what_responses)-1)])


        # ......................................................................
        else:
            print 'Unknown action ' + str(action[0]) + '!'
            self.say('hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh')

        return False

    # ------------------------------------------------------------------------ #
    # Action adding.
    # ------------------------------------------------------------------------ #
    def add_goto(self, target):
        '''
        Go to target. Can be a node, node index, (x, y) or 'color'
        '''
        # Set starting position.
        pos = self.last_pos()

        # Find shortest path.
        path, qs = self.mapgraph.find_path(pos, target)

        # Show path
        self.mapgraph.highlight_path(path)

        if len(path) > 0:
            last = path[-1]
        else:
            return

        # Set last position in queue
        self.last_target_pos = (last.xpos, last.ypos)

        # Add path to action queue
        for i in range(len(path)):
            # TODO izracuni q
            self.actions.append(('goto', {'x' : path[i].xpos,
                                          'y' : path[i].ypos,
                                          'q' : qs[i]}))

    def add_say(self, text):
        '''
        Say [text].
        '''
        self.actions.append(('say', text))

    def add_spin(self, thing, color):
        '''
        Spin at last position and check if thing was found.
        '''
        # Set position.
        x,y = self.last_pos()

        # Append spinning actions.
        for i in range(16):
            self.actions.append(('found?', thing, color))
            self.actions.append(('goto', {'x' : x, 'y' : y, 'q' : i*math.pi/8}))

    def add_search_street(self, thing, color):
        '''
        Go to street and search it.
        '''

        all_nodes = self.mapgraph.nodes
        nodes = filter(lambda n: n.color == color, all_nodes)
        nodes = map(lambda n: n.seq, nodes)

        if len(nodes) == 0:
            print 'Unknown color!'
            return

        # Decide on searching direction.
        self.mapgraph.dijkstra(self.last_pos())
        s_node = self.mapgraph.nodes[nodes[0]]
        e_node = self.mapgraph.nodes[nodes[-1]]
        if s_node.dist > e_node.dist:
            nodes.reverse()

        for n in nodes:
            #rospy.sleep(0.5)
            self.add_goto(n)
            self.add_spin(thing, color)


    def add_search_for(self, thing, color):
        '''
        Search the [color] street for thing.
        '''
        self.last_action = ('search', thing, color, self.last_pos())

        # Go to thing if you already know it.
        print thing
        if thing == 'building' and color in self.known_buildings:
            self.add_goto(known_building[color])
            self.add_approach(self.known_faces[thing])
            self.add_say('You have arroved at your destination.')
            self.add_wait_for_command('from_operator')
        elif thing in self.known_faces:
            self.add_goto(self.known_faces[thing])
            self.add_approach(self.known_faces[thing])
            self.add_say('Hello, ' + action[1] + '. Where do you want to go?')
            self.add_wait_for_command('from_person')
        else:
            self.add_search_street(thing, color)
            self.add_search_street(thing, color)
            self.add_search_street(thing, color)
            self.add_say("I don't want to do this anymore. I give up.")
            self.add_wait_for_command('from_operator')

    def add_approach(self, coords):
        '''
        Approach coords.
        '''
        print 'Approaching ' + str(coords)
        dist = 1
        # Go to nearest node.
        self.add_goto(coords)

        # Approach.
        sel_x, sel_y = self.last_pos()
        tar_x, tar_y = coords
        phi = math.atan2(tar_y - sel_y, tar_x - sel_x)
        a_to_b = math.sqrt((sel_y - tar_y)**2 + (sel_x - tar_x)**2)

        if a_to_b < dist:
            self.actions.append(('goto', {'x' : sel_x, 'y' : sel_y, 'q' : phi}))
            self.last_target_pos = (sel_x, sel_y)
        else:
            self.actions.append(('goto',
                                 {'x' : (sel_x - tar_x) / a_to_b * dist + tar_x,
                                  'y' : (sel_y - tar_y) / a_to_b * dist + tar_y,
                                  'q' : phi}))
            self.last_target_pos = ((sel_x - tar_x) / a_to_b * dist + tar_x,
                                    (sel_y - tar_y) / a_to_b * dist + tar_y)

    def add_stop(self):
        self.actions.append(('stop',))

    def add_wait_for_command(self, from_who):
        self.last_action = None
        self.actions.append(('wait_for_command', from_who))

    def recalculate_route(self):
        self.say('Recalculating route.')
        if self.last_action != None:
            self.clear_actions()
            self.add_search_for(last_action[1], last_action[2])
        else:
            goto = self.last_pos()
            self.clear_actions()
            self.add_goto(goto)

if __name__ == '__main__':

    rospy.init_node('planner')

    try:

        print 'Init planner.'
        # Init graph:
        planner = Planner(MapGraph(
            [ (-3.2, 1.9, 'red'),         # 0
              (-2.6, 2.3, 'red'),         # 1
              (-2.5, 1.5, 'red'),         # 2
              (-2.3, 1.1, 'red'),         # 3
              (-2.0, 0.5, 'red'),         # 4

              (-1.9, -0.1, 'yellow'),     # 5
              (-1.6, -0.8, 'yellow'),     # 6
              (-1.5, -1.5, 'yellow'),     # 7

              (-0.9, 0.2, 'green'),       # 8
              ( 0.1, 0.3, 'green'),       # 9
              ( 0.9, 0.0, 'green'),       # 11
              ( 1.0, 0.7, 'green'),       # 10
              ( 1.8, 0.9, 'green'),       # 12
              ( 2.6, 1.0, 'green'),       # 13

              (-0.8, -1.3, 'blue'),       # 14
              ( 0.2, -1.0, 'blue'),       # 15
              ( 1.1, -0.8, 'blue'),       # 16
              ( 2.0, -0.6, 'blue'),       # 17
              ( 2.8, -0.3, 'blue'),       # 18
              ( 2.7,  0.3, 'blue')        # 19

          ], [(0,1), (1,2), (0,2), (2,3), (3,4),
              (4,5), (5,6), (6,7),
              (5,8), (8,9), (9,11), (9,10), (10,11), (11,12), (12,13),
              (7,14), (14,15), (15,16), (16,17), (17,18), (18,19), (19,13), (16,10)
          ]
        ))


        planner.mapgraph.show_markers()

        print 'Waiting...'
        rospy.sleep(3)
        print '[OK]'
        #planner.add_goto(0)
        #planner.add_goto(13)
        #planner.add_search_street('green')
#       planner.add_search_for('building', 'red')
#       planner.add_search_for('building', 'yellow')
#       planner.add_search_for('building', 'blue')
#       planner.add_search_for('building', 'green')
#       planner.add_goto(13)
#       planner.add_goto(0)
#       planner.add_goto('blue')
#       planner.add_goto(13)
#       planner.add_say('woop woop woop')
#       planner.add_search_for('KKKKKKK', 'blue')

        print 'Run.'
        planner.say('har har har')
        planner.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        planner.voice_onoff(False)
        pass
