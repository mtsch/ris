#!/usr/bin/env python
import message_filters
import collections
from localizer.srv import Localize
import roslib
roslib.load_manifest('sign_detector')
import rospy
import math
import tf
import actionlib
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseStamped, Vector3
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from math import degrees, radians
from detection_msgs.msg import Detection
from cv_bridge import CvBridge
import cv2
import random
import numpy as np
import matplotlib.pyplot as pyplot
import matplotlib.image as mpimg
import os
import rospkg
from image_geometry import PinholeCameraModel
from sound_play.libsoundplay import SoundClient

THRESH = 10
BIN_THRESH = 4

class SignDetector:

    def __init__(self):
        self.bridge = CvBridge()
        path = rospkg.RosPack().get_path('sign_detector')
        self.signs = []
        self.signs.append(cv2.imread(path + '/horn.png'))
        self.signs.append(cv2.imread(path + '/mandatory_left.png'))
        self.signs.append(cv2.imread(path + '/one_way.png'))
        self.signs.append(cv2.imread(path + '/speed_limit.png'))
        self.signs.append(cv2.imread(path + '/stop.png'))

        self.sign_names  = np.array(['horn', 'mandatory_left', 'one_way', 'speed_limit', 'stop'])
        self.sign_colors = np.array([True,   True,             False,     False,         False])

        self.bridge = CvBridge()

        self.hog = cv2.HOGDescriptor()
        self.sc = SoundClient()
        self.time_to_honk = True
        self.time_of_last_honk = rospy.Time(0)

        #self.hists = []
        self.hogs = []

        for s in self.signs:
            s = s * 100
            s = cv2.resize(s, (130,130))
            h = self.hog.compute(s)
            self.hogs.append(h.reshape(len(h)))
            #plt.imshow(s)
            #plt.show()

        self.listener    = tf.TransformListener()
        self.transformer = tf.TransformerROS(True, rospy.Duration(10.0))

        self.region_scope = rospy.get_param('~region', 3)
        self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
        rospy.wait_for_service('localizer/localize')

        self.camera_infos = collections.deque(maxlen = self.buffer_size)
        self.camera_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.camera_sub.registerCallback(self.camera_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)

        self.det_sub = message_filters.Subscriber('/detector/traffic_signs', Detection)
        self.det_sub.registerCallback(self.detect_sign)

        self.last_id = 0

        self.markers_pub = rospy.Publisher('/sign_detector/markers', MarkerArray, queue_size = 300)
        self.tmpmarkers_pub = rospy.Publisher('/sign_detector/markers_tmp', Marker, queue_size = 300)
        self.image_pub = rospy.Publisher('/sign_detector/image', Image, queue_size = 300)
        rospy.sleep(2)

        self.sign_bins = []

        print "Sign detector ready."

    def camera_callback(self, camera_info):
        self.camera_infos.append(camera_info)

    def detect_sign(self, data):

        #print '!'

        # Header header
        # int32 x
        # int32 y
        # int32 height
        # int32 width
        # string source
        # string label
        # float32 confidence
        # sensor_msgs/Image image

        xy = self.transform_coords(data)
        if xy != None:
            x, y = xy
            #print x,y

            # Upostevat x,y in image
            im = np.asarray(self.bridge.imgmsg_to_cv2(data.image, 'bgr8'))
            im = self.crop_image(im, (130,130))

            if im != None:
                self.detect_color(im)
                best = self.hog_class(im, 'X')
                if best != None:
                    print 'Detected: ' + self.sign_names[best] + '.'
                    self.add_sign_and_publish(best, x, y)
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(
                            im[:,:,[2,1,0]].astype(np.uint8) , "bgr8"))
                    if self.sign_names[best] == 'horn' and \
                       rospy.Time.now() - self.time_of_last_honk > rospy.Duration(20):
                        self.sc.playWave('/home/team_alpha/honk.wav')
                        self.time_to_honk = False
                        self.time_of_last_honk = rospy.Time.now()
                    elif self.sign_names[best] == 'horn':
                        print 'Time to next honk:', \
                            (rospy.Duration(20) - (rospy.Time.now() - self.time_of_last_honk)) / 1e9


        else:
            pass
            #print "Problem localizing!"



    def dist(self, q, v, typ='KL'):
        '''
        Calculate distance between histograms.
        '''
        if typ == 'KL':
            d = (q * np.log(q / v))
            return np.sum(d[~np.isnan(d) & ~np.isinf(d)])
        elif typ == 'H':
            return np.sqrt(1 - np.sum(np.sqrt(q * v)))
        elif typ == 'X':
            d = (((q - v) ** 2) / (q + v))
            return np.sum(d[~np.isnan(d) & ~np.isinf(d)])
        else:
            pass
            print 'Unknown dist ' + typ + '!'

    def crop_image(self, im, size):
        '''
        Crop [im] and resize it to [size]
        '''
        cutoff = np.std(im, axis=2) > THRESH

        # Find bounds by x and y
        x_lb = 0
        while np.sum(cutoff[x_lb,:]) < cutoff[1,:].size/10 and x_lb < im.shape[0]-1:
            x_lb+= 1

        x_ub = cutoff.shape[0]-1
        while np.sum(cutoff[x_ub,:]) < cutoff[1,:].size/10 and x_ub < im.shape[0]-1:
            x_ub-= 1

        y_lb = 0
        while np.sum(cutoff[:,y_lb]) < cutoff[:,1].size/10 and y_lb < im.shape[1]-1:
            y_lb+= 1

        y_ub = cutoff.shape[1]-1
        while np.sum(cutoff[:,y_ub]) < cutoff[:,1].size/10 and y_ub < im.shape[1]-1:
            y_ub-= 1

        if x_ub - x_lb < 10 or y_ub - x_lb < 10: return None

        im = im[x_lb:x_ub, y_lb:y_ub, [2,1,0]]
        im = cv2.resize(im, size)

        return im

    def hog_class(self, im, dist_typ):
        '''
        Classify by computing the distances between HOGs.
        Return non-normalized probability probability distribution.
        '''

        imh = self.hog.compute(im)
        imh = imh.reshape(len(imh))
        dists = []

        for h in self.hogs:
            dists.append(self.dist(imh, h, dist_typ))


        dists = np.asarray(dists)
        dists[3] += 250  # TODO GRDO

        print "Dists: " + str(dists)

        if np.min(dists) > 4500:
            return None

        col = self.detect_color(im)

        if col == "blue":
            cand_names = self.sign_names[self.sign_colors]
            cand_dists = dists[self.sign_colors]
            return np.argmin(np.hstack([cand_dists, np.inf, np.inf, np.inf]))
            #return np.hstack([1 - cand_dists / np.sum(cand_dists), 0, 0, 0])
        else:
            cand_names = self.sign_names[~self.sign_colors]
            cand_dists = dists[~self.sign_colors]
            best = np.argmin(np.hstack([np.inf, np.inf, cand_dists]))
            if best == 3 and np.min(dists) > 3000:
                return
            else:
                return np.argmin(np.hstack([np.inf, np.inf, cand_dists]))
            #return np.hstack([0, 0, 1 - cand_dists / np.sum(cand_dists)])

    def detect_color(self, im):
        '''
        Detect the color of a sign.
        '''
        r = np.mean(im[:,:,0])
        g = np.mean(im[:,:,1])
        b = np.mean(im[:,:,2])

       #print "..."
       #print 'r' + str(r)
       #print 'g' + str(g)
       #print 'b' + str(b)
       #print 's' + str(r+g+b)
       #print "..."

        if r > b:
            return "red"
        else:
            return "blue"

    def transform_coords(self, det):
        '''
        Transform a det from base_link to map.
        '''
        u = det.x + det.width / 2
        v = det.y + det.height / 2

        camera_info = None
        best_time = 100
        for i in range(len(self.camera_infos)):
            ci = self.camera_infos[i]
            time = abs(ci.header.stamp.to_sec() - det.header.stamp.to_sec())
            if time < best_time:
                camera_info = ci
                best_time = time

        if not camera_info:
            #print 'not camera_info'
            return None
        if best_time > 2:
            #print 'best_time > 1 (' + str(best_time) + ')'
            return None

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
             ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)

        localization = self.localize(det.header, point, self.region_scope)

        if not localization:
            #print 'not localization'
            return None

        #print localization.pose.position.z
        #print localization.pose.position.x
        if (localization.pose.position.x == 0 and localization.pose.position.y == 0) \
           or localization.pose.position.z > 2:
            return None


        now = det.header.stamp
        stampedPose                 = PoseStamped()
        stampedPose.pose            = localization.pose
        stampedPose.header.stamp    = now
        stampedPose.header.frame_id = det.header.frame_id

        self.listener.waitForTransform('map', stampedPose.header.frame_id, now, rospy.Duration(3.0))
        newPose = self.listener.transformPose('map', stampedPose)

        return newPose.pose.position.x, newPose.pose.position.y


    def add_sign_and_publish(self, best, x, y, delta = 2):
        '''
        Add sign to sign_bins
        '''

        changed = False
        found  = False
        for b in self.sign_bins:
            if math.sqrt((b['x'] - x)**2 + (b['y'] - y)**2) < delta:
                found = True
                oldbest = np.argmax(b['d'])
                oldval  = np.max(b['d'])
                b['d'][best] += 1
                newbest = np.argmax(b['d'])
                newval  = np.max(b['d'])
                #print oldbest
                #print newbest
                if newbest != oldbest:
                    print 'Sign changed from ' + \
                       self.sign_names[oldbest] + ' to ' + \
                       self.sign_names[newbest]
                    b['changed'] = True
                    changed = True
                if oldval <= BIN_THRESH and newval > BIN_THRESH:
                    changed = True
                    b['changed'] = True
                break

        if not found:
            d = np.zeros(5)
            d[best] += 1
            self.sign_bins.append({'x' : x, 'y' : y, 'd' : d, 'changed' : False})
            print 'Sign ' + self.sign_names[np.argmax(d)] + ' added.'
            changed = True

        # Publish single marker if something has changed
        if changed:
            for c in self.sign_bins:
                if c['changed'] and max(c['d']) > BIN_THRESH:
                    c = self.sign_bins[-1]
                    m = Marker()
                    m.pose            = Pose(Point(c['x'], c['y'], 0.05),
                                             Quaternion(0, 0, 0, 0))
                    m.color           = ColorRGBA(1,0,0,1)
                    m.action          = Marker.ADD
                    m.header.frame_id = 'map'
                    m.header.stamp    = rospy.Time.now()
                    m.header.seq      = self.last_id
                    m.id              = self.last_id
                    m.lifetime        = rospy.Duration.from_sec(3)
                    m.frame_locked    = False
                    m.scale           = Vector3(0.1,0.1,0.1)
                    m.type            = Marker.TEXT_VIEW_FACING
                    m.ns              = 'signs'
                    #m.type            = Marker.SPHERE
                    m.text            = self.sign_names[np.argmax(c['d'])]

                    c['changed'] = False
                    self.tmpmarkers_pub.publish(m)
                    self.last_id += 1

        # Publish all markers if something has changed.
        if changed:
            markers = []
            print 'pushing.'
            for i in range(len(self.sign_bins)):
                c = self.sign_bins[i]
                if np.max(c['d'] > BIN_THRESH):
                    m = Marker()
                    m.pose            = Pose(Point(c['x'], c['y'], 0.05),
                                             Quaternion(0, 0, 0, 0))
                    m.color           = ColorRGBA(1,0,0,1)
                    m.action          = Marker.ADD
                    m.header.frame_id = 'map'
                    m.header.stamp    = rospy.Time.now()
                    m.header.seq      = i
                    m.id              = i
                    m.frame_locked    = False
                    m.scale           = Vector3(0.1,0.1,0.1)
                    m.type            = Marker.TEXT_VIEW_FACING
                    m.ns              = 'signs'
                    #m.type            = Marker.SPHERE
                    m.text            = self.sign_names[np.argmax(c['d'])]
                    markers.append(m)

            marr = MarkerArray()
            marr.markers = markers
            self.markers_pub.publish(marr)
            #print 'Num markers: ' + str(len(markers)) + '.'

if __name__ == '__main__':

    rospy.init_node('sign_detector')

    try:
        sd = SignDetector()
        #print 'Sign detector initialized.'
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
