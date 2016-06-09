#!/usr/bin/env python

#robot.py implementation goes here
import astar
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from mdp import mdp
import rospy
from std_msgs.msg import Bool
from medic_bot.msg import AStarPath, PolicyList
from sensor_msgs.msg import Image
class Robot():
    def __init__(self):

        rospy.init_node('Robot', anonymous=True)

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector( cv2.HOGDescriptor_getDefaultPeopleDetector() )
        
        self.avg_width = 0.45
        
        self.path_pub = rospy.Publisher(
            "results/path_list",
            AStarPath,
            queue_size = 10
        )
        self.policy_pub = rospy.Publisher(
            "results/policy_list",
            PolicyList,
            queue_size = 10
        )
        self.sim_pub = rospy.Publisher(
            "map_node/sim_complete",
            Bool,
            queue_size = 10
        )
        self.cam_sub = rospy.Subscriber(
            "/camera/visible/image",
            Image,
            self.detect_people
        )
        path_list = astar.astar()
        for pl in path_list:
            asp = AStarPath()
            asp.data = pl
            rospy.sleep(1)
            self.path_pub.publish(asp)

        markov = mdp()
        pl = PolicyList()
        pl.data = markov.policy_list
        rospy.sleep(1)

        
        rospy.sleep(1)
        rospy.sleep(1)
        self.policy_pub.publish(pl)
        #print markov.policy_list
        #for m in markov.policy_list:
            #pl = PolicyList()
            #pl.data = m
            #rospy.sleep(1)
            #self.policy_pub.publish(pl)
    
        rospy.sleep(1)
        rospy.signal_shutdown("Done.")

    def detect_people(self, img):
        self.frame_width = img.width
        cv_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="bgr8")


        print "what the fuk"
        found, w = self.hog.detectMultiScale(cv_image, winStride=(8,8), padding=(32,32), scale=1.05)
        found_filtered = []
        for ri, r in enumerate(found):
            for qi, q in enumerate(found):
                if ri != qi and inside(r, q):
                    break
            else:
                found_filtered.append(r)
        x, y, w, _ = found_filtered[0]
        self.calculate_distance(x, w)

        if self.cur_distance <= 1.0:
            rospy.sleep(1)
            rospy.signal_shutdown("Done.")

        self.calculate_goal()

        self.markov = mdp(self.goal)
        self.pl = PolicyList()
        self.pl.data = markov.policy_list

        self.make_move()

    def calculate_distance(self, x, w):
              
        # Z'=D*f/d' 
        self.cur_distance = self.avg_width * self.cam_focus / w
        
        #Find the middle point of person
        mid_point = x + w/2.0
        # Ratio of width of person in pixels to the image
        x_ratio = mid_point/self.frame_width
        # Angle at which person is from the left end of vision
        agle = x_ratio/self.camera_fov
        # True angle from the middle of vision
        self.angle = (self.camera_fov/2) - agle

    def calculate_goal(self):
        
        new_x = self.cur_distance * math.sin(self.angle)
        new_y = self.cur_distance * math.cos(self.angle)

        self.goal = [new_x, new_y]

if __name__ == '__main__':
    try:
        Robot()
    except rospy.ROSInterruptException:
        pass
