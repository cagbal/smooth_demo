#!/usr/bin/python

import rospy

from simple_script_server import *

import numpy as np

from cob_perception_msgs.msg import DetectionArray

"""
This file contains demo script for Smooth Workshop @ Fraunhofer IPA, Stuttgart
It asks user to input a user name to follow. If that user in the field of view,
the robot will try to go to that person.

Objective:
     To experience the people perception of Care-O-bot.

Installation:
Connect to the Robot with ssh -XC username@cob4-7-IP
cd ~/git/care-obot
catkin_make && cd src
git clone https://github.com/ipa-rmb/cob_people_perception.git
git clone https://github.com/ipa-rmb/cob_perception_common.git
git clone https://github.com/cagbal/smooth_demo.git
cd .. && source devel/setup.bash
rosdep install --from-path src/ -y -i
catkin_make -DCMAKE_BUILD_TYPE="Release"

Run the script as a ROS node:
>>> rosrun smooth_demo demo.py

Author:
    Cagatay odabasi
    cagatay.odabasi@ipa.fraunhofer.de
"""

class PeopleFollower(object):
    """docstring for PeopleFollower."""
    def __init__(self):
        super(PeopleFollower, self).__init__()
        # simple script server object
        self.sss = simple_script_server()
        # user to follow
        self.user = None
        # subscriber
        self.subs = None

    def follow(self, msg):
        """
        Function that makes the robot follow a specific people.
        """
        # get the number of detections
        no_of_detections = len(msg.detections)

        # Check if there is a detection
        if no_of_detections > 0:
            for detection in msg.detections:
                # get the angle of user according to the center of image
                people_angle = np.arcsin(\
                    detection.pose.pose.position.x/\
                    detection.pose.pose.position.z)

                # Print detections
                print "Detected person: {}".format(detection.label)

                if detection.label == self.user and \
                    np.absolute(people_angle) > 0.15:

                    print people_angle

                    self.subs.unregister()

                    # Rotate to that person
                    self.sss.move_base_rel("base", [0, 0, -people_angle])

                    # Subscribe to the face positions again
                    self.subs= rospy.Subscriber("/detection_tracker/face_position_array",\
                        DetectionArray, self.face_callback)


    def face_callback(self, data):
        """
        Callback for face tracking message
        """
        self.follow(data)

def ask_user(follower):
    """
    Ask user input
    """
    # Get the name of the user that should be followed
    follower.user = raw_input("***\nEnter the name of the user: ")

    # Subscribe to the face positions
    follower.subs = rospy.Subscriber("/detection_tracker/face_position_array",\
            DetectionArray, follower.face_callback)

    return follower

def main():
    """
    Main function
    """

    # init the rosnode
    rospy.init_node('follow_people', anonymous=True)#create node

    # create follower object
    follower = PeopleFollower()

    # ask user to follow
    follower = ask_user(follower)

    # endless loop
    rospy.spin()


if __name__ == "__main__":
    main()
