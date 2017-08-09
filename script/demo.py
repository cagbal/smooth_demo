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
        # distance between robot and people
        self.distance_to_people = None
        # allowed tolerance of the distance
        self.margin = None

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

                if detection.label == self.user:
                    print people_angle # [0.30 -0.30] is good.

                    # Calculate the difference between people-robot distance and
                    # desired distance
                    diff = detection.pose.pose.position.z - \
                        self.distance_to_people

                    print "Difference {}".format(diff)

                    # check if the robot should make translation
                    if abs(diff) < self.margin:
                        # the people is in margin, just rotate
                        self.sss.move_base_rel("base", [0, 0, -people_angle])
                    elif diff > 0:
                        # the robot is too far away from people, go forward
                        self.sss.move_base_rel(\
                            "base", [diff, 0, -people_angle])
                    else:
                        # the robot is too close to people, go back
                        self.sss.move_base_rel(\
                            "base",[-diff, 0, 0])

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

    # Get the distance and margin values
    try:
        follower.distance_to_people = float(
            raw_input("***\nMinimum distance to people: ")
            )
    except Exception as err:
        print err
        raise

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

    # Subscribe to the face positions
    rospy.Subscriber("/detection_tracker/face_position_array",\
        DetectionArray, follower.face_callback)

    # endless loop
    rospy.spin()


if __name__ == "__main__":
    main()
