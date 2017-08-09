#!/usr/bin/env python
import rospy
from simple_script_server import *

def smoothWS_app():
  rospy.init_node('myApp', anonymous=True)
  rospy.spin()

if __name__ == '__main__':
  smoothWS_app()
