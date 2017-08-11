# smooth_demo

A template for hands-on workshops with either Care-O-bot 4 or rob@work. It is
an example for how to subscribe to a topic, how to use script server and how to
use cob_people_detection repo.

## Installation

- Connect to the robot via ssh
- cd && mkdir -p catkin_ws/src
- catkin_make && cd src
- git clone https://github.com/ipa-rmb/cob_people_perception.git
- git clone https://github.com/ipa-rmb/cob_perception_common.git
- git clone https://github.com/cagbal/smooth_demo.git
- cd .. && source devel/setup.bash
- rosdep install --from-path src/ -y -i
- catkin_make -DCMAKE_BUILD_TYPE="Release"

## Running
rosrun smooth_demo demo.py
