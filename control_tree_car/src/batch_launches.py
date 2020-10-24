#!/usr/bin/env python

import subprocess
import time
from collections import namedtuple
import roslaunch
import sys
import rospy

def kill_safe():
  kill()
  time.sleep(1)
  kill()
  time.sleep(1)
  kill()

def kill():
  ps = subprocess.Popen(["ps", "-xa"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
  output, errors = ps.communicate()

  for line in output.splitlines():
    if "gzserver" in line or "gnuplot" in line or "rosout" in line or "ros/kinetic/bin" in line or "pedestrian_tree_qp_node" in line or "pedestrian_popping" in line or "trajectory_control" in line:
      print(line)
      potential_pids = line.split(' ')

      for potential_pid in potential_pids[0:3]:
        try:
          pid = int(potential_pid)
          ps = subprocess.Popen(["kill", "-9", potential_pid])
          print("KILLED :{}".format(potential_pid))
          break
        except:
          print("COULD NOT KILL!! :{}".format(potential_pid))
          pass


TestConfig = namedtuple('TestConfig', ['n_pedestrians', 'n_branches', 'crossing_probability'])

tests = list()

kill_safe()
#
tests.append(TestConfig(n_pedestrians=1, n_branches=2, crossing_probability=0.05))
tests.append(TestConfig(n_pedestrians=1, n_branches=1, crossing_probability=0.05))

tests.append(TestConfig(n_pedestrians=1, n_branches=2, crossing_probability=0.25))
tests.append(TestConfig(n_pedestrians=1, n_branches=1, crossing_probability=0.25))

#
tests.append(TestConfig(n_pedestrians=4, n_branches=5, crossing_probability=0.01))
tests.append(TestConfig(n_pedestrians=4, n_branches=2, crossing_probability=0.01))
tests.append(TestConfig(n_pedestrians=4, n_branches=1, crossing_probability=0.01))
#
tests.append(TestConfig(n_pedestrians=4, n_branches=5, crossing_probability=0.05))
tests.append(TestConfig(n_pedestrians=4, n_branches=2, crossing_probability=0.05))
tests.append(TestConfig(n_pedestrians=4, n_branches=1, crossing_probability=0.05))
#
tests.append(TestConfig(n_pedestrians=4, n_branches=5, crossing_probability=0.25))
tests.append(TestConfig(n_pedestrians=4, n_branches=2, crossing_probability=0.25))
tests.append(TestConfig(n_pedestrians=4, n_branches=1, crossing_probability=0.25))

for test in tests:
  # ros
  sys.argv.append("n_pedestrians:={}".format(test.n_pedestrians))
  sys.argv.append("n_branches:={}".format(test.n_branches))
  sys.argv.append("p_crossing:={}".format(test.crossing_probability))

  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/camille/git/catkin_ws/src/rss_2020/control_tree_car/launch/pedestrian.launch"])
  launch.start()
  rospy.loginfo("started")

  # gazebo
  print("/home/camille/git/catkin_ws/src/rss_2020/lgp_car_gazebo_plugin/world/pedestrian_{}.world".format(test.n_pedestrians))
  gazebo = subprocess.Popen(["gzserver", "/home/camille/git/catkin_ws/src/rss_2020/lgp_car_gazebo_plugin/world/pedestrian_{}.world".format(test.n_pedestrians)])

  time.sleep(1800)

  launch.shutdown()
  kill_safe()


#  #ros = subprocess.Popen(["roslaunch", "control_tree_car", "pedestrian.launch", "n_pedestrians:={}".format(test.n_pedestrians), "n_branches:={}".format(test.n_branches), "p_crossing:={}".format(test.crossing_probability)])
