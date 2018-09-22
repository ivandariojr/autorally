#!/usr/bin/env python

import rospy
from autorally_msgs.msg import spawnObstacles

def main():
  pub = rospy.Publisher('/spawn_obstacles', spawnObstacles, queue_size=1)
  rospy.init_node('trivial_obstacle_spawner')
  rate = rospy.Rate(1)
  # while not rospy.is_shutdown():
  for i in range(10):
    print('sent message')
    pub.publish(spawnObstacles(rospy.Header(), False, 25, 3))
    rate.sleep()

  print("hello ros")

if __name__ == '__main__':
  main()