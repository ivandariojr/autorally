#!/usr/bin/env python

import rospy
from autorally_msgs.msg import spawnObstacles, obstacleStatus

def status_cb(msg):
  print(msg.obstacle_names)

def main():
  pub = rospy.Publisher('/spawn_obstacles', spawnObstacles, queue_size=1)
  sub = rospy.Subscriber('/obstacles_status', obstacleStatus, status_cb)
  rospy.init_node('trivial_obstacle_spawner')
  rate = rospy.Rate(1)
  # while not rospy.is_shutdown():
  for i in range(2):
    print('sent message')
    pub.publish(spawnObstacles(rospy.Header(), False, 3, 3, ['construction_cone']*3))
    rate.sleep()

  print("hello ros")

if __name__ == '__main__':
  main()