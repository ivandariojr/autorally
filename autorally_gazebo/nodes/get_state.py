#!/usr/bin/env python

import rospy
import numpy as np
from matplotlib import pyplot as plt

# import message types
from sensor_msgs.msg import Image
from autorally_msgs.msg import wheelSpeeds
from autorally_msgs.msg import chassisState

class VehicleState(object):
  """
  An object of class Vehicle is a node which receives the camera feed and state information
  from the AutoRally.
  """

  def __init__(self):
    rospy.init_node('camera_listener', anonymous=True)
    plt.ion()

    # get namespace
    try:
      self._vehicle_prefix = rospy.get_param("~vehicle_prefix")
      if self._vehicle_prefix != "":
        self._vehicle_prefix = "/" + self._vehicle_prefix
    except:
      rospy.logwarn("The specified namespace value is invalid. "
                    "The default timeout value will be used instead.")
      self._vehicle_prefix = ""

    # create subscribers
    self.left_camera_sub = rospy.Subscriber(self._vehicle_prefix + '/left_camera/image_raw',
                                            Image, self.left_camera_cb)
    self.right_camera_sub = rospy.Subscriber(self._vehicle_prefix + '/right_camera/image_raw',
                                             Image, self.right_camera_cb)
    self.wheel_speeds_sub = rospy.Subscriber(self._vehicle_prefix + '/wheelSpeeds',
                                             wheelSpeeds, self.wheel_speeds_cb)
    self.chassis_state_sub = rospy.Subscriber(self._vehicle_prefix + '/chassisState',
                                              chassisState, self.chassis_state_cb)

  def left_camera_cb(self, data):
    width = data.width
    height = data.height
    self._left_stamp = {'secs':data.header.stamp.secs, 'nsecs':data.header.stamp.nsecs}
    self._left_image = np.fromstring(data.data, dtype='uint8').reshape((height, width, 3))

  def right_camera_cb(self, data):
    width = data.width
    height = data.height
    self._right_stamp = {'secs':data.header.stamp.secs, 'nsecs':data.header.stamp.nsecs}
    self._right_image = np.fromstring(data.data, dtype='uint8').reshape((height, width, 3))

  def wheel_speeds_cb(self, data):
    self._lf_speed = data.lfSpeed
    self._rf_speed = data.rfSpeed
    self._lb_speed = data.lbSpeed
    self._rb_speed = data.rbSpeed
    self._wheel_speed_stamp = {'secs':data.header.stamp.secs, 'nsecs':data.header.stamp.nsecs}

  def chassis_state_cb(self, data):
    self._steering = data.steering
    self._throttle = data.throttle
    self._front_brake = data.frontBrake

  def display(self, image):
    plt.imshow(image)
    plt.draw()
    plt.pause(0.0000001)

  def spin(self):
    while not rospy.is_shutdown():
      try:
        self.display(self._left_image)
      except:
        continue

if __name__ == '__main__':
  vehicle = VehicleState()
  vehicle.spin()