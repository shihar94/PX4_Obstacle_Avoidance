#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2 as cv
from std_msgs import msg
from std_msgs.msg import String
from perception_px4.msg import CNN_out
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#imports for dronet processing 
from keras import backend as K
import keras
import os, datetime
import utils
import tensorflow as tf
import support



#loading of the model for ROS
json_model_path ="/home/shihar/mini_research_ws/src/perception_px4/models/model_struct.json"
weights_model_path ="/home/shihar/mini_research_ws/src/perception_px4/models/model_weights.h5"
model = support.load_model(json_model_path,weights_model_path,False)
graph = tf.get_default_graph()

debug = False

class image_converter(object):

  def __init__(self):
    self.pub = rospy.Publisher("/cnn_predictions", CNN_out, queue_size=5)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/iris/usb_cam/image_raw",Image,self.callback)
    self.msg = CNN_out()

  def callback(self,data):
    try:
      global graph
      with graph.as_default():
        #msg = CNN_out()
        self.msg.header.stamp = rospy.Time.now()
        cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
        cv_im = support.image_support(cv_image)
        steer, coll = support.prediction_model(model,cv_im)
        self.msg.steering_angle = steer
        self.msg.collision_prob = coll
        rate = rospy.Rate(20)
        self.pub.publish(self.msg)
        if(debug):
          print(steer," steer")
          print(coll,"coll")
        rate.sleep()

    except CvBridgeError as e:
      print (e)



def main():
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("shutting Down")

if __name__ == '__main__':
  main()

