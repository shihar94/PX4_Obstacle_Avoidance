#This file is to load the required models and weights of the network 
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

K.set_learning_phase(0)
os.environ['TF_CPP_MIN_LOG_LEVEL'] ='2'

def load_model(json_model_path, weights_model_path,debug=False):

    model = utils.jsonToModel(json_model_path)
    model.load_weights(weights_model_path)
    print("Loaded model from {}".format(weights_model_path))
    model.compile(loss='mse', optimizer='sgd')
    if(debug):
        print(model.summary())
    return model

def prediction_model(model,cv_im):
    outs = model.predict_on_batch(cv_im[None])
    steer, coll = outs[0][0], outs[1][0]

    return steer,coll

def image_support(cv_image):
    cv_im = cv_image
    cv_im = cv.cvtColor(cv_im,cv.COLOR_BGR2GRAY)
    cv_im = cv.equalizeHist(cv_im)
    cv_im = utils.callback_img(cv_im,[200,200])
    return cv_im

