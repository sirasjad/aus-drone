#!/usr/bin/env python3

import rospy
import rospkg
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from asd_msg.msg import objectImage

#ROS init

rospy.init_node('listener')
bridge = CvBridge()
feedPublisher = rospy.Publisher('/drone/rawdetectorfeed', Image, queue_size=1)
detectedObjectPublisher = rospy.Publisher('/drone/detectedObject', objectImage, queue_size=10)

#Object detection init
node_dir = rospkg.RosPack().get_path('object_detector')
net = cv.dnn.readNet(node_dir + "/yolov3-tiny.weights", node_dir + "/yolov3-tiny.cfg")
classes = []
with open(node_dir + "/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

def detect(image):
  blob = cv.dnn.blobFromImage(image, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
  net.setInput(blob)
  outs = net.forward(output_layers)
  class_ids = []
  confidences = []
  boxes = []

  width, height, channels = image.shape

  for out in outs:
    for detection in out:
      scores = detection[5:]
      class_id = np.argmax(scores)
      confidence = scores[class_id]
      if confidence > 0.5:
        # Object detected
        center_x = int(detection[0] * width)
        center_y = int(detection[1] * height)
        w = int(detection[2] * width)
        h = int(detection[3] * height)
        # Rectangle coordinates
        x = int(center_x - w / 2)
        y = int(center_y - h / 2)
        boxes.append([x, y, w, h])
        confidences.append(float(confidence))
        class_ids.append(class_id)

  indexes = cv.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
  detectedObjects = list()
  newImage = image.copy()

  font = cv.FONT_HERSHEY_PLAIN
  for i in range(len(boxes)):
    if i in indexes:
      x, y, w, h = boxes[i]
      label = str(classes[class_ids[i]])
      color = colors[i]
      cv.rectangle(newImage, (x, y), (x + w, y + h), color, 2)
      cv.putText(newImage, label, (x, y + 30), font, 3, color, 3)

      objectMessage = objectImage()
      cropped = image[y:y+h, x:x+w]
      imageheight, imagewidth, channels = cropped.shape
      if imageheight > 0 and imagewidth > 0:
        png = cv.imencode('.png', cropped)
        objectMessage.png_data = png[1].tostring()
        objectMessage.object_name = label
        detectedObjectPublisher.publish(objectMessage)

  return newImage, detectedObjects

def callback(data):
  image = bridge.imgmsg_to_cv2(data, 'bgr8')
  blob = cv.dnn.blobFromImage(image, 1.0, (609, 609), (0, 0, 0), True, crop=False)
  boxes, detectedObjects = detect(image)
  bridged = bridge.cv2_to_imgmsg(boxes, 'bgr8')
  feedPublisher.publish(bridged)

sub = rospy.Subscriber('/drone/rawlivefeed', Image, callback, queue_size=1)

if __name__ == '__main__':
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
