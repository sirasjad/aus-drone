#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

image_transport::Publisher livefeedPublisher;

#ifdef RASPI
#include <raspicam/raspicam_cv.h>
#endif

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera");

#ifdef RASPI
  raspicam::RaspiCam_Cv capture;
  ROS_INFO("Compiled for Raspberry pi, using pi camera");
  if (!capture.open()) {
    ROS_ERROR("Failed to open pi camera");
    return 1;
  }
#else
  cv::VideoCapture capture;
  std::string videoSource;
  ros::NodeHandle privateNodeHandle("~");
  if (!privateNodeHandle.getParam("video_source", videoSource)) {
    ROS_ERROR("No video_source setting found!");
    return 1;
  }
  ROS_INFO("Using video source: %s", videoSource.c_str());
  capture = cv::VideoCapture(videoSource, cv::CAP_ANY);
#endif

  const int framerate = 30;

  if (!capture.isOpened()) {
    ROS_ERROR("Failed to open video feed");
    return 1;
  }

  sensor_msgs::ImagePtr message;

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  livefeedPublisher = it.advertise("/drone/livefeed", 1);

  cv::Mat frame;
  ros::Rate loopRate(framerate); // limit to framerate of camera feed
  try {
    while (ros::ok()) {
#ifdef RASPI
      capture.grab();
      capture.retrieve(frame);
#else
      if (!capture.read(frame)) {
        ROS_ERROR("Couldn't read capture");
        break;
      } else {
#endif
      if (!frame.empty()) {
        message =
            cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
        livefeedPublisher.publish(message);
      }
#ifndef RASPI
    }
#endif
    ros::spinOnce();
    loopRate.sleep();
  }
}
catch (cv::Exception &ex) {
  ROS_ERROR("Received opencv exception %s. Shutting down...", ex.what());
}

capture.release();
}
