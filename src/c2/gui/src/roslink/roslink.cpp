#include "roslink/roslink.hpp"

#include <ros/ros.h>

RosLink::RosLink(int argc, char **argv)
    : m_argc(argc), m_argv(argv), m_isGearDown(true) {}

void RosLink::livefeedCallback(const sensor_msgs::Image::ConstPtr &image) {
  emit livefeedFrame(image->data, image->width, image->height);
}

void RosLink::detectorfeedCallback(const sensor_msgs::Image::ConstPtr &image) {
  emit detectorfeedFrame(image->data, image->width, image->height);
}

void RosLink::detectedObjectCallback(
    const asd_msg::objectImage::ConstPtr &data) {
  emit objectDetected(data->png_data,
                      QString::fromStdString(data->object_name));
}

void RosLink::setLandingGear(const bool isDown) {
  std_msgs::Bool message;
  message.data = isDown;
  m_gearPublisher.publish(message);
}

void RosLink::gearStateCallback(const std_msgs::Bool::ConstPtr &newState) {
  emit gearStateChanged(newState->data);
}

void RosLink::run() {
  ros::init(m_argc, m_argv, "gui");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  m_livefeedSubscriber =
      it.subscribe("/drone/livefeed", 1, &RosLink::livefeedCallback, this);
  m_detectorfeedSubscriber = it.subscribe("/drone/detectorfeed", 1,
                                          &RosLink::detectorfeedCallback, this);
  m_detectedObjectSubscriber = nh.subscribe(
      "/drone/detectedObject", 100, &RosLink::detectedObjectCallback, this);

  m_gearPublisher =
      nh.advertise<std_msgs::Bool>("/drone/landing_gear/set_down", 1);
  m_gearStateSubscriber = nh.subscribe("/drone/landing_gear/is_down", 1,
                                       &RosLink::gearStateCallback, this);

  ros::spin();

  emit shouldClose();
}
