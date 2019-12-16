#pragma once

// Qt
#include <QThread>

// ROS
#include <asd_msg/objectImage.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>

class RosLink : public QThread {
  Q_OBJECT
public:
  void run() override;
  RosLink(int argc, char **argv);
signals:
  // As raw image data
  void livefeedFrame(const std::vector<unsigned char> frameData, unsigned width,
                     unsigned height);
  void detectorfeedFrame(const std::vector<unsigned char> frameData,
                         unsigned width, unsigned height);

  void objectDetected(const std::vector<unsigned char> pngData,
                      const QString objectName);
  void gearStateChanged(bool isDown);
  void shouldClose(); // Emitted when Ros::ok() returns false, use to close the
                      // window
public slots:
  void setLandingGear(const bool isDown);

private:
  int m_argc;
  char **m_argv;

  bool m_isGearDown;

  image_transport::Subscriber m_livefeedSubscriber;
  image_transport::Subscriber m_detectorfeedSubscriber;
  void livefeedCallback(const sensor_msgs::Image::ConstPtr &image);
  void detectorfeedCallback(const sensor_msgs::Image::ConstPtr &image);

  ros::Subscriber m_detectedObjectSubscriber;

  void detectedObjectCallback(const asd_msg::objectImage::ConstPtr &data);

  ros::Publisher m_gearPublisher;
  ros::Subscriber m_gearStateSubscriber;
  void gearStateCallback(const std_msgs::Bool::ConstPtr &newState);
};
