#include <fstream>
#include <iostream>

#include <asd_msg/controllerAxis.h>
#include <asd_msg/controllerButton.h>
#include <cmath>
#include <ros/ros.h>

#include "libnaza/naza_interface_manual.h"
#include "libnaza/pca9685.h"

static naza_interface_manual_c naza;
static PCA9685 pca9685;
static ConfigFile cf("/etc/naza/pwm_config.txt");

// Takes something in range [-1.0, 1.0] and turns it into a positive or negative percentage.
int convertToPercent(float value) {
  return static_cast<int>(std::floor(value * 100.0));
}

int convertToThrottle(float value) {
  return static_cast<int>(std::floor((value + 1) / 2.0 * 100.0));
}

void callback(const asd_msg::controllerAxis::ConstPtr &message) {
  int throttle = convertToThrottle(message->throttle);
  int yaw = convertToPercent(message->yaw);
  int pitch = convertToPercent(message->pitch);
  int roll = convertToPercent(message->roll);

  ROS_INFO("Throttle: %i, yaw: %i, pitch: %i, roll: %i", throttle, yaw, pitch, roll);
  naza.fly_throttle(cf, pca9685, throttle);
  if (yaw < 0)
    naza.fly_turn_left(cf, pca9685, -yaw); // abs of yaw
  else
    naza.fly_turn_right(cf, pca9685, yaw);
  if (pitch < 0)
    naza.fly_forward(cf, pca9685, pitch);
  else
    naza.fly_back(cf, pca9685, -pitch);

  if (roll < 0)
    naza.fly_left(cf, pca9685, -roll);
  else
    naza.fly_right(cf, pca9685, roll);
}

void buttonCallback(const asd_msg::controllerButton::ConstPtr &message) {
  if(message->pressed &&
      message->button == asd_msg::controllerButton::BUTTON_ARM)
    naza.arm_motors(cf, pca9685);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/c2/joystickAxes", 1, &callback);
  ros::Subscriber buttonSub =
      nh.subscribe("/c2/joystickButton", 1, &buttonCallback);

  naza.init_naza(cf, pca9685);
  ros::spin();
}
