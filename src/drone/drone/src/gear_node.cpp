#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <wiringPi.h>

#define SERVO_PIN 18
#define DEG_UP 180
#define DEG_DOWN 90

void init_gpio() {
  wiringPiSetupGpio();
  pinMode(SERVO_PIN, PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);
  pwmSetRange(2000);
  pwmSetClock(192);
}

static bool isDown;
ros::Publisher gearStatePublisher;

void setFeetDown(bool down) {
  if (down && !isDown) {
    for (int i = DEG_DOWN; i < DEG_UP; i++) {
      pwmWrite(SERVO_PIN, i);
      delay(5);
    }
  } else if (!down && isDown) {
    for (int i = DEG_UP; i > DEG_DOWN; i--) {
      pwmWrite(SERVO_PIN, i);
      delay(5);
    }
  }
  std_msgs::Bool message;
  message.data = down;
  gearStatePublisher.publish(message);
  isDown = down;
}

void setDownCallback(const std_msgs::Bool::ConstPtr &msg) {
  setFeetDown(msg->data);
}

int main(int argc, char **argv) {
  init_gpio();
  ros::init(argc, argv, "gear_node");
  ros::NodeHandle localNh("~");
  ros::NodeHandle nh;

  setFeetDown(true);

  ros::Subscriber sub =
      nh.subscribe("/drone/landing_gear/set_down", 1, setDownCallback);
  gearStatePublisher =
      nh.advertise<std_msgs::Bool>("/drone/landing_gear/is_down", 1);
  ros::spin();

  return 0;
}
