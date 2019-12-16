#include "joystick_mapper.hpp"
#include <asd_msg/controllerAxis.h>
#include <asd_msg/controllerButton.h>
#include <ros/ros.h>

#include <climits>
#include <stdexcept>

// Linux-includes for joystick driving
#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

struct AxisState {
  short x, y;
};

static AxisState axisStates[6] = {};

bool read_event(int fd, struct js_event *event) {
  ssize_t bytes;

  bytes = read(fd, event, sizeof(*event));

  if (bytes != sizeof(*event))
    return false;

  return true;
}

size_t get_axis_state(struct js_event *event, AxisState *state) {
  size_t axis = event->number;

  if (event->number % 2 == 0)
    state[axis].x = event->value;
  else {
    state[axis].y = event->value;
  }

  return axis;
}

static ros::Publisher axisPublisher;
static ros::Publisher buttonPublisher;

void publish_axes() {
  asd_msg::controllerAxis out;
  short throttle, yaw, pitch, roll;
  throttle = axisStates[AXIS_THROTTLE].y;
  yaw = axisStates[AXIS_YAW].x;
  pitch = axisStates[AXIS_ROLL].x;
  roll = axisStates[AXIS_PITCH].y;

  out.throttle =
      AXIS_THROTTLE_DIRECTION * static_cast<float>(throttle) / SHRT_MAX;
  out.yaw = AXIS_YAW_RIGHT_DIRECTION * static_cast<float>(yaw) / SHRT_MAX;
  out.pitch = AXIS_PITCH_UP_DIRECTION * static_cast<float>(pitch) / SHRT_MAX;
  out.roll = AXIS_ROLL_RIGHT_DIRECTION * static_cast<float>(roll) / SHRT_MAX;

  axisPublisher.publish(out);
}

void publish_button_event(unsigned char button, short value) {
  asd_msg::controllerButton out;
  switch (button) {
  case CONTROLLER_BUTTON_ARM:
    out.button = asd_msg::controllerButton::BUTTON_ARM;
    out.pressed = static_cast<bool>(value);
    break;
  default:
    return;
  }

  buttonPublisher.publish(out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  axisPublisher = nh.advertise<asd_msg::controllerAxis>("/c2/joystickAxes", 1);
  buttonPublisher =
      nh.advertise<asd_msg::controllerButton>("/c2/joystickButton", 1);

  const char *device = JS_DEVICE;
  ROS_INFO("Using joystick device %s", device);

  int js = open(device, O_RDONLY);
  if (js == -1) {
    ROS_ERROR("Could not open joystick: %s", strerror(errno));
    return 1;
  }

  memset(&axisStates, 0, 6 * sizeof(AxisState));

  ros::Rate rate(60);

  while (ros::ok()) {
    struct js_event event;
    while (read_event(js, &event)) {
      switch (event.type) {
      case JS_EVENT_BUTTON:
        publish_button_event(event.number, event.value);
        break;
      case JS_EVENT_AXIS:
        get_axis_state(&event, axisStates);
        ;
        publish_axes();
      }
    }
    rate.sleep();
  }
}
