// This code is not stable. It has been rewritten in Python

#include <iostream>
#include <ros/ros.h>
#include <wiringPi.h>
#include <asd_msg/ultrasonic.h>

static volatile long startTimeUsec;
static volatile long endTimeUsec;
static double distanceMeters;
static long travelTimeUsec;
static int trigger = 18;
static int echo = 24;

void init_gpio() {
    wiringPiSetupGpio();
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
}

void recordPulseLength(){
    startTimeUsec = micros();
    while (digitalRead(echo) == HIGH);
    endTimeUsec = micros();
}

double distance(int timeout) {
    delay(10);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    long now = micros();

    while(digitalRead(echo) == LOW && (micros()-now) < timeout){
        recordPulseLength();
    }

    travelTimeUsec = endTimeUsec - startTimeUsec;
    distanceMeters = 100 * ((travelTimeUsec/1000000.0) * 340.29) / 2;

    return distanceMeters;
}

int main(int argc, char **argv){
    init_gpio();

    digitalWrite(trigger, LOW);

    ros::init(argc, argv, "ultrasonic_driver");
    ros::NodeHandle nh;
    ros::Publisher distPublisher = nh.advertise<asd_msg::ultrasonic>("/drone/ultrasonic", 1);
    ros::Rate rate(10);
    while (ros::ok()) {
	    asd_msg::ultrasonic out;
	    out.distanceMeters = distance(30000);
	    distPublisher.publish(out);
	    rate.sleep();
    }

    std::cout << "Distance is " << distance(30000) << " cm.\n";
    return 0;
}
