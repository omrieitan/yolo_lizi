
#define USE_TEENSY_HW_SERIAL

#include "imu.h"
#include <ros.h>
#include <yolo_lizi/lizi_imu.h>


ros::NodeHandle nh;
ric::Imu imu;

yolo_lizi::lizi_imu msg;

ros::Publisher imu_glove("imu_glove", &msg);

unsigned long pre_t;

void setup() {
  nh.initNode();
  nh.advertise(imu_glove);
  Serial.begin(115200);
  imu.init(0);
  if (!imu.isOk()) Serial.println("Error initializing IMU");
  Serial3.begin(19200);
  pre_t = millis();
}

ric::protocol::orientation data;

void loop() {
  unsigned long now = millis();
  if (now - pre_t > 20) {
    if (imu.read(data))
    {
      msg.pitch = data.pitch;
      msg.roll = data.roll;
      imu_glove.publish( &msg );
      nh.spinOnce();
    }
    pre_t=now;
  }
}
