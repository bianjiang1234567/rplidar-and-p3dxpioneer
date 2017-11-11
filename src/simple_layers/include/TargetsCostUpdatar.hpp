
#ifndef TARGETSCOSTUPDATER_H
#define TARGETSCOSTUPDATER_H

#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point32.h>
//#include "PTrackingBridge/TargetEstimations"

using geometry_msgs::Point32;

namespace pedestrian_layer {

class TargetsCostUpdater
{
 public:
  TargetsCostUpdater();
  virtual ~TargetsCostUpdater();
  void init(ros::NodeHandle&, std::string);
  void drawPedestrians(Point32[], Point32[], Point32[], Point32[]);
 private:
  double update_freq;
  int time_steps;
  int counter;
};

}

#endif /* TARGETSCOSTUPDATER_H */
