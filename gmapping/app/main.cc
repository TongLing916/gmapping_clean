#include <glog/logging.h>
#include <ros/ros.h>

#include "slam_gmapping.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging("GMapping");
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "gmapping_node");

  gmapping::SlamGMapping gm;
  gm.StartLiveSlam();

  ros::spin();

  google::ShutdownGoogleLogging();

  return 0;
}