#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <message_filters/subscriber.h>
#include <nav_msgs/GetMap.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "grid_fast_slam/grid_slam_processor.h"
#include "sensor/odometry_sensor.h"
#include "sensor/range_sensor.h"
#include "type/oriented_point.h"

namespace gmapping {

class SlamGMapping {
 public:
  enum Status { kUnknown = -1, kFree = 0, kOccupied = 100 };

 public:
  SlamGMapping();

  void StartLiveSlam();

 private:
  void Initialize();

  bool InitializeMapper(const sensor_msgs::LaserScan& scan);

  bool MapCallBack(nav_msgs::GetMap::Request& req,
                   nav_msgs::GetMap::Response& res);

  void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

  void PublishLoop(const double transform_publish_period);

  void PublishTransform();

  bool AddScan(const sensor_msgs::LaserScan& scan,
               OrientedPoint2d* const odom_pose);

  /**
   * @brief Update map using the particle with max weight
   * @details This function is just for visualization.
   */
  void UpdateMap(const sensor_msgs::LaserScan& scan);

  double ComputePoseEntropy();

  bool GetOdomPose(const ros::Time& t, OrientedPoint2d* const pose);

  int GetMapIdx(const int s, const int x, const int y) { return s * y + x; }

 private:
  ros::NodeHandle private_nh_;

  tf::Transform map_2_odom_;
  std::mutex map_2_odom_mtx_;
  int laser_cnt_;

  unsigned long seed_;

  bool got_first_scan_;
  bool got_map_;
  std::mutex map_mtx_;
  nav_msgs::GetMap::Response map_;

  std::shared_ptr<GridSlamProcessor> grid_slam_processor_;
  std::shared_ptr<RangeSensor> range_sensor_;
  std::shared_ptr<OdometrySensor> odometry_sensor_;

  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<std::thread> th_transfrom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>
      scan_filter_sub_;
  std::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;

  ros::NodeHandle nh_;
  ros::Publisher entropy_publisher_;
  ros::Publisher map_publisher_;
  ros::Publisher map_metadata_publisher_;
  ros::ServiceServer dynamic_map_server_;
  tf::TransformListener tf_;

  /**
   * @brief The angles in the laser, going from -x to x
   * @details Adjustment is made to get the laser between symmetrical bounds as
   * that's what gmapping expects.
   */
  std::vector<double> laser_angles_;

  // Parameters used by ROS wrapper
  int throttle_scans_;
  std::string base_frame_;
  std::string laser_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  double transform_publish_period_;
  double tf_delay_;
  ros::Duration map_update_interval_;

  // Parameters used by GMapping
  double max_range_;      // Laser's max range
  double max_urange_;     // Laser's max usable range
  double minimum_score_;  // Minimum score for considering matching outcome good
  double sigma_;          // Variance for computing score
  int kernel_size_;       // Size of searching window when computing score
  double lstep_;          // Linear step size for optimization
  double astep_;          // Angular step size for optimization
  int iterations_;        // Number of optimization iterations
  double lsigma_;         // Variance for computing likelihood
  double ogain_;          // Smoothing factor for the likelihood
  int lskip_;             // Number of beams we skip when computing likelihood
  double srr_;            // Linear error's variance due to linear motion
  double srt_;            // Angular error's variance due to linear motion
  double str_;            // Linear error's variance due to angular motion
  double stt_;            // Angular error's variance due to angular motion
  double linear_update_;
  double angular_update_;
  double temporal_update_;
  double resample_threshold_;
  int num_particles_;
  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;

  /** Map resolution */
  double delta_;

  double occ_thresh_;
  double ll_sample_range_;  // Translational sampling range
  double ll_sample_step_;   // Translational sampling step
  double la_sample_range_;  // Angular sampling range
  double la_sample_step_;   // Angular sampling step

  size_t laser_beam_cnt_;
  bool do_reverse_range_;

  // The pose in the original laser frame, of the corresponding centered laser
  // with z facing up
  tf::Stamped<tf::Pose> centered_laser_pose_;
};

}  // namespace gmapping
