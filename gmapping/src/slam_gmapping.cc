#include "slam_gmapping.h"

#include <map>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>

#include "grid/hierarchical_array_2d.h"
#include "scan_matcher/scan_matcher.h"
#include "sensor/range_reading.h"
#include "utility/random_helper.h"

namespace gmapping {

SlamGMapping::SlamGMapping()
    : private_nh_("~"),
      map_2_odom_(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
                                tf::Point(0, 0, 0))),
      laser_cnt_(0),
      seed_(time(nullptr)),
      got_first_scan_(false),
      got_map_(false),
      grid_slam_processor_(std::make_shared<GridSlamProcessor>()),
      tf_broadcaster_(std::make_shared<tf::TransformBroadcaster>()) {
  ROS_ASSERT(grid_slam_processor_);
  ROS_ASSERT(tf_broadcaster_);

  Initialize();
}

void SlamGMapping::Initialize() {
  // Parameters used by ROS wrapper
  {
    private_nh_.param<int>("throttle_scans", throttle_scans_, 1);

    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");

    private_nh_.param<double>("transform_publish_period",
                              transform_publish_period_, 0.05);
    private_nh_.param<double>("tf_delay", tf_delay_, transform_publish_period_);

    double tmp = 5.0;
    private_nh_.param<double>("map_update_interval", tmp, 5.0);
    map_update_interval_.fromSec(tmp);
  }

  // Parameters used by GMapping itself
  {
    max_range_ = 0.0;  // preliminary default, will be set in InitMapper()
    max_urange_ = 0.0;

    private_nh_.param<double>("minimumScore", minimum_score_, 0.);
    private_nh_.param<double>("sigma", sigma_, 0.05);
    private_nh_.param<int>("kernelSize", kernel_size_, 1);
    private_nh_.param<double>("lstep", lstep_, 0.05);
    private_nh_.param<double>("astep", astep_, 0.05);
    private_nh_.param<int>("iterations", iterations_, 5);
    private_nh_.param<double>("lsigma", lsigma_, 0.075);
    private_nh_.param<double>("ogain", ogain_, 3.0);
    private_nh_.param<int>("lskip", lskip_, 0);

    private_nh_.param<double>("srr", srr_, 0.1);
    private_nh_.param<double>("srt", srt_, 0.2);
    private_nh_.param<double>("str", str_, 0.1);
    private_nh_.param<double>("stt", stt_, 0.2);

    private_nh_.param<double>("linearUpdate", linear_update_, 1.0);
    private_nh_.param<double>("angularUpdate", angular_update_, 0.5);
    private_nh_.param<double>("temporalUpdate", temporal_update_, -1.0);

    private_nh_.param<double>("resampleThreshold", resample_threshold_, 0.5);
    private_nh_.param<int>("particles", num_particles_, 30);
    private_nh_.param<double>("xmin", xmin_, -100.0);
    private_nh_.param<double>("ymin", ymin_, -100.0);
    private_nh_.param<double>("xmax", xmax_, 100.0);
    private_nh_.param<double>("ymax", ymax_, 100.0);
    private_nh_.param<double>("delta", delta_, 0.05);
    private_nh_.param<double>("occ_thresh", occ_thresh_, 0.25);
    private_nh_.param<double>("llsamplerange", ll_sample_range_, 0.01);
    private_nh_.param<double>("llsamplestep", ll_sample_step_, 0.01);
    private_nh_.param<double>("lasamplerange", la_sample_range_, 0.005);
    private_nh_.param<double>("lasamplestep", la_sample_step_, 0.005);
  }
}

void SlamGMapping::StartLiveSlam() {
  entropy_publisher_ =
      private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);

  map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_metadata_publisher_ =
      nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

  dynamic_map_server_ =
      nh_.advertiseService("dynamic_map", &SlamGMapping::MapCallBack, this);

  // Subscribe laser topic and wait until the tranform to [odom_frame_] is ready
  {
    scan_filter_sub_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(
            nh_, "scan", 5);
    scan_filter_ = std::make_shared<tf::MessageFilter<sensor_msgs::LaserScan>>(
        *scan_filter_sub_, tf_, odom_frame_, 5);

    scan_filter_->registerCallback(
        [&](const sensor_msgs::LaserScan::ConstPtr& scan) {
          this->LaserCallBack(scan);
        });
  }

  th_transfrom_ = std::make_shared<std::thread>(
      &SlamGMapping::PublishLoop, this, transform_publish_period_);
}

bool SlamGMapping::MapCallBack(nav_msgs::GetMap::Request& req,
                               nav_msgs::GetMap::Response& res) {
  std::unique_lock<std::mutex> lck(map_mtx_);
  if (got_map_ && map_.map.info.width && map_.map.info.height) {
    res = map_;
    return true;
  }

  return false;
}

void SlamGMapping::LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_ASSERT(scan);
  ROS_ASSERT(throttle_scans_ != 0);

  ++laser_cnt_;
  if (laser_cnt_ % throttle_scans_ != 0) {
    return;
  }

  static ros::Time last_map_update(0, 0);

  // Once we got the first scan, we will start to initialize
  if (!got_first_scan_) {
    if (!InitializeMapper(*scan)) {
      return;
    }
    got_first_scan_ = true;
  }

  OrientedPoint2d odom_pose;
  if (!AddScan(*scan, &odom_pose)) {
    ROS_WARN("CANNOT PROCESS SCAN");
    return;
  }

  const auto best_pose = grid_slam_processor_->GetBestParticle().pose;

  ROS_DEBUG("Scan is being processed...");
  ROS_DEBUG("Best particle pose: %.3f, %.3f, %.3f", best_pose.x, best_pose.y,
            best_pose.theta);
  ROS_DEBUG("Odometry pose: %.3f, %.3f, %.3f", odom_pose.x, odom_pose.y,
            odom_pose.theta);
  ROS_DEBUG("Pose difference: %.3f, %.3f, %.3f", best_pose.x - odom_pose.x,
            best_pose.y - odom_pose.y, best_pose.theta - odom_pose.theta);

  const auto laser_2_map =
      tf::Transform(tf::createQuaternionFromRPY(0., 0., best_pose.theta),
                    tf::Vector3(best_pose.x, best_pose.y, 0.))
          .inverse();
  const auto odom_2_laser =
      tf::Transform(tf::createQuaternionFromRPY(0., 0., odom_pose.theta),
                    tf::Vector3(odom_pose.x, odom_pose.y, 0.));
  {
    std::unique_lock<std::mutex> lck(map_2_odom_mtx_);
    map_2_odom_ = (odom_2_laser * laser_2_map).inverse();
  }

  // Update map if
  // 1) We do not have any map yet
  // 2) Certain time has elapsed
  if (!got_map_ ||
      scan->header.stamp - last_map_update > map_update_interval_) {
    UpdateMap(*scan);
    last_map_update = scan->header.stamp;
  }
}

void SlamGMapping::PublishLoop(const double transform_publish_period) {
  if (transform_publish_period_ < 1e-6) {
    return;
  }

  ros::Rate r(1. / transform_publish_period_);
  while (ros::ok()) {
    PublishTransform();
    r.sleep();
  }
}

void SlamGMapping::PublishTransform() {
  std::unique_lock<std::mutex> lck(map_2_odom_mtx_);
  const ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tf_broadcaster_->sendTransform(tf::StampedTransform(
      map_2_odom_, tf_expiration, map_frame_, odom_frame_));
}

bool SlamGMapping::InitializeMapper(const sensor_msgs::LaserScan& scan) {
  laser_frame_ = scan.header.frame_id;

  // Get the laser's pose relative to base
  tf::Stamped<tf::Pose> laser_pose;
  {
    tf::Stamped<tf::Pose> identity;
    identity.setIdentity();
    identity.frame_id_ = laser_frame_;
    identity.stamp_ = scan.header.stamp;

    try {
      tf_.transformPose(base_frame_, identity, laser_pose);
    } catch (const tf::TransformException& e) {
      ROS_ERROR("Failed to compute laser pose, aborting initialization (%s)",
                e.what());
      return false;
    }
  }

  // Check whether laser is horizontally mounted
  // Process:
  // 1) Create a point 1m above the laser.
  // 2) Transform it into the laser frame.
  // 3) If laser is horizontal, then the transformed point's z is still 1
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp, base_frame_);
  {
    try {
      tf_.transformPoint(laser_frame_, up, up);
      ROS_DEBUG("Z-axis in sensor frame: %.3f", up.z());
    } catch (const tf::TransformException& e) {
      ROS_ERROR("Unable to determine orientation of laser: %s", e.what());
      return false;
    }

    if (fabs(fabs(up.z()) - 1.) > 1e-3) {
      ROS_WARN(
          "Laser has to be mounted planarly! Z-coordinate has to be 1 or -1, "
          "but gave: %.5f",
          up.z());
      return false;
    }
  }

  laser_beam_cnt_ = scan.ranges.size();

  // Check whether laser is mounted reversely.
  {
    const double angle_center = (scan.angle_min + scan.angle_max) / 2.;

    if (up.z() > 0.) {
      ROS_INFO("Laser is mounted upwards.");
      do_reverse_range_ = scan.angle_min > scan.angle_max;
      centered_laser_pose_ = tf::Stamped<tf::Pose>(
          tf::Transform(tf::createQuaternionFromRPY(0, 0, angle_center),
                        tf::Vector3(0, 0, 0)),
          ros::Time::now(), laser_frame_);
    } else {
      ROS_INFO("Laser is mounted upside down.");
      do_reverse_range_ = scan.angle_min < scan.angle_max;
      centered_laser_pose_ = tf::Stamped<tf::Pose>(
          tf::Transform(tf::createQuaternionFromRPY(M_PI, 0, -angle_center),
                        tf::Vector3(0, 0, 0)),
          ros::Time::now(), laser_frame_);
    }
  }

  // Compute the laser angles from -x to x, basically symmetric and in
  // increasing order
  {
    laser_angles_.resize(scan.ranges.size());

    double theta = -fabs(scan.angle_min - scan.angle_max) / 2.;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      laser_angles_.at(i) = theta;
      theta += fabs(scan.angle_increment);
    }

    ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f",
              scan.angle_min, scan.angle_max, scan.angle_increment);
    ROS_DEBUG(
        "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f "
        "inc: %.3f",
        laser_angles_.front(), laser_angles_.back(),
        fabs(scan.angle_increment));
  }

  // Set max_range_ and max_urange_ here so we can set a reasonable default
  private_nh_.param<double>("maxRange", max_range_, scan.range_max - 0.01);
  private_nh_.param<double>("maxUrange", max_urange_, max_range_);

  // Initialize a range sensor
  range_sensor_ = std::make_shared<RangeSensor>(
      "FLASER", laser_beam_cnt_, fabs(scan.angle_increment),
      OrientedPoint2d(0, 0, 0), 0., max_range_);

  // Initialize a sensor map for slam processor
  SensorMap names_2_sensors;
  names_2_sensors.emplace(range_sensor_->GetName(), range_sensor_);
  grid_slam_processor_->SetSensorMap(names_2_sensors);

  // Initialize an odometry
  odometry_sensor_ = std::make_shared<OdometrySensor>(odom_frame_);

  // Get initial pose of odometry
  OrientedPoint2d initial_pose;
  if (!GetOdomPose(scan.header.stamp, &initial_pose)) {
    ROS_WARN(
        "Unable to determine inital pose of laser! Starting point will be set "
        "to zero.");
    initial_pose = OrientedPoint2d(0.0, 0.0, 0.0);
  }

  // Configure gmapping algorithm
  grid_slam_processor_->SetMatchingParameters(
      max_urange_, max_range_, sigma_, kernel_size_, lstep_, astep_,
      iterations_, lsigma_, lskip_, ogain_);

  grid_slam_processor_->SetMotionModelParameters(srr_, srt_, str_, stt_);
  grid_slam_processor_->SetUpdateDistances(linear_update_, angular_update_,
                                           resample_threshold_);
  grid_slam_processor_->SetUpdatePeriod(temporal_update_);
  grid_slam_processor_->SetGenerateMap(false);

  grid_slam_processor_->Initialize(xmin_, ymin_, xmax_, ymax_, delta_,
                                   num_particles_, initial_pose);

  grid_slam_processor_->SetTranslationalSamplingRange(ll_sample_range_);
  grid_slam_processor_->SetTranslationalSamplingStep(ll_sample_step_);

  // TODO: Check these calls. In the gmapping gui, they use llsamplestep and
  // llsamplerange intead of lasamplestep and lasamplerange. It was probably a
  // typo, but who knows.
  grid_slam_processor_->SetAngularSamplingRange(la_sample_range_);
  grid_slam_processor_->SetAngularSamplingStep(la_sample_step_);

  grid_slam_processor_->SetMinimumScore(minimum_score_);

  RandomHelper::SampleGaussian(1, seed_);

  ROS_INFO("Initialization completed!");

  return true;
}

bool SlamGMapping::AddScan(const sensor_msgs::LaserScan& scan,
                           OrientedPoint2d* const odom_pose) {
  ROS_ASSERT(odom_pose);

  // Obtain the corresponding odometry pose
  if (!GetOdomPose(scan.header.stamp, odom_pose)) {
    return false;
  }

  if (scan.ranges.size() != laser_beam_cnt_) {
    return false;
  }

  // If the angle increment is negative, we have to invert the order of the
  // readings.
  const size_t num_beams = scan.ranges.size();
  std::vector<double> ranges(num_beams);
  if (do_reverse_range_) {
    ROS_DEBUG("Invert scan ranges' order!");

    for (size_t i = 0; i < num_beams; ++i) {
      // Filter out short readings
      if (scan.ranges.at(num_beams - i - 1) < scan.range_min) {
        ranges.at(i) = static_cast<double>(scan.range_max);
      } else {
        ranges.at(i) = static_cast<double>(scan.ranges.at(num_beams - i - 1));
      }
    }
  } else {
    for (size_t i = 0; i < num_beams; ++i) {
      // Filter out short readings
      if (scan.ranges.at(i) < scan.range_min) {
        ranges.at(i) = static_cast<double>(scan.range_max);
      } else {
        ranges.at(i) = static_cast<double>(scan.ranges.at(i));
      }
    }
  }

  RangeReading reading(range_sensor_, ranges, scan.header.stamp.toSec());
  reading.SetPose(*odom_pose);

  return grid_slam_processor_->ProcessScan(reading);
}

void SlamGMapping::UpdateMap(const sensor_msgs::LaserScan& scan) {
  ROS_INFO("Start to update map!");

  std::unique_lock<std::mutex> lck(map_mtx_);

  // Publish the entropy of the pose
  std_msgs::Float64 entropy;
  entropy.data = ComputePoseEntropy();
  if (entropy.data > 0.) {
    entropy_publisher_.publish(entropy);
  }

  // If no map exists, initialize one
  if (!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.;
    map_.map.info.origin.position.y = 0.;
    map_.map.info.origin.position.z = 0.;
    map_.map.info.origin.orientation.x = 0.;
    map_.map.info.origin.orientation.y = 0.;
    map_.map.info.origin.orientation.z = 0.;
    map_.map.info.origin.orientation.w = 1.;
  }

  // Initialize a scan scan_matcher
  ScanMatcher scan_matcher;
  scan_matcher.SetLaserParamters(scan.ranges.size(), laser_angles_,
                                 range_sensor_->GetPose());
  scan_matcher.SetMaxRange(max_range_);
  scan_matcher.SetUsableRange(max_urange_);
  scan_matcher.SetGenerateMap(true);

  // Map's center
  Point2d center;
  center.x = (xmin_ + xmax_) / 2.;
  center.y = (ymin_ + ymax_) / 2.;

  // Initialize a scan matcher map for map generation
  ScanMatcherMap scan_matcher_map(center, xmin_, ymin_, xmax_, ymax_, delta_);

  // Update map with the particle with the max weight
  // Note: Traverse the whole trajectory of the particle, then compute a map
  // according to the stored information
  const auto best_particle = grid_slam_processor_->GetBestParticle();
  ROS_DEBUG("Trajectory tree:");
  NodePtr node = best_particle.node;
  while (node) {
    ROS_DEBUG(" %.3f %.3f %.3f", node->pose.x, node->pose.y, node->pose.theta);

    if (!node->reading) {
      ROS_DEBUG("Reading is NULL");
      node = node->parent;
      continue;
    }

    scan_matcher.RegisterScan(node->pose, node->reading->GetRanges(),
                              &scan_matcher_map);

    // TODO: Endless loop
    node = node->parent;
  }

  // The map may have expanded, so resize ROS message as well
  const int map_size_x = scan_matcher_map.GetMapSizeX();
  const int map_size_y = scan_matcher_map.GetMapSizeY();
  if (map_.map.info.width != static_cast<uint32_t>(map_size_x) ||
      map_.map.info.height != static_cast<uint32_t>(map_size_y)) {
    // NOTE: The results of ScanMatcherMap::GetSize() are different from the
    // parameters given to the constructor, so we must obtain the bounding box
    // in a different way
    const auto wmin = scan_matcher_map.Map2World(Point2i(0, 0));
    const auto wmax =
        scan_matcher_map.Map2World(Point2i(map_size_x, map_size_y));
    xmin_ = wmin.x;
    ymin_ = wmin.y;
    xmax_ = wmax.x;
    ymax_ = wmax.y;

    ROS_DEBUG("Map size is now %dx%d pixels (%.3f,%.3f)-(%.3f,%.3f)",
              map_size_x, map_size_y, xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = map_size_x;
    map_.map.info.height = map_size_y;
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("Map origin: (%.3f, %.3f)", map_.map.info.origin.position.x,
              map_.map.info.origin.position.y);
  }

  // According to the map information, ensure every point's statsu (occupied,
  // free, unknown)
  for (int x = 0; x < map_size_x; ++x) {
    for (int y = 0; y < map_size_y; ++y) {
      const Point2i p(x, y);
      const double occ_prob = scan_matcher_map.GetCell(p)->GetOccupancyProb();
      ROS_ASSERT(occ_prob <= 1.);

      if (occ_prob < 0.) {
        map_.map.data.at(GetMapIdx(map_.map.info.width, x, y)) = kUnknown;
      } else if (occ_prob > occ_thresh_) {
        map_.map.data.at(GetMapIdx(map_.map.info.width, x, y)) = kOccupied;
      } else {
        map_.map.data.at(GetMapIdx(map_.map.info.width, x, y)) = kFree;
      }
    }
  }

  got_map_ = true;
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve(map_frame_);

  map_publisher_.publish(map_.map);
  map_metadata_publisher_.publish(map_.map.info);

  ROS_INFO("Finished updating map!");
}

double SlamGMapping::ComputePoseEntropy() {
  double sum_weight = 0.;
  const auto& particles = grid_slam_processor_->GetParticles();
  for (auto it = particles.cbegin(); it != particles.cend(); ++it) {
    sum_weight += it->weight;
  }

  ROS_ASSERT(sum_weight > 1e-6);

  const double sum_weight_inv = 1. / sum_weight;

  double entropy = 0.;
  for (auto it = particles.cbegin(); it != particles.cend(); ++it) {
    entropy += it->weight * sum_weight_inv * log(it->weight * sum_weight_inv);
  }

  return -entropy;
}

bool SlamGMapping::GetOdomPose(const ros::Time& t,
                               OrientedPoint2d* const pose) {
  ROS_ASSERT(pose);

  centered_laser_pose_.stamp_ = t;

  tf::Stamped<tf::Transform> odom_pose;
  try {
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  } catch (const tf::TransformException& e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  const double yaw = tf::getYaw(odom_pose.getRotation());
  *pose = OrientedPoint2d(odom_pose.getOrigin().x(), odom_pose.getOrigin().y(),
                          yaw);
  return true;
}

}  // namespace gmapping
