#include <cassert>
#include <map>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <rtabmap_ros/RGBDImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv2/imgproc.hpp>

// Temporary
#include <iostream>
#include <opencv2/imgcodecs.hpp>

namespace osgar
{

namespace {
using Observation = std::vector<tf::Vector3>;

inline tf::Vector3 MatToVector3(const cv::Mat mat)
{
  float* data = reinterpret_cast<float*>(mat.data);
  tf::Vector3 vec;
  if (mat.rows == 3)
  {
    return {data[0], data[mat.step/sizeof(float)], data[2*mat.step/sizeof(float)]};
  }
  else
  {
    return {data[0], data[1], data[2]};
  }
}

}  // namespace

class Flyability
{
  public:
    Flyability() : ros_handle_("~"), rnd_gen_(1337), rnd_(0, 1) {}

    bool Init();

  private:
    ros::NodeHandle ros_handle_;
    ros::Subscriber depth_handler_;
    ros::Subscriber compressed_depth_handler_;
    ros::Subscriber camera_info_handler_;
    ros::Subscriber scan_handler_;
    ros::Subscriber range_handler_;
    tf::TransformListener transform_listener_;
    tf::TransformBroadcaster transform_broadcaster_;

    std::unordered_map<std::string, sensor_msgs::CameraInfo::ConstPtr> camera_infos_;
    std::mt19937 rnd_gen_;
    std::uniform_real_distribution<float> rnd_;

    struct {
      std::string robot_frame_id;
      std::string world_frame_id;

      // Maximum number of depth-based frames per input depth camera stored
      // in the local map.
      int max_depth_observations;
      // Maximum depth from depth camera, in meters.
      float max_depth;
      // Taking every n-th column and row in the input depth map.
      int depth_image_stride;
      // Storing only every n-th interesting point from the pointcloud.
      int depth_subsampling;

      // How often to publish outputs.
      ros::Duration publish_rate;
      // Dimensions of the robot.
      float robot_height_up, robot_height_bottom;
      float robot_radius;
      // What up/down slope is the drone willing to undertake?
      float max_slope;
      // Preferred height above ground.
      float above_ground;
      // Maximum number of scan-based frames per input lidar stored in the local
      // map.
      int max_scan_observations;
      // Ignoring lidar mesurements below this distance.
      float min_lidar_range;
      // Ignoring lidar mesurements past this distance.
      float max_lidar_range;
      // Storing every n-th interesting point from lidar.
      int lidar_subsampling;

      // Measurement variance of range sensors.
      float range_variance;
      float range_variance_increment_per_step;
      int max_range_observations;

      // Using at most this many nearby points when publishing output maps.
      int max_num_nearby_points;
      // Publishing points at most this far.
      float map_range;
      // Minimum range set in the published synthetic lidar scan representing
      // local map.
      float min_map_scan_range;
      // How far below and above the robot we report obstacles.
      float max_up_down_range;
    } config_;

    bool is_flying_ = false;

    struct RangeEstimate
    {
      double mean;
      double variance;
      uint64_t observations = 0;
    };
    std::map<std::string, RangeEstimate> range_estimates_;

    // Maps are keyed by frame_ids of individual sensors.
    std::map<std::string, int> observation_seq_id_;
    std::map<std::string, std::vector<Observation>> observations_;

    ros::Timer publish_timer_;
    ros::Publisher points_publisher_;
    ros::Publisher scan_publisher_;
    ros::Publisher ground_publisher_;
    ros::Publisher ceiling_publisher_;

    std::optional<tf::StampedTransform> GetTransform(
        const std::string& target_frame, const std::string& source_frame, const ros::Time& when) const;
    void InsertObservation(
        Observation&& observation, const std::string& frame_id,
        size_t max_num_observations_per_source);
    void HandleDepth(
        const cv::Mat& img, const std::string& camera_frame_id,
        const ros::Time& stamp);

    void OnCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void OnDepth(const sensor_msgs::Image::ConstPtr& msg);
    void OnCompressedDepth(const rtabmap_ros::RGBDImage::ConstPtr& msg);
    void OnScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void OnRange(const sensor_msgs::LaserScan::ConstPtr& msg);
    void OnTimer(const ros::TimerEvent& event);
};

bool Flyability::Init()
{
  ros_handle_.param<std::string>("robot_frame_id", config_.robot_frame_id, "");
  ros_handle_.param<std::string>("world_frame_id", config_.world_frame_id, "odom");
  ros_handle_.param("max_depth_observations", config_.max_depth_observations, 30 * 6);
  ros_handle_.param("max_depth", config_.max_depth, 10.0f);
  ros_handle_.param("depth_image_stride", config_.depth_image_stride, 4);
  ros_handle_.param("depth_subsampling", config_.depth_subsampling, 123);
  ros_handle_.param("robot_height_up", config_.robot_height_up, 0.1f);
  ros_handle_.param("robot_height_bottom", config_.robot_height_bottom, 0.4f);
  ros_handle_.param("robot_radius", config_.robot_radius, 0.7f);
  ros_handle_.param("max_slope", config_.max_slope, 28.0f);
  config_.max_slope = M_PI * config_.max_slope / 180;  // To radians.
  ros_handle_.param("above_ground", config_.above_ground, 3.0f);
  ros_handle_.param("max_scan_observations", config_.max_scan_observations, 20 * 6);
  ros_handle_.param("min_lidar_range", config_.min_lidar_range, 0.f);
  ros_handle_.param("max_lidar_range", config_.max_lidar_range, 10.0f);
  ros_handle_.param("lidar_subsampling", config_.lidar_subsampling, 4);
  ros_handle_.param("range_variance", config_.range_variance, 0.0009f);
  ros_handle_.param("range_variance_increment_per_step", config_.range_variance_increment_per_step, 0.01f);
  ros_handle_.param("max_range_observations", config_.max_range_observations, 500);
  float publish_rate_tmp;
  ros_handle_.param("publish_rate", publish_rate_tmp, 0.048f);
  config_.publish_rate = ros::Duration(publish_rate_tmp);
  ros_handle_.param("max_num_nearby_points", config_.max_num_nearby_points, 60000);
  ros_handle_.param("map_range", config_.map_range, 8.0f);
  ros_handle_.param("min_map_scan_range", config_.min_map_scan_range, 0.001f);
  ros_handle_.param("max_up_down_range", config_.max_up_down_range, 40.0f);

  camera_info_handler_ = ros_handle_.subscribe("input/camera_info", 10, &Flyability::OnCameraInfo, this);
  depth_handler_ = ros_handle_.subscribe("input/depth", 10, &Flyability::OnDepth, this);
  compressed_depth_handler_ = ros_handle_.subscribe("input/depth/compressed", 10, &Flyability::OnCompressedDepth, this);
  scan_handler_ = ros_handle_.subscribe("input/scan", 10, &Flyability::OnScan, this);
  range_handler_ = ros_handle_.subscribe("input/range", 10, &Flyability::OnRange, this);

  publish_timer_ = ros_handle_.createTimer(config_.publish_rate, &Flyability::OnTimer, this);
  points_publisher_ = ros_handle_.advertise<sensor_msgs::PointCloud2>("output/map", 10);
  scan_publisher_ = ros_handle_.advertise<sensor_msgs::LaserScan>("output/scan", 10);
  ground_publisher_ = ros_handle_.advertise<sensor_msgs::LaserScan>("output/down", 10);
  ceiling_publisher_ = ros_handle_.advertise<sensor_msgs::LaserScan>("output/up", 10);
  return true;
}

std::optional<tf::StampedTransform> Flyability::GetTransform(
        const std::string& target_frame, const std::string& source_frame, const ros::Time& when) const
{
  tf::StampedTransform pose;
  tf::TransformException error("none");
  for (int i = 0; i < 40; ++i)
  {
    try
    {
      transform_listener_.lookupTransform(target_frame, source_frame, when, pose);
      return pose;
    }
    catch (tf::TransformException e)
    {
      error = e;
      ros::Duration(0.005).sleep();
    }
  }
  ROS_ERROR("%s", error.what());
  return {};
}

void Flyability::InsertObservation(
        Observation&& observation, const std::string& frame_id,
        const size_t max_num_observations_per_source)
{
  if (observation.empty())  // Skip empty measurements.
  {
    return;
  }

  auto& observations = observations_[frame_id];
  if (observations.size() == max_num_observations_per_source)
  {
    observations[observation_seq_id_[frame_id] %
                 max_num_observations_per_source] = observation;
  }
  else
  {
    observations.push_back(observation);
  }
  ++observation_seq_id_[frame_id];
}

void Flyability::OnCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_infos_.insert({msg->header.frame_id, msg});
}

void Flyability::OnDepth(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr input_img_ptr;
  try
  {
    input_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  HandleDepth(input_img_ptr->image, msg->header.frame_id, msg->header.stamp);
}

void Flyability::OnCompressedDepth(const rtabmap_ros::RGBDImage::ConstPtr& msg)
{
  cv::Mat decompressed = cv::imdecode(msg->depth_compressed.data, cv::IMREAD_UNCHANGED);
  cv::Mat depth(decompressed.rows,
                decompressed.cols,
                CV_32F,
                decompressed.data,
                decompressed.step);
  HandleDepth(depth, msg->header.frame_id, msg->header.stamp);
}

void Flyability::HandleDepth(
    const cv::Mat& img, const std::string& camera_frame_id,
    const ros::Time& stamp)
{
  if (!is_flying_)
  {
    return;  // Otherwise we would be putting ground in starting area into our map and the drone could not even take off.
  }

  const auto camera_info = camera_infos_.find(camera_frame_id);
  if (camera_info == camera_infos_.end())
  {
    ROS_DEBUG("Received an image from a camera with unknown parameters: %s", camera_frame_id.c_str());
    return;
  }

  std::optional<tf::StampedTransform> sensor_pose =
    GetTransform(config_.world_frame_id, camera_frame_id, stamp);
  if (!sensor_pose) return;

  std::optional<tf::StampedTransform> robot_pose =
    GetTransform(config_.world_frame_id, config_.robot_frame_id, stamp);
  if (!robot_pose) return;

  cv::Mat fog_filtered_img;
  cv::medianBlur(img, fog_filtered_img, 5);
  const auto& ci = camera_info->second;
  // Intrinsic camera matrix before undistortion.
  const cv::Mat camera_k(3, 3, CV_64F, const_cast<double*>(ci->K.data()));
  // Camera distortion parameters.
  const cv::Mat camera_d(1, ci->D.size(), CV_64F, const_cast<double*>(ci->D.data()));
  cv::Mat undistorted_img;
  cv::undistort(fog_filtered_img, undistorted_img, camera_k, camera_d);

  if (config_.depth_image_stride != 1)
  {
    cv::Mat smaller;
    cv::resize(undistorted_img,
               smaller,
               cv::Size(undistorted_img.cols / config_.depth_image_stride,
                        undistorted_img.rows / config_.depth_image_stride));
    undistorted_img = smaller;
  }

  // Intrinsic camera matrix after undistortion.
  const cv::Mat camera_p_all(3, 4, CV_64F, const_cast<double*>(ci->P.data()));
  cv::Mat camera_p(camera_p_all, cv::Rect(0, 0, 3, 3));
  camera_p.convertTo(camera_p, CV_32F);
  const cv::Mat camera_p_inv = camera_p.inv();
  Observation points;
  points.reserve(undistorted_img.rows * undistorted_img.cols);

  cv::Mat full_size_uv(3, 1, CV_32F);
  float& full_size_uv_v = full_size_uv.at<float>(1, 0);
  float& full_size_uv_u = full_size_uv.at<float>(0, 0);
  full_size_uv.at<float>(2, 0) = 1;
  size_t cnt = 0;
  for (int v = 0; v < undistorted_img.rows; ++v)
  {
    float* undistorted_img_row_v = reinterpret_cast<float*>(undistorted_img.ptr(v));
    full_size_uv_v = v * config_.depth_image_stride;
    for (int u = 0; u < undistorted_img.cols; ++u) 
    {
      full_size_uv_u = u * config_.depth_image_stride;
      const float depth = undistorted_img_row_v[u];
      if (depth > 0 && depth < config_.max_depth && !std::isnan(depth) &&
          cnt++ % (config_.depth_subsampling / config_.depth_image_stride / config_.depth_image_stride) == 0)
      {
        // For some reason, doing the calculation with tf::Transform is a lot
        // faster than with OpenCV's Mat.
        const tf::Vector3 pt = *sensor_pose * (depth * MatToVector3(camera_p_inv * full_size_uv));
        points.push_back(std::move(pt));
      }
    }
  }

  InsertObservation(std::move(points), camera_frame_id,
                    config_.max_depth_observations);
}

float median(std::vector<float> values)
{
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

void Flyability::OnScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::optional<tf::StampedTransform> sensor_pose =
    GetTransform(config_.world_frame_id, msg->header.frame_id, msg->header.stamp);
  if (!sensor_pose) return;

  Observation points;
  points.reserve(msg->ranges.size());
  size_t cnt = 0;
  tf::Transform rot;
  rot.setIdentity();  // Initializes offsets to zero.
  tf::Vector3 beam;
  for (size_t i = 2; i  + 2 < msg->ranges.size(); ++i)
  {
    const float angle = msg->angle_min + i * msg->angle_increment;
    // Fog filtering.
    const float range = median({
        msg->ranges[i - 2],
        msg->ranges[i - 1],
        msg->ranges[i],
        msg->ranges[i + 1],
        msg->ranges[i + 2]});
    if (range <= config_.min_lidar_range || range >= config_.max_lidar_range)  // Skip invalid/missing/ignored measurements.
    {
      continue;
    }

    beam.setX(range);
    rot.getBasis().setRPY(0, 0, angle);
    tf::Vector3 pt = *sensor_pose * rot * beam;

    if (cnt++ % config_.lidar_subsampling == 0)
    {
      points.push_back(std::move(pt));
    }
  }

  InsertObservation(std::move(points),
                    msg->header.frame_id,
                    config_.max_scan_observations);
}

void Flyability::OnRange(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (msg->ranges.size() != 1)
  {
    ROS_ERROR("Expecting single range distance. Got: %d", (int) msg->ranges.size());
    return;
  }

  auto measurement = msg->ranges[0];
  if (measurement <= msg->range_min || measurement >= msg->range_max)
  {
    return;
  }

  std::optional<tf::StampedTransform> sensor_pose =
    GetTransform(config_.world_frame_id, msg->header.frame_id, msg->header.stamp);
  if (!sensor_pose) return;

  auto& estimate = range_estimates_[msg->header.frame_id];
  if (estimate.observations == 0)
  {
    estimate.mean = measurement;
    estimate.variance = config_.range_variance;
    estimate.observations = 1;
  }
  else
  {
    // Kalman filter over time.
    estimate.variance += config_.range_variance_increment_per_step;
    const double k = estimate.variance / (estimate.variance + config_.range_variance);
    estimate.mean += k * (measurement - estimate.mean);
    estimate.variance = (1 - k) * estimate.variance;
    ++estimate.observations;
  }

  tf::Transform rot;
  rot.setIdentity();  // Initializes offsets to zero.
  rot.getBasis().setRPY(0, 0, msg->angle_min);
  tf::Vector3 beam(estimate.mean, 0, 0);
  const tf::Vector3 pt = *sensor_pose * rot * beam;
  InsertObservation({std::move(pt)},
                    msg->header.frame_id,
                    config_.max_range_observations);
}

void Flyability::OnTimer(const ros::TimerEvent& event)
{
  if (points_publisher_.getNumSubscribers() == 0 &&
      scan_publisher_.getNumSubscribers() == 0 &&
      ground_publisher_.getNumSubscribers() == 0 &&
      ceiling_publisher_.getNumSubscribers() == 0)
  {
    return;  // Nobody cares. Do not bother with all the calculation.
  }

  const ros::Time latest(0);
  const auto to_local = GetTransform(config_.robot_frame_id, config_.world_frame_id, latest);
  if (!to_local) return;
  if (to_local->getOrigin().z() < -(config_.robot_height_bottom + config_.robot_height_up))
  {
    is_flying_ = true;
  }

  const auto to_global = to_local->inverse();

  // Publish a TF with current position of the robot, current heading of the robot,
  // but roll and pitch aligned horizontally, i.e. both zero.
  const tf::Quaternion full_rotation = to_global.getRotation();
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(full_rotation).getRPY(roll, pitch, yaw);
  tf::Quaternion recover_yaw;
  recover_yaw.setRPY(0, 0, yaw);
  recover_yaw.normalize();
  tf::Quaternion horizontally = full_rotation.inverse() * recover_yaw;
  horizontally.normalize();
  tfScalar y, p, r;
  tf::Matrix3x3(horizontally * full_rotation).getRPY(r, p, y);
  tf::Transform horizontal_tf;
  horizontal_tf.setIdentity();
  horizontal_tf.setRotation(horizontally);
  const std::string horizontal_frame_id = config_.robot_frame_id + "/horizontal";
  transform_broadcaster_.sendTransform(
      tf::StampedTransform(
        horizontal_tf, event.current_real, config_.robot_frame_id, horizontal_frame_id));

  // Convert all collected pointclouds to the current horizontally aligned
  // coordinate system.
  const tf::Transform horizontal_tf_inv = horizontal_tf.inverse();
  // Convert all collected pointclouds to be relative to the current robot pose.
  std::vector<tf::Vector3> local_pts;
  float ground_distance = -std::numeric_limits<float>::infinity();
  float ceiling_distance = std::numeric_limits<float>::infinity();
  for (const auto single_source_observations : observations_)
  {
    const auto& observations = single_source_observations.second;
    for (const auto& observation : observations)
    {
      for (const auto& pt : observation)
      {
        const auto local_pt = horizontal_tf_inv* *to_local * pt;
        local_pts.push_back(local_pt);
        if (std::hypot(local_pt.x(), local_pt.y()) <= config_.robot_radius)
        {
          if (local_pt.z() < 0 && local_pt.z() > ground_distance)
          {
            ground_distance = local_pt.z();
          }
          else if (local_pt.z() > 0 && local_pt.z() < ceiling_distance)
          {
            ceiling_distance = local_pt.z();
          }
        }
      }
    }
  }

  // Publish ground and ceiling distance.
  if (ground_publisher_.getNumSubscribers())
  {
    tf::Quaternion down_rot;
    down_rot.setRPY(0, M_PI_2, 0);
    down_rot.normalize();
    tf::Transform down_tf;
    down_tf.setIdentity();
    down_tf.setRotation(down_rot);
    const std::string down_frame_id = config_.robot_frame_id + "/down";
    transform_broadcaster_.sendTransform(
        tf::StampedTransform(
          down_tf, event.current_real, horizontal_frame_id, down_frame_id));
    sensor_msgs::LaserScan down_scan;
    down_scan.header.frame_id = down_frame_id;
    down_scan.header.stamp = event.current_real;
    down_scan.angle_min = 0;
    down_scan.angle_max = 1e-6;
    down_scan.angle_increment = 1e-3;
    down_scan.range_min = 1e-3;
    down_scan.range_max = config_.max_up_down_range;
    // Correcting for the negative relative coordinate of ground.
    down_scan.ranges.push_back(-ground_distance);
    ground_publisher_.publish(down_scan);
  }

  if (ceiling_publisher_.getNumSubscribers())
  {
    tf::Quaternion up_rot;
    up_rot.setRPY(0, -M_PI_2, 0);
    up_rot.normalize();
    tf::Transform up_tf;
    up_tf.setIdentity();
    up_tf.setRotation(up_rot);
    const std::string up_frame_id = config_.robot_frame_id + "/up";
    transform_broadcaster_.sendTransform(
        tf::StampedTransform(
          up_tf, event.current_real, horizontal_frame_id, up_frame_id));
    sensor_msgs::LaserScan up_scan;
    up_scan.header.frame_id = up_frame_id;
    up_scan.header.stamp = event.current_real;
    up_scan.angle_min = 0;
    up_scan.angle_max = 1e-6;
    up_scan.angle_increment = 1e-3;
    up_scan.range_min = 1e-3;
    up_scan.range_max = config_.max_up_down_range;
    up_scan.ranges.push_back(ceiling_distance);
    ceiling_publisher_.publish(up_scan);
  }

  // Focus on points in the small neighborhood of the robot.
  std::vector<tf::Vector3> nearby_pts;
  nearby_pts.reserve(local_pts.size());
  // Keeping points beyond the max range so that even at the last point within
  // range we have enough data to estimate the normal of the surface.
  std::copy_if(
      local_pts.begin(), local_pts.end(), std::back_inserter(nearby_pts),
      [this](const tf::Vector3& pt)
      {
        return std::hypot(pt.x(), pt.y()) < this->config_.map_range;
      });

  if (nearby_pts.size() > static_cast<size_t>(config_.max_num_nearby_points))
  {
    for (size_t i = 0; i < nearby_pts.size(); ++i)
    {
      size_t j = rnd_(rnd_gen_) * nearby_pts.size();
      if (i != j)
      {
        std::swap(nearby_pts[i], nearby_pts[j]);
      }
    }
    nearby_pts.resize(config_.max_num_nearby_points);
  }

  ROS_DEBUG("Pointcloud: %zu / %zu", nearby_pts.size(), local_pts.size());

  if (points_publisher_.getNumSubscribers())
  {
    sensor_msgs::PointCloud2 local_map;
    local_map.header.stamp = event.current_real;
    local_map.header.frame_id = horizontal_frame_id;
    local_map.height = 1;
    local_map.width = nearby_pts.size();
    local_map.fields.resize(3);
    auto& field_x = local_map.fields[0];
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::PointField::FLOAT32;
    field_x.count = 1;
    auto& field_y = local_map.fields[1];
    field_y.name = "y";
    field_y.offset = sizeof(float);
    field_y.datatype = sensor_msgs::PointField::FLOAT32;
    field_y.count = 1;
    auto& field_z = local_map.fields[2];
    field_z.name = "z";
    field_z.offset = 2 * sizeof(float);
    field_z.datatype = sensor_msgs::PointField::FLOAT32;
    field_z.count = 1;
    local_map.point_step = 3 * sizeof(float);
    local_map.data.resize(nearby_pts.size() * local_map.point_step);
    local_map.row_step = local_map.data.size();
    local_map.is_bigendian = false;
    local_map.is_dense = true;
    float* local_map_data = reinterpret_cast<float*>(local_map.data.data());
    for (const auto& pt : nearby_pts)
    {
      local_map_data[0] = pt.x();
      local_map_data[1] = pt.y();
      local_map_data[2] = pt.z();
      local_map_data += 3;
    }
    points_publisher_.publish(local_map);
  }

  if (scan_publisher_.getNumSubscribers())
  {
    sensor_msgs::LaserScan local_scan;
    local_scan.header.frame_id = horizontal_frame_id;
    local_scan.header.stamp = event.current_real;
    local_scan.angle_min = -M_PI;
    local_scan.angle_max = 179 * M_PI / 180;
    local_scan.angle_increment = M_PI / 180;
    local_scan.range_min = config_.min_map_scan_range;
    local_scan.range_max = config_.map_range;
    local_scan.ranges.resize(
        1 + (local_scan.angle_max - local_scan.angle_min) /
              local_scan.angle_increment);
    auto& slopes = local_scan.intensities;  // We use `insetnsities` to store the selected slope.
    slopes.resize(local_scan.ranges.size());
    struct Obstruction {
      float low;
      float middle;
      float high;
      float safe_distance;
      float full_distance;
      float z;
    };
    std::vector<std::vector<Obstruction>> obstructions;
    obstructions.resize(local_scan.ranges.size());
    for (const auto& pt : nearby_pts)
    {
      const float direction = std::atan2(pt.y(), pt.x());
      const size_t bucket = static_cast<size_t>(
          std::round((direction - local_scan.angle_min) /
                     local_scan.angle_increment)) %
          local_scan.ranges.size();
      const float distance = std::hypot(pt.x(), pt.y());
      // The *front* edge of the drone needs to fly over the obstacle and not
      // just its center. This leads to a shorter climbing distance.
      const float d = std::max(1e-3f, distance - config_.robot_radius);
      // The bottom edge of the drone (-robot_height_bottom) needs to climb above the
      // obstacle (robot_height_up) and similar for top edge below an obstacle.
      obstructions[bucket].emplace_back(Obstruction{
          static_cast<float>(std::atan2(pt.z() - config_.robot_height_up, d)),
          static_cast<float>(std::atan2(pt.z(), d)),
          static_cast<float>(std::atan2(pt.z() + config_.robot_height_bottom, d)),
          d,
          distance,
          static_cast<float>(pt.z())});
    }

    for (size_t i = 0; i < local_scan.ranges.size(); ++i)
    {
      auto& obstructions_i = obstructions[i];
      std::sort(obstructions_i.begin(), obstructions_i.end(),
          [](const Obstruction& a, const Obstruction& b)
          {
            if (a.low < b.low)
            {
              return true;
            }
            else if (a.low == b.low && a.high < b.high)
            {
              return true;
            }
            else if (a.low == b.low && a.high == b.high && a.safe_distance > b.safe_distance)
            {
              return true;
            }
            else if (a.low == b.low && a.high == b.high && a.safe_distance == b.safe_distance)
            {
              return a.z < b.z;
            }
            return false;
          });

      if (obstructions_i.empty())
      {
        local_scan.ranges[i] = 0;   // No obstacle ahead.
        slopes[i] = -config_.max_slope;
        continue;
      }

      if (obstructions_i.front().low >= -config_.max_slope)
      {
        local_scan.ranges[i] = 0;  // The drone can dive under the obstacles.
        slopes[i] = -config_.max_slope;
        continue;
      }

      size_t below_idx = 0;
      size_t above_idx = obstructions_i.size();
      for (size_t j = 1; j < obstructions_i.size(); ++j)
      {
        const auto obstruction = obstructions_i[j];
        const float high = obstructions_i[below_idx].high;
        if (obstruction.low > high && obstruction.low >= -config_.max_slope)
        {
          above_idx = j;
          break;
        }
        if (obstruction.high > high)
        {
          below_idx = j;
        }
      }
      if (obstructions_i[below_idx].high <= config_.max_slope)
      {
        local_scan.ranges[i] = 0;  // There is a way through in this direction.
        const auto& below = obstructions_i[below_idx];
        const float z_below = std::tan(below.middle) * below.safe_distance;
        const float z_safe = z_below + config_.above_ground;
        if (above_idx == obstructions_i.size())
        {
          slopes[i] = std::atan2(z_safe, below.safe_distance);
        }
        else
        {
          const auto& above = obstructions_i[above_idx];
          slopes[i] = std::min(
              std::atan2(z_safe, below.safe_distance),
              (below.high + above.low) / 2);
        }
        slopes[i] = std::max(
            -config_.max_slope,
            std::min(config_.max_slope, slopes[i]));
        continue;
      }

      // No way through. Let's report distance to obstacle as if only flying
      // horizontally.
      auto& range = local_scan.ranges[i];
      range = std::numeric_limits<float>::infinity();
      for (const auto& o : obstructions_i)
      {
        if (o.z >= -config_.robot_height_bottom &&
            o.z <= config_.robot_height_up)
        {
          range = std::min(range, o.full_distance);
        }
      }
      if (std::isinf(range))
      {
        range = 0;  // This should not happen. How comes we found a way forward without finding it above?
        slopes[i] = 0;
      }
    }

    scan_publisher_.publish(local_scan);
  }
}

}  // namespace osgar

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flyability");
  cv::setNumThreads(1);
  osgar::Flyability flyability;
  if (!flyability.Init())
  {
    return -1;
  }
  ros::spin();
}
