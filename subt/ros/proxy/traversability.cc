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
#include <std_msgs/Empty.h>
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
#if 1
    return {data[0], data[mat.step/sizeof(float)], data[2*mat.step/sizeof(float)]};
#else
    return {mat.at<float>(0, 0),
            mat.at<float>(1, 0),
            mat.at<float>(2, 0)};
#endif
  }
  else
  {
    return {data[0], data[1], data[2]};
  }
}

inline cv::Point3f Vector3ToPoint3f(const tf::Vector3& vec)
{
  return {static_cast<float>(vec.x()),
          static_cast<float>(vec.y()),
          static_cast<float>(vec.z())};
}

}  // namespace

class Traversability
{
  public:
    Traversability() : ros_handle_("~"), rnd_gen_(1337), rnd_(0, 1) {}

    bool Init();

  private:
    ros::NodeHandle ros_handle_;
    ros::Subscriber depth_handler_;
    ros::Subscriber compressed_depth_handler_;
    ros::Subscriber camera_info_handler_;
    ros::Subscriber scan_handler_;
    ros::Subscriber breadcrumb_handler_;
    tf::TransformListener transform_listener_;

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
      // We use points this many pixels apart, in the original image before
      // applying stride, to detect horizontal and vertical surfaces.
      int horizontal_pixel_offset, vertical_pixel_offset;
      // Maximum metric distance between two points for them to be still
      // considered part of the same surface.
      float max_neighbor_distance;

      // How often to publish outputs.
      ros::Duration publish_rate;
      // When publishing local map and pointcloud, ignore points more than this
      // above the robot.
      float max_relative_z;
      // Maximum number of scan-based frames per input lidar stored in the local
      // map.
      int max_scan_observations;
      // Ignoring lidar mesurements below this distance.
      float min_lidar_range;
      // Ignoring lidar mesurements past this distance.
      float max_lidar_range;
      // Storing every n-th interesting point from lidar.
      int lidar_subsampling;

      // Using at most this many nearby points when publishing output maps.
      int max_num_nearby_points;
      // Publishing points at most this far.
      float map_range;
      // Minimum range set in the published synthetic lidar scan representing
      // local map.
      float min_map_scan_range;

      // Traversability slope threshold, in radians. User input in ROS launch
      // file needs to be in degrees.
      float max_slope;
      // A vertical obstacle needs to be at least this many meters high to be
      // considered a real obstacle and not just a road bump.
      float max_bump_height;
      // A dip not deeper than this threshold is considered something that
      // the robot can drive over. These dips occur, for example, when crossing
      // a rail submerged into the surface.
      float max_dip_down;
      // A dipped obstacle, such as a rail, can extrude slightly above the
      // surrounding surface.
      float max_dip_up;
      // We insert a synthetic obstacle this far into the map when there is
      // no traversable map ahead. For the system to work, a horizontally
      // placed robot needs to have this obstacle at the bottom of depth
      // camera vision just below the ground surface.
      float synthetic_obstacle_distance;
      // To feel comforrtable, we need to see ground somewhere between these
      // two distance thresholds.
      float visible_ground_min, visible_ground_max;
      // Obstacles more than this above ground are ignored, because they are not
      // relevant. the robot can go under them,
      float robot_height;

      // We mask out obstacles around dropped breadcrumbs to avoid getting stuck
      // on them.
      float breadcrumb_radius;
      float breadcrumb_height;
    } config_;

    // Maps are keyed by frame_ids of individual sensors.
    std::map<std::string, int> observation_seq_id_;
    std::map<std::string, std::vector<Observation>> observations_;

    std::vector<tf::Vector3> breadcrumbs_;

    ros::Timer publish_timer_;
    ros::Publisher points_publisher_;
    ros::Publisher scan_publisher_;

    std::optional<tf::StampedTransform> GetTransform(
        const std::string& target_frame, const std::string& source_frame, const ros::Time& when) const;
    void InsertObservation(
        Observation&& observation, const std::string& frame_id,
        size_t max_num_observations_per_source);
    void HandleDepth(
        const cv::Mat& img, const std::string& camera_frame_id,
        const ros::Time& stamp);
    bool NearBreadcrumb(const tf::Vector3& pt) const;

    void OnCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void OnDepth(const sensor_msgs::Image::ConstPtr& msg);
    void OnCompressedDepth(const rtabmap_ros::RGBDImage::ConstPtr& msg);
    void OnScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void OnBreadcrumb(const std_msgs::Empty::ConstPtr& msg);
    void OnTimer(const ros::TimerEvent& event);
};

bool Traversability::Init()
{
  ros_handle_.param<std::string>("robot_frame_id", config_.robot_frame_id, "");
  ros_handle_.param<std::string>("world_frame_id", config_.world_frame_id, "odom");
  ros_handle_.param("max_depth_observations", config_.max_depth_observations, 30 * 6);
  ros_handle_.param("max_depth", config_.max_depth, 10.0f);
  ros_handle_.param("depth_image_stride", config_.depth_image_stride, 2);
  ros_handle_.param("depth_subsampling", config_.depth_subsampling, 123);
  ros_handle_.param("horizontal_pixel_offset", config_.horizontal_pixel_offset, 12);
  ros_handle_.param("vertical_pixel_offset", config_.vertical_pixel_offset, 12);
  ros_handle_.param("max_neighbor_distance", config_.max_neighbor_distance, 0.3f);
  ros_handle_.param("max_relative_z", config_.max_relative_z, 40.f);
  ros_handle_.param("max_scan_observations", config_.max_scan_observations, 20 * 6);
  ros_handle_.param("min_lidar_range", config_.min_lidar_range, 0.f);
  ros_handle_.param("max_lidar_range", config_.max_lidar_range, 10.0f);
  ros_handle_.param("lidar_subsampling", config_.lidar_subsampling, 4);
  float publish_rate_tmp;
  ros_handle_.param("publish_rate", publish_rate_tmp, 0.048f);
  config_.publish_rate = ros::Duration(publish_rate_tmp);
  ros_handle_.param("max_num_nearby_points", config_.max_num_nearby_points, 60000);
  ros_handle_.param("map_range", config_.map_range, 8.0f);
  ros_handle_.param("min_map_scan_range", config_.min_map_scan_range, 0.001f);
  ros_handle_.param("max_slope", config_.max_slope, 35.0f); // In degrees.
  config_.max_slope = config_.max_slope * M_PI / 180;  // Converting to radians.
  ros_handle_.param("max_bump_height", config_.max_bump_height, 0.07f);
  ros_handle_.param("max_dip_down", config_.max_dip_down, 0.2f);
  ros_handle_.param("max_dip_up", config_.max_dip_up, 0.35f);
  ros_handle_.param("synthetic_obstacle_distance", config_.synthetic_obstacle_distance, 0.5f);
  ros_handle_.param("visible_ground_min", config_.visible_ground_min, 0.6f);
  ros_handle_.param("visible_ground_max", config_.visible_ground_max, 2.5f);
  ros_handle_.param("robot_height", config_.robot_height, 0.5f);
  ros_handle_.param("breadcrumb_radius", config_.breadcrumb_radius, 0.6f);
  ros_handle_.param("breadcrumb_height", config_.breadcrumb_height, 0.5f);

  camera_info_handler_ = ros_handle_.subscribe("input/camera_info", 10, &Traversability::OnCameraInfo, this);
  depth_handler_ = ros_handle_.subscribe("input/depth", 10, &Traversability::OnDepth, this);
  compressed_depth_handler_ = ros_handle_.subscribe("input/depth/compressed", 10, &Traversability::OnCompressedDepth, this);
  scan_handler_ = ros_handle_.subscribe("input/scan", 10, &Traversability::OnScan, this);
  breadcrumb_handler_ = ros_handle_.subscribe("input/breadcrumb", 10, &Traversability::OnBreadcrumb, this);

  publish_timer_ = ros_handle_.createTimer(config_.publish_rate, &Traversability::OnTimer, this);
  points_publisher_ = ros_handle_.advertise<sensor_msgs::PointCloud2>("output/map", 10);
  scan_publisher_ = ros_handle_.advertise<sensor_msgs::LaserScan>("output/scan", 10);
  return true;
}

std::optional<tf::StampedTransform> Traversability::GetTransform(
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

void Traversability::InsertObservation(
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

void Traversability::OnCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_infos_.insert({msg->header.frame_id, msg});
}

void Traversability::OnDepth(const sensor_msgs::Image::ConstPtr& msg)
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

void Traversability::OnCompressedDepth(const rtabmap_ros::RGBDImage::ConstPtr& msg)
{
  cv::Mat decompressed = cv::imdecode(msg->depth_compressed.data, cv::IMREAD_UNCHANGED);
  cv::Mat depth(decompressed.rows,
                decompressed.cols,
                CV_32F,
                decompressed.data,
                decompressed.step);
  HandleDepth(depth, msg->header.frame_id, msg->header.stamp);
}

void Traversability::HandleDepth(
    const cv::Mat& img, const std::string& camera_frame_id,
    const ros::Time& stamp)
{
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

  cv::Mat xyz(undistorted_img.rows, undistorted_img.cols, CV_32FC3);
  cv::Mat full_size_uv(3, 1, CV_32F);
  float& full_size_uv_v = full_size_uv.at<float>(1, 0);
  float& full_size_uv_u = full_size_uv.at<float>(0, 0);
  full_size_uv.at<float>(2, 0) = 1;
  const cv::Point3f INVALID_POINT(0, 0, 0);
  for (int v = 0; v < undistorted_img.rows; ++v)
  {
    float* undistorted_img_row_v = reinterpret_cast<float*>(undistorted_img.ptr(v));
    cv::Point3f* xyz_row_v = reinterpret_cast<cv::Point3f*>(xyz.ptr(v));
    full_size_uv_v = v * config_.depth_image_stride;
    for (int u = 0; u < undistorted_img.cols; ++u) 
    {
      full_size_uv_u = u * config_.depth_image_stride;
      const float depth = undistorted_img_row_v[u];
      auto& pt = xyz_row_v[u];
      if (depth <= 0 || depth > config_.max_depth || std::isnan(depth))
      {
        pt = INVALID_POINT;
      }
      else
      {
        // For some reason, doing the calculation with tf::Transform is a lot
        // faster than with OpenCV's Mat.
        pt = Vector3ToPoint3f(*sensor_pose * (depth * MatToVector3(camera_p_inv * full_size_uv)));
      }
    }
  }
  // We work with three points of interest: A central one, one horizontally in
  // the image from the central one and one vertically in the image from the
  // central one.
  const int horizontal_pixel_offset = std::max(1, config_.horizontal_pixel_offset / config_.depth_image_stride);
  const int vertical_pixel_offset = std::max(1, config_.vertical_pixel_offset / config_.depth_image_stride);
  cv::Mat horizontal = cv::Mat::zeros(undistorted_img.rows, undistorted_img.cols, CV_8U);
  cv::Mat vertical = cv::Mat::zeros(undistorted_img.rows, undistorted_img.cols, CV_8U);
  const float cos_horizontal_threshold = std::cos(M_PI_2 - config_.max_slope);
  for (int v_center = 0;
      v_center + vertical_pixel_offset < undistorted_img.rows;
      ++v_center)
  {
    const int v_right = v_center;
    const int v_bottom = v_center + vertical_pixel_offset;
    cv::Point3f* xyz_row_v_center = reinterpret_cast<cv::Point3f*>(xyz.ptr(v_center));
    cv::Point3f* xyz_row_v_right = xyz_row_v_center;
    cv::Point3f* xyz_row_v_bottom = reinterpret_cast<cv::Point3f*>(xyz.ptr(v_bottom));
    for (int u_center = 0;
        u_center + horizontal_pixel_offset < undistorted_img.cols;
        ++u_center)
    {
      const int u_right = u_center + horizontal_pixel_offset;
      const int u_bottom = u_center;

      const cv::Point3f& pt_center = xyz_row_v_center[u_center];
      const cv::Point3f& pt_right = xyz_row_v_right[u_right];
      const cv::Point3f& pt_bottom = xyz_row_v_bottom[u_bottom];
      if (pt_center == INVALID_POINT || pt_right == INVALID_POINT ||
          pt_bottom == INVALID_POINT)
      {
        continue;
      }

      const auto right = pt_right - pt_center;
      const auto bottom = pt_bottom - pt_center;
      const auto normal = bottom.cross(right);
      const auto cos_tilt = normal.z / cv::norm(normal);
      if (cos_tilt > cos_horizontal_threshold)
      {
        horizontal.at<uint8_t>(v_center, u_center) = 255;
      }
      else if (cos_tilt > -cos_horizontal_threshold &&
               cv::norm(right) <= config_.max_neighbor_distance &&
               cv::norm(bottom) <= config_.max_neighbor_distance)
      {
        vertical.at<uint8_t>(v_center, u_center) =
          vertical.at<uint8_t>(v_right, u_right) =
          vertical.at<uint8_t>(v_bottom, u_bottom) = 255;
      }
      // else ceiling
    }
  }
  cv::Mat horizontal_components, vertical_components;
  const int num_horizontal_components = cv::connectedComponents(
      horizontal, horizontal_components);
  const int num_vertical_components = cv::connectedComponents(
      vertical, vertical_components);
  std::vector<std::vector<cv::Point2i>> horizontal_uvs(
      num_horizontal_components - 1);
  std::vector<std::vector<cv::Point2i>> vertical_uvs(num_vertical_components - 1);
  constexpr int BACKGROUND_COMPONENT = 0;
  for (int v = 0; v < undistorted_img.rows; ++v)
  {
    for (int u = 0; u < undistorted_img.cols; ++u)
    {
      const int32_t hor = horizontal_components.at<int32_t>(v, u);
      const int32_t ver = vertical_components.at<int32_t>(v, u);
      if (hor != BACKGROUND_COMPONENT)
      {
        horizontal_uvs[hor - 1].emplace_back(u, v);
      }
      if (ver != BACKGROUND_COMPONENT)
      {
        vertical_uvs[ver - 1].emplace_back(u, v);
      }
    }
  }

  // - Only high-enough vertical components count as an obstacle.
  std::map<int, std::vector<cv::Point2i>> grouped_uvs;
  std::vector<cv::Point3f> pts;
  const auto verticality_comparator = [](const cv::Point3f& a, const cv::Point3f& b) {
    return a.z < b.z;
  };
  cv::Mat pruned_vertical = cv::Mat::zeros(undistorted_img.rows, undistorted_img.cols, CV_8U);
  for (auto& component : vertical_uvs)
  {
    pts.clear();
    for (const auto& uv : component)
    {
      pts.push_back(xyz.at<cv::Point3f>(uv.y, uv.x));
    }
    const auto& pt_low = *std::min_element(
        pts.begin(), pts.end(), verticality_comparator);
    const auto& pt_high = *std::max_element(
        pts.begin(), pts.end(), verticality_comparator);
    if (pt_high.z - pt_low.z > config_.max_bump_height)
    {
      for (const auto& uv : component)
      {
        pruned_vertical.at<uint8_t>(uv.y, uv.x) = 255;
      }
    }
  }

  const auto& robot_xyz = robot_pose->getOrigin();
  const auto& sensor_xyz = sensor_pose->getOrigin();
  size_t cnt = 0;
  for (int u = 0; u < undistorted_img.cols - horizontal_pixel_offset; ++u)
  {
    bool obscured_view = false;
    bool ground_visible = false;

    // If ground detection is disabled, pretend we are happily seeing it.
    if (config_.visible_ground_min == 0 &&
	config_.visible_ground_max == 0) {
	    ground_visible = true;
    }

    float expected_ground = robot_xyz.z();
    cv::Mat full_size_uv(3, 1, CV_32F);
    // As if there was an obstacle visible at the bottom line of the image,
    // config_.synthetic_obstacle_distance far.
    full_size_uv.at<float>(0, 0) = u * config_.depth_image_stride;
    full_size_uv.at<float>(1, 0) = undistorted_img.rows * config_.depth_image_stride - 1; 
    full_size_uv.at<float>(2, 0) = 1;
    const auto pt_down_near = *sensor_pose * (config_.synthetic_obstacle_distance * MatToVector3(camera_p_inv * full_size_uv));

    for (int v = undistorted_img.rows - vertical_pixel_offset - 1; v >= 0; --v)
    {
      if (!horizontal.at<uint8_t>(v, u) &&
          !pruned_vertical.at<uint8_t>(v, u))
      {
        continue;
      }

      // Ignore points far above where we expect ground.
      const cv::Point3f pt = xyz.at<cv::Point3f>(v, u);
      if (pt.z > expected_ground + config_.robot_height)
      {
        continue;
      }

      const float pt_dist_from_sensor =
        std::hypot(pt.x - sensor_xyz.x(), pt.y - sensor_xyz.y());

      // Vertical obstacle.
      if (pruned_vertical.at<uint8_t>(v, u))
      {
        // Vertical surfaces in narrow dips in the surface can be ignored,
        // because the robot can cross the dip. A rail sunk under the surface
        // is an example of such surface.
        if (pt.z - expected_ground > config_.max_dip_up ||
            pt.z - expected_ground < config_.max_dip_down)
        {
          if (pt_dist_from_sensor <= config_.visible_ground_min)
          {
            obscured_view = true;
          }
          if (cnt++ % (config_.depth_subsampling / config_.depth_image_stride / config_.depth_image_stride) == 0)
          {
            points.emplace_back(pt.x, pt.y, pt.z);
          }
        }
      }
      else if (horizontal.at<uint8_t>(v, u))
      {
        if (pt.z - robot_xyz.z() > config_.max_bump_height &&
            pt_dist_from_sensor <= config_.visible_ground_min)
        {
          obscured_view = true;
        }

        if (pt_dist_from_sensor >= config_.visible_ground_min &&
            pt_dist_from_sensor <= config_.visible_ground_max)
        {
          ground_visible = true;
        }

        expected_ground = pt.z;
      }
    }

    // We react to missing ground only when the robot is looking sufficiently
    // horizontally or downwards. Otherwise, the robot may be climbing up a
    // ridge and does not even have a chance to see the ground.
    if (config_.synthetic_obstacle_distance > 0 &&
        pt_down_near.z() - robot_xyz.z() <= 0) {
      // If
      // a) We haven't seen any ground in this direction;
      // b) The view is not obstructed by an obstacle;
      // c) The robot is not pitched/rolled in a way that seeing
      //    ground in this direction is impossible;
      // then we should be worried.
      if (!ground_visible && !obscured_view)
      {
        points.push_back(pt_down_near);
      }
    }
  }

  InsertObservation(std::move(points), camera_frame_id,
                    config_.max_depth_observations);
}

bool Traversability::NearBreadcrumb(const tf::Vector3& pt) const
{
  for (const auto &b : breadcrumbs_)
  {
    if (std::hypot(pt.x() - b.x(), pt.y() - b.y()) <
          config_.breadcrumb_radius &&
        std::abs(pt.z() - b.z()) < config_.breadcrumb_height / 2)
    {
      return true;
    }
  }
  return false;
}

float median(std::vector<float> values)
{
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

void Traversability::OnScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::optional<tf::StampedTransform> sensor_pose =
    GetTransform(config_.world_frame_id, msg->header.frame_id, msg->header.stamp);
  if (!sensor_pose) return;

  std::optional<tf::StampedTransform> robot_pose =
    GetTransform(config_.world_frame_id, config_.robot_frame_id, msg->header.stamp);
  if (!robot_pose) return;
  const auto& robot_xyz = robot_pose->getOrigin();

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

    const float slope = std::atan2(
        pt.z() - robot_xyz.z(),
        std::hypot(pt.x() - robot_xyz.x(),
                   pt.y() - robot_xyz.y()));

    if (std::abs(slope) > config_.max_slope &&
        cnt++ % config_.lidar_subsampling == 0)
    {
      points.push_back(std::move(pt));
    }
  }

  InsertObservation(std::move(points),
                    msg->header.frame_id,
                    config_.max_scan_observations);
}

void Traversability::OnBreadcrumb(const std_msgs::Empty::ConstPtr& msg __attribute__((unused)))
{
  std::optional<tf::StampedTransform> robot_pose =
    GetTransform(config_.world_frame_id, config_.robot_frame_id, ros::Time(0));
  if (!robot_pose) return;
  // Ideally, we would compensate for the fact that a breadcrumb is dropped at
  // the rear end of the robot and not in the center.
  const auto& robot_xyz = robot_pose->getOrigin();
  breadcrumbs_.push_back(robot_xyz);
}

void Traversability::OnTimer(const ros::TimerEvent& event)
{
  if (points_publisher_.getNumSubscribers() == 0 &&
      scan_publisher_.getNumSubscribers() == 0)
  {
    return;  // Nobody cares. Do not bother with all the calculation.
  }

  const ros::Time latest(0);
  const auto to_local = GetTransform(config_.robot_frame_id, config_.world_frame_id, latest);
  if (!to_local) return;

  // Convert all collected pointclouds to be relative to the current robot pose.
  std::vector<tf::Vector3> local_pts;
  for (const auto single_source_observations : observations_)
  {
    const auto& observations = single_source_observations.second;
    for (const auto& observation : observations)
    {
      for (const auto& pt : observation)
      {
        const auto local_pt = *to_local * pt;
        if (local_pt.z() < config_.max_relative_z &&
	    !NearBreadcrumb(pt))
        {
          local_pts.push_back(local_pt);
        }
      }
    }
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
    local_map.header.frame_id = config_.robot_frame_id;
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
    local_scan.header.frame_id = config_.robot_frame_id;
    local_scan.header.stamp = event.current_real;
    local_scan.angle_min = -M_PI;
    local_scan.angle_max = 179 * M_PI / 180;
    local_scan.angle_increment = M_PI / 180;
    local_scan.range_min = config_.min_map_scan_range;
    local_scan.range_max = config_.map_range;
    local_scan.ranges.resize(
        1 + (local_scan.angle_max - local_scan.angle_min) /
              local_scan.angle_increment);
    for (auto& range : local_scan.ranges)
    {
      range = std::numeric_limits<float>::infinity();
    }

    for (const auto& pt : nearby_pts)
    {
      const float direction = std::atan2(pt.y(), pt.x());
      const size_t bucket = static_cast<size_t>(
          std::round((direction - local_scan.angle_min) /
                     local_scan.angle_increment)) %
          local_scan.ranges.size();
      const float distance = std::hypot(pt.x(), pt.y());
      auto& range = local_scan.ranges[bucket];
      range = std::min(range, distance);
    }
    for (auto& range : local_scan.ranges)
    {
      if (std::isinf(range))
      {
        range = 0;  // Invalid reading.
      }
    }

    scan_publisher_.publish(local_scan);
  }
}

}  // namespace osgar

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability");
  cv::setNumThreads(1);
  osgar::Traversability traversability;
  if (!traversability.Init())
  {
    return -1;
  }
  ros::spin();
}
