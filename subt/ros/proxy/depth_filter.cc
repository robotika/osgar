#include <cassert>
#include <limits>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace osgar
{
class DepthFilter
{
  public:
    DepthFilter() : ros_handle_("~") {}

    bool Init();

  private:
    ros::NodeHandle ros_handle_;
    ros::Subscriber input_handler_;
    ros::Publisher output_publisher_;

    struct {
      int k;
      float min_support_fraction;
      float max_support_distance;
      float max_depth;
    } config_;

    void OnDepth(const sensor_msgs::Image::ConstPtr& msg);
};

bool DepthFilter::Init()
{
  ros_handle_.param("k", config_.k, 2);
  ros_handle_.param("min_support_fraction", config_.min_support_fraction, 2.0f / 3.0f);
  ros_handle_.param("max_support_distance", config_.max_support_distance, 0.6f);
  ros_handle_.param("max_depth", config_.max_depth, 10.0f);

  input_handler_ = ros_handle_.subscribe("input", 10, &DepthFilter::OnDepth, this);
  output_publisher_ = ros_handle_.advertise<sensor_msgs::Image>("output", 10);

  return true;
}

void DepthFilter::OnDepth(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr input_img_ptr;
  try
  {
    input_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }

  const cv::Mat& input = input_img_ptr->image;
  cv_bridge::CvImage output;
  output.encoding = "32FC1";
  output.image = cv::Mat(input.rows, input.cols, CV_32FC1);

  int num_filtered = 0;
  for (int u = 0; u < input.rows; ++u)
  {
    for (int v = 0; v < input.cols; ++v)
    {
      const float inp = input.at<float>(u, v);
      float& outp = output.image.at<float>(u, v);

      if (inp == 0 ||
          inp >= config_.max_depth ||
          std::isnan(inp))
      {
        outp = std::numeric_limits<float>::infinity();
        continue;
      }

      int support = -1;  // Compensating for self-support.
      int count = 0;
      for (int i = std::max(0, u - config_.k); i <= std::min(u + config_.k, input.rows - 1); ++i)
      {
        for (int j = std::max(0, v - config_.k); j <= std::min(v + config_.k, input.cols - 1); ++j)
        {
          ++count;
          const float neighbor = input.at<float>(i, j);
          if (neighbor == 0 ||
              neighbor >= config_.max_depth ||
              std::isnan(neighbor))
          {
            continue;
          }

          if (std::abs(neighbor - inp) <= config_.max_support_distance)
          {
            ++support;
          }
        }
      }

      if (support / static_cast<float>(count) >= config_.min_support_fraction)
      {
        outp = inp;
      }
      else
      {
        outp = std::numeric_limits<float>::infinity();
        ++num_filtered;
      }
    }
  }

  ROS_DEBUG("filtered: %d", num_filtered);
  auto out = output.toImageMsg();
  out->header = msg->header;
  output_publisher_.publish(out);
}

}  // namespace osgar

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_filter");
  osgar::DepthFilter depth_filter;
  if (!depth_filter.Init())
  {
    return -1;
  }
  ros::spin();
}
