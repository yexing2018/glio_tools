#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "data_preprocess/msg/gps_heading_data.hpp"

namespace
{

int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

std::string utm_band(double latitude)
{
  static const std::string bands = "CDEFGHJKLMNPQRSTUVWXX";
  const auto index = static_cast<std::size_t>(std::floor((latitude + 80.0) / 8.0));
  return std::string(1, bands[std::min(index, bands.size() - 1)]);
}

}  // namespace

class GpsHeadingPreprocessNode : public rclcpp::Node
{
public:
  GpsHeadingPreprocessNode()
  : Node("gps_heading_preprocess")
  {
    declare_parameter<std::string>("heading_topic", "/imu_orientation");
    declare_parameter<std::string>("gps_topic", "/fix");
    declare_parameter<std::string>("output_topic", "/gps_heading_preprocessed");
    declare_parameter<std::string>("output_frame_id", "utm");
    declare_parameter<double>("sync_tolerance_sec", 0.05);
    declare_parameter<int>("queue_size", 200);
    declare_parameter<bool>("require_valid_fix", true);

    const auto heading_topic = get_parameter("heading_topic").as_string();
    const auto gps_topic = get_parameter("gps_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    sync_tolerance_ns_ = static_cast<int64_t>(
      std::max(0.0, get_parameter("sync_tolerance_sec").as_double()) * 1e9);
    queue_size_ = static_cast<std::size_t>(
      std::max<int64_t>(2, get_parameter("queue_size").as_int()));
    require_valid_fix_ = get_parameter("require_valid_fix").as_bool();

    publisher_ = create_publisher<data_preprocess::msg::GpsHeadingData>(output_topic, 10);
    heading_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      heading_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsHeadingPreprocessNode::heading_callback, this, std::placeholders::_1));
    gps_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsHeadingPreprocessNode::gps_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Synchronizing %s and %s with %.3f s tolerance; publishing %s",
      heading_topic.c_str(), gps_topic.c_str(),
      static_cast<double>(sync_tolerance_ns_) * 1e-9, output_topic.c_str());
  }

private:
  using HeadingInput = sensor_msgs::msg::Imu;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using MatchedPair = std::pair<HeadingInput::ConstSharedPtr, NavSatFix::ConstSharedPtr>;

  template<typename MessageT>
  void insert_by_stamp(
    std::deque<std::shared_ptr<const MessageT>> & queue,
    const std::shared_ptr<const MessageT> & message)
  {
    const int64_t timestamp = stamp_nanoseconds(message->header.stamp);
    const auto position = std::upper_bound(
      queue.begin(), queue.end(), timestamp,
      [](int64_t value, const std::shared_ptr<const MessageT> & queued) {
        return value < stamp_nanoseconds(queued->header.stamp);
      });
    queue.insert(position, message);
    while (queue.size() > queue_size_) {
      queue.pop_front();
    }
  }

  void heading_callback(const HeadingInput::ConstSharedPtr message)
  {
    std::vector<MatchedPair> matches;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      insert_by_stamp(heading_queue_, message);
      matches = match_messages_locked();
    }
    publish_matches(matches);
  }

  void gps_callback(const NavSatFix::ConstSharedPtr message)
  {
    if (!std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
      !std::isfinite(message->altitude) || message->latitude < -80.0 ||
      message->latitude > 84.0 || message->longitude < -180.0 ||
      message->longitude > 180.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Discarding invalid or non-UTM GPS coordinate");
      return;
    }
    if (require_valid_fix_ && message->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Discarding GPS message without a valid fix");
      return;
    }

    std::vector<MatchedPair> matches;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      insert_by_stamp(gps_queue_, message);
      matches = match_messages_locked();
    }
    publish_matches(matches);
  }

  std::vector<MatchedPair> match_messages_locked()
  {
    std::vector<MatchedPair> matches;

    while (!gps_queue_.empty() && !heading_queue_.empty()) {
      const int64_t gps_time = stamp_nanoseconds(gps_queue_.front()->header.stamp);

      while (!heading_queue_.empty() &&
        stamp_nanoseconds(heading_queue_.front()->header.stamp) < gps_time - sync_tolerance_ns_)
      {
        heading_queue_.pop_front();
      }
      if (heading_queue_.empty()) {
        break;
      }

      const auto first_after = std::lower_bound(
        heading_queue_.begin(), heading_queue_.end(), gps_time,
        [](const HeadingInput::ConstSharedPtr & heading, int64_t value) {
          return stamp_nanoseconds(heading->header.stamp) < value;
        });

      // Wait until a heading sample reaches or passes the GPS timestamp. This lets
      // us choose the nearest sample on either side instead of pairing too early.
      if (first_after == heading_queue_.end()) {
        break;
      }

      auto best = first_after;
      if (first_after != heading_queue_.begin()) {
        const auto before = std::prev(first_after);
        const int64_t before_error = std::llabs(
          stamp_nanoseconds((*before)->header.stamp) - gps_time);
        const int64_t after_error = std::llabs(
          stamp_nanoseconds((*first_after)->header.stamp) - gps_time);
        if (before_error <= after_error) {
          best = before;
        }
      }

      const int64_t error = std::llabs(stamp_nanoseconds((*best)->header.stamp) - gps_time);
      if (error <= sync_tolerance_ns_) {
        matches.emplace_back(*best, gps_queue_.front());
        heading_queue_.erase(heading_queue_.begin(), std::next(best));
        gps_queue_.pop_front();
      } else if (stamp_nanoseconds((*first_after)->header.stamp) >
        gps_time + sync_tolerance_ns_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping GPS message: no heading sample within synchronization tolerance");
        gps_queue_.pop_front();
      } else {
        break;
      }
    }

    return matches;
  }

  void publish_matches(const std::vector<MatchedPair> & matches)
  {
    for (const auto & match : matches) {
      publish_match(match.first, match.second);
    }
  }

  void publish_match(
    const HeadingInput::ConstSharedPtr & heading, const NavSatFix::ConstSharedPtr & gps)
  {
    const auto & input_q = heading->orientation;
    const double norm = std::sqrt(
      input_q.x * input_q.x + input_q.y * input_q.y +
      input_q.z * input_q.z + input_q.w * input_q.w);
    if (!std::isfinite(norm) || norm < std::numeric_limits<double>::epsilon()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Discarding invalid heading quaternion");
      return;
    }

    const double x = input_q.x / norm;
    const double y = input_q.y / norm;
    const double z = input_q.z / norm;
    const double w = input_q.w / norm;

    const double sin_roll_cos_pitch = 2.0 * (w * x + y * z);
    const double cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

    const double sin_pitch = 2.0 * (w * y - z * x);
    const double pitch = std::abs(sin_pitch) >= 1.0 ?
      std::copysign(M_PI / 2.0, sin_pitch) : std::asin(sin_pitch);

    const double sin_yaw_cos_pitch = 2.0 * (w * z + x * y);
    const double cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

    int zone = 0;
    bool north = true;
    double easting = 0.0;
    double northing = 0.0;
    try {
      GeographicLib::UTMUPS::Forward(
        gps->latitude, gps->longitude, zone, north, easting, northing);
    } catch (const GeographicLib::GeographicErr & error) {
      RCLCPP_WARN(get_logger(), "UTM conversion failed: %s", error.what());
      return;
    }

    data_preprocess::msg::GpsHeadingData output;
    output.header = gps->header;
    if (!output_frame_id_.empty()) {
      output.header.frame_id = output_frame_id_;
    }
    output.heading_stamp = heading->header.stamp;
    output.gps_stamp = gps->header.stamp;
    output.w = w;
    output.x = x;
    output.y = y;
    output.z = z;
    output.roll = roll;
    output.yaw = yaw;
    output.pitch = pitch;
    output.latitude = gps->latitude;
    output.longitude = gps->longitude;
    output.altitude = gps->altitude;
    output.utm_zone = zone;
    output.utm_band = utm_band(gps->latitude);
    output.utm_north = north;
    output.utm_easting = easting;
    output.utm_northing = northing;
    output.variance_x = gps->position_covariance[0];
    output.variance_y = gps->position_covariance[4];
    output.variance_z = gps->position_covariance[8];
    output.position_covariance_type = gps->position_covariance_type;
    output.heading_gps_time_offset = static_cast<double>(
      stamp_nanoseconds(heading->header.stamp) - stamp_nanoseconds(gps->header.stamp)) * 1e-9;
    publisher_->publish(output);
  }

  std::mutex queue_mutex_;
  std::deque<HeadingInput::ConstSharedPtr> heading_queue_;
  std::deque<NavSatFix::ConstSharedPtr> gps_queue_;
  int64_t sync_tolerance_ns_{50000000};
  std::size_t queue_size_{200};
  bool require_valid_fix_{true};
  std::string output_frame_id_{"utm"};

  rclcpp::Subscription<HeadingInput>::SharedPtr heading_subscription_;
  rclcpp::Subscription<NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Publisher<data_preprocess::msg::GpsHeadingData>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsHeadingPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
