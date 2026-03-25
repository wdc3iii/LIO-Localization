#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

struct Waypoint {
  double x, y, z, roll, pitch, yaw;
};

class WaypointDropperNode : public rclcpp::Node {
public:
  WaypointDropperNode() : Node("waypoint_dropper") {
    // Parameters
    std::string pcd_file_name =
        declare_parameter<std::string>("waypoint_dropper.pcd_file_name", "");
    double map_voxel_size =
        declare_parameter<double>("waypoint_dropper.map_viz_voxel_size", 0.25);
    std::string map_frame =
        declare_parameter<std::string>("frames.map_frame", "map");
    body_frame_ =
        declare_parameter<std::string>("frames.body_frame", "body");
    output_dir_ =
        declare_parameter<std::string>("output_dir",
            std::string(ROOT_DIR) + "outputs");
    ground_search_radius_x_ =
        declare_parameter<double>("ground_estimation.ground_search_radius_x", 5.0);
    ground_search_radius_y_ =
        declare_parameter<double>("ground_estimation.ground_search_radius_y", 5.0);
    ground_percentile_ =
        declare_parameter<double>("ground_estimation.ground_percentile", 0.05);

    // Load map
    if (pcd_file_name.empty()) {
      RCLCPP_FATAL(get_logger(),
          "No PCD file specified (waypoint_dropper.pcd_file_name)");
      throw std::runtime_error("No PCD file specified");
    }

    std::string pcd_path = std::string(ROOT_DIR) + "pcd/" + pcd_file_name;
    map_cloud_ = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *map_cloud_) == -1) {
      RCLCPP_FATAL(get_logger(), "Failed to load PCD: %s",
                   pcd_path.c_str());
      throw std::runtime_error("Failed to load PCD: " + pcd_path);
    }
    RCLCPP_INFO(get_logger(), "Loaded map with %zu points from %s",
                map_cloud_->size(), pcd_path.c_str());

    // Publish downsampled map (latched)
    auto qos = rclcpp::QoS(1).transient_local();
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", qos);

    auto downsampled = std::make_shared<PointCloud>();
    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(map_cloud_);
    voxel.setLeafSize(map_voxel_size, map_voxel_size, map_voxel_size);
    voxel.filter(*downsampled);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*downsampled, msg);
    msg.header.frame_id = map_frame;
    msg.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    pub_map_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "Published downsampled map (%zu -> %zu points, voxel %.2fm)",
                map_cloud_->size(), downsampled->size(), map_voxel_size);

    // Subscribe to /initialpose (RViz 2D Pose Estimate) -> poses list
    sub_initialpose_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", rclcpp::QoS(1),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
              Waypoint wp = extract_waypoint(
                  msg->pose.pose.position.x, msg->pose.pose.position.y,
                  msg->pose.pose.orientation);
              poses_.push_back(wp);
              RCLCPP_INFO(get_logger(),
                  "Pose #%zu: x=%.2f y=%.2f z=%.2f yaw=%.2f",
                  poses_.size(), wp.x, wp.y, wp.z, wp.yaw);
            });

    // Subscribe to /goal_pose (RViz 2D Goal Pose) -> goals list
    sub_goalpose_ =
        create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rclcpp::QoS(1),
            [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
              Waypoint wp = extract_waypoint(
                  msg->pose.position.x, msg->pose.position.y,
                  msg->pose.orientation);
              goals_.push_back(wp);
              RCLCPP_INFO(get_logger(),
                  "Goal #%zu: x=%.2f y=%.2f z=%.2f yaw=%.2f",
                  goals_.size(), wp.x, wp.y, wp.z, wp.yaw);
            });

    RCLCPP_INFO(get_logger(),
        "Waypoint dropper ready. Use RViz '2D Pose Estimate' for poses and "
        "'2D Goal Pose' for goals. Press Ctrl+C to save and exit.");
  }

  void save_waypoints() {
    // Print to console
    RCLCPP_INFO(get_logger(), "=== Poses (%zu) ===", poses_.size());
    for (size_t i = 0; i < poses_.size(); ++i) {
      const auto& wp = poses_[i];
      RCLCPP_INFO(get_logger(),
          "  [%zu] x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f",
          i, wp.x, wp.y, wp.z, wp.roll, wp.pitch, wp.yaw);
    }
    RCLCPP_INFO(get_logger(), "=== Goals (%zu) ===", goals_.size());
    for (size_t i = 0; i < goals_.size(); ++i) {
      const auto& wp = goals_[i];
      RCLCPP_INFO(get_logger(),
          "  [%zu] x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f",
          i, wp.x, wp.y, wp.z, wp.roll, wp.pitch, wp.yaw);
    }

    if (poses_.empty() && goals_.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoints to save.");
      return;
    }

    // Build YAML
    YAML::Emitter out;
    out << YAML::BeginMap;

    auto emit_list = [&](const std::string& name,
                         const std::vector<Waypoint>& wps) {
      out << YAML::Key << name << YAML::Value << YAML::BeginSeq;
      for (const auto& wp : wps) {
        out << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << wp.x;
        out << YAML::Key << "y" << YAML::Value << wp.y;
        out << YAML::Key << "z" << YAML::Value << wp.z;
        out << YAML::Key << "roll" << YAML::Value << wp.roll;
        out << YAML::Key << "pitch" << YAML::Value << wp.pitch;
        out << YAML::Key << "yaw" << YAML::Value << wp.yaw;
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    };

    emit_list("poses", poses_);
    emit_list("goals", goals_);
    out << YAML::EndMap;

    // Ensure output directory exists
    if (output_dir_.empty()) {
      RCLCPP_ERROR(get_logger(), "No output directory specified.");
      return;
    }
    std::filesystem::create_directories(output_dir_);

    // Generate timestamped filename
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char time_buf[64];
    std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", &tm);

    std::string filepath =
        output_dir_ + "/waypoints_" + time_buf + ".yaml";

    std::ofstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s for writing",
                   filepath.c_str());
      return;
    }
    file << out.c_str() << "\n";
    file.close();

    RCLCPP_INFO(get_logger(), "Waypoints saved to %s", filepath.c_str());
  }

private:
  Waypoint extract_waypoint(double x, double y,
                            const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll_unused, pitch_unused, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll_unused, pitch_unused, yaw);

    double ground_z = compute_ground_height(x, y);

    return {x, y, ground_z, 0.0, 0.0, yaw};
  }

  double compute_ground_height(double x, double y) const {
    std::vector<float> z_values;
    z_values.reserve(1000);

    for (const auto& pt : map_cloud_->points) {
      if (std::abs(pt.x - static_cast<float>(x)) < ground_search_radius_x_ &&
          std::abs(pt.y - static_cast<float>(y)) < ground_search_radius_y_) {
        z_values.push_back(pt.z);
      }
    }

    if (z_values.empty()) {
      RCLCPP_WARN(get_logger(),
          "No map points near (%.2f, %.2f) within (%.1f x %.1f). Using z=0.",
          x, y, ground_search_radius_x_, ground_search_radius_y_);
      return 0.0;
    }

    size_t idx =
        static_cast<size_t>(ground_percentile_ * (z_values.size() - 1));
    std::nth_element(z_values.begin(), z_values.begin() + idx, z_values.end());
    double ground_z = z_values[idx];

    RCLCPP_INFO(get_logger(),
        "Ground height at (%.2f, %.2f): %.3f (%zu points in search region)",
        x, y, ground_z, z_values.size());
    return ground_z;
  }

  PointCloud::Ptr map_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_initialpose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_goalpose_;

  std::vector<Waypoint> poses_;
  std::vector<Waypoint> goals_;

  std::string output_dir_;
  std::string body_frame_;
  double ground_search_radius_x_;
  double ground_search_radius_y_;
  double ground_percentile_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointDropperNode>();

  rclcpp::spin(node);

  // Save waypoints after spin returns (triggered by Ctrl+C / SIGINT)
  node->save_waypoints();

  rclcpp::shutdown();
  return 0;
}
