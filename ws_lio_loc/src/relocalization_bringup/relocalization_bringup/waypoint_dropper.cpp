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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

struct Waypoint {
  uint64_t id;
  double x, y, z, roll, pitch, yaw;
};

class WaypointDropperNode : public rclcpp::Node {
public:
  WaypointDropperNode() : Node("waypoint_dropper"), next_id_(0), insert_index_(-1) {
    // Parameters
    std::string pcd_file_name =
        declare_parameter<std::string>("waypoint_dropper.pcd_file_name", "");
    double map_voxel_size =
        declare_parameter<double>("waypoint_dropper.map_viz_voxel_size", 0.25);
    map_frame_ =
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

    std::string input_file =
        declare_parameter<std::string>("waypoint_dropper.input_waypoint_file", "");
    insert_index_ =
        declare_parameter<int>("waypoint_dropper.insert_index", -1);

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
    msg.header.frame_id = map_frame_;
    msg.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    pub_map_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "Published downsampled map (%zu -> %zu points, voxel %.2fm)",
                map_cloud_->size(), downsampled->size(), map_voxel_size);

    // Interactive marker server
    marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "waypoint_markers", this);

    // Setup menu handler
    menu_insert_before_ = menu_handler_.insert("Insert Before",
        std::bind(&WaypointDropperNode::menu_callback, this,
                  std::placeholders::_1));
    menu_insert_after_ = menu_handler_.insert("Insert After",
        std::bind(&WaypointDropperNode::menu_callback, this,
                  std::placeholders::_1));
    menu_delete_ = menu_handler_.insert("Delete",
        std::bind(&WaypointDropperNode::menu_callback, this,
                  std::placeholders::_1));

    // Load waypoints from file if specified (relative paths resolve from outputs dir)
    if (!input_file.empty()) {
      std::string load_path = input_file;
      if (load_path.front() != '/') {
        load_path = output_dir_ + "/" + load_path;
      }
      load_waypoints(load_path);
      rebuild_markers();
    }

    // Subscribe to /initialpose (RViz 2D Pose Estimate) -> waypoints list
    sub_initialpose_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", rclcpp::QoS(1),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
              Waypoint wp = extract_waypoint(
                  msg->pose.pose.position.x, msg->pose.pose.position.y,
                  msg->pose.pose.orientation);

              // Read insert_index from parameter (allows runtime changes)
              insert_index_ = get_parameter("waypoint_dropper.insert_index").as_int();

              size_t pos;
              if (insert_index_ < 0 ||
                  insert_index_ >= static_cast<int>(waypoints_.size())) {
                pos = waypoints_.size();
                waypoints_.push_back(wp);
              } else {
                pos = static_cast<size_t>(insert_index_);
                waypoints_.insert(waypoints_.begin() + insert_index_, wp);
              }

              // Auto-advance: move insert_index past the just-inserted waypoint
              if (insert_index_ >= 0) {
                insert_index_++;
                set_parameter(rclcpp::Parameter("waypoint_dropper.insert_index", insert_index_));
              }

              RCLCPP_INFO(get_logger(),
                  "Waypoint #%zu (id=%lu): x=%.2f y=%.2f z=%.2f yaw=%.2f "
                  "(inserted at index %zu, next insert_index=%d)",
                  waypoints_.size(), wp.id, wp.x, wp.y, wp.z, wp.yaw,
                  pos, insert_index_);

              rebuild_markers();
            });

    RCLCPP_INFO(get_logger(),
        "Waypoint dropper ready. Use RViz '2D Pose Estimate' to drop waypoints. "
        "Left-click a marker to set insertion point. Right-click for menu "
        "(Insert Before/After, Delete). Press Ctrl+C to save and exit.");
    RCLCPP_INFO(get_logger(),
        "Set insert index: ros2 param set /waypoint_dropper "
        "waypoint_dropper.insert_index <N>  (-1 = append)");
  }

  void save_waypoints() {
    // Print to console
    RCLCPP_INFO(get_logger(), "=== Waypoints (%zu) ===", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];
      RCLCPP_INFO(get_logger(),
          "  [%zu] x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f",
          i, wp.x, wp.y, wp.z, wp.roll, wp.pitch, wp.yaw);
    }

    if (waypoints_.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoints to save.");
      return;
    }

    // Build YAML
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
    for (const auto& wp : waypoints_) {
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
    out << YAML::EndMap;

    // Ensure output directory exists
    if (output_dir_.empty()) {
      RCLCPP_ERROR(get_logger(), "No output directory specified.");
      return;
    }
    std::filesystem::create_directories(output_dir_);

    // Generate timestamped filename
    auto now_tp = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now_tp);
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
  // --- Waypoint extraction ---

  Waypoint extract_waypoint(double x, double y,
                            const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll_unused, pitch_unused, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll_unused, pitch_unused, yaw);

    double ground_z = compute_ground_height(x, y);

    return {next_id_++, x, y, ground_z, 0.0, 0.0, yaw};
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

  // --- YAML loading ---

  void load_waypoints(const std::string& filepath) {
    try {
      YAML::Node config = YAML::LoadFile(filepath);

      auto load_list = [this](const YAML::Node& node) {
        if (!node || !node.IsSequence()) return;
        for (const auto& entry : node) {
          Waypoint wp;
          wp.id = next_id_++;
          wp.x = entry["x"].as<double>();
          wp.y = entry["y"].as<double>();
          wp.z = entry["z"].as<double>();
          wp.roll = entry["roll"].as<double>();
          wp.pitch = entry["pitch"].as<double>();
          wp.yaw = entry["yaw"].as<double>();
          waypoints_.push_back(wp);
        }
      };

      // New format
      if (config["waypoints"]) {
        load_list(config["waypoints"]);
      }
      // Legacy format
      if (config["poses"]) {
        load_list(config["poses"]);
      }
      if (config["goals"]) {
        load_list(config["goals"]);
      }

      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s",
                  waypoints_.size(), filepath.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load waypoints from %s: %s",
                   filepath.c_str(), e.what());
    }
  }

  // --- Interactive marker visualization ---

  void rebuild_markers() {
    marker_server_->clear();

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];

      // Color gradient: green (first) -> red (last)
      double t = waypoints_.size() > 1
          ? static_cast<double>(i) / (waypoints_.size() - 1)
          : 0.0;
      float r = static_cast<float>(t);
      float g = static_cast<float>(1.0 - t);
      float b = 0.0f;

      // Create interactive marker
      visualization_msgs::msg::InteractiveMarker int_marker;
      int_marker.header.frame_id = map_frame_;
      int_marker.header.stamp = now();
      int_marker.name = "wp_" + std::to_string(wp.id);
      int_marker.description = "";
      int_marker.pose.position.x = wp.x;
      int_marker.pose.position.y = wp.y;
      int_marker.pose.position.z = wp.z;
      // Set orientation from yaw for the marker itself
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, wp.yaw);
      int_marker.pose.orientation.x = q.x();
      int_marker.pose.orientation.y = q.y();
      int_marker.pose.orientation.z = q.z();
      int_marker.pose.orientation.w = q.w();
      int_marker.scale = 1.0;

      // Button control (enables click + right-click menu)
      visualization_msgs::msg::InteractiveMarkerControl button_control;
      button_control.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
      button_control.always_visible = true;

      // Sphere marker
      visualization_msgs::msg::Marker sphere;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.scale.x = 0.4;
      sphere.scale.y = 0.4;
      sphere.scale.z = 0.4;
      sphere.color.r = r;
      sphere.color.g = g;
      sphere.color.b = b;
      sphere.color.a = 0.85;
      // Offset sphere up so it's visible above ground
      sphere.pose.position.z = 0.5;
      button_control.markers.push_back(sphere);

      // Arrow marker showing yaw direction
      visualization_msgs::msg::Marker arrow;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.scale.x = 0.8;  // length
      arrow.scale.y = 0.12; // width
      arrow.scale.z = 0.12; // height
      arrow.color.r = r;
      arrow.color.g = g;
      arrow.color.b = b;
      arrow.color.a = 0.9;
      // Arrow at same height as sphere, orientation follows the interactive marker's yaw
      arrow.pose.position.z = 0.5;
      button_control.markers.push_back(arrow);

      // Text label
      visualization_msgs::msg::Marker text;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.text = std::to_string(i);
      text.scale.z = 0.5;  // text height
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.pose.position.z = 1.2;  // above the sphere
      button_control.markers.push_back(text);

      int_marker.controls.push_back(button_control);

      marker_server_->insert(int_marker,
          std::bind(&WaypointDropperNode::marker_feedback_cb, this,
                    std::placeholders::_1));
      menu_handler_.apply(*marker_server_, int_marker.name);
    }

    marker_server_->applyChanges();
  }

  // --- Marker feedback (left-click) ---

  void marker_feedback_cb(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
    if (feedback->event_type !=
        visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK) {
      return;
    }

    // Find which waypoint was clicked
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      if (("wp_" + std::to_string(waypoints_[i].id)) == feedback->marker_name) {
        insert_index_ = static_cast<int>(i) + 1;  // insert after clicked waypoint
        set_parameter(rclcpp::Parameter("waypoint_dropper.insert_index", insert_index_));
        RCLCPP_INFO(get_logger(),
            "Clicked waypoint %zu (id=%lu) -> insert_index set to %d (insert after)",
            i, waypoints_[i].id, insert_index_);
        return;
      }
    }
  }

  // --- Menu callbacks ---

  void menu_callback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
    // Find which waypoint this menu was triggered on
    size_t wp_idx = 0;
    bool found = false;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      if (("wp_" + std::to_string(waypoints_[i].id)) == feedback->marker_name) {
        wp_idx = i;
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_WARN(get_logger(), "Menu callback: marker '%s' not found",
                  feedback->marker_name.c_str());
      return;
    }

    if (feedback->menu_entry_id == menu_insert_before_) {
      insert_index_ = static_cast<int>(wp_idx);
      set_parameter(rclcpp::Parameter("waypoint_dropper.insert_index", insert_index_));
      RCLCPP_INFO(get_logger(),
          "Insert Before waypoint %zu -> insert_index set to %d",
          wp_idx, insert_index_);
    } else if (feedback->menu_entry_id == menu_insert_after_) {
      insert_index_ = static_cast<int>(wp_idx) + 1;
      set_parameter(rclcpp::Parameter("waypoint_dropper.insert_index", insert_index_));
      RCLCPP_INFO(get_logger(),
          "Insert After waypoint %zu -> insert_index set to %d",
          wp_idx, insert_index_);
    } else if (feedback->menu_entry_id == menu_delete_) {
      RCLCPP_INFO(get_logger(),
          "Deleted waypoint %zu (id=%lu, x=%.2f y=%.2f)",
          wp_idx, waypoints_[wp_idx].id,
          waypoints_[wp_idx].x, waypoints_[wp_idx].y);
      waypoints_.erase(waypoints_.begin() + wp_idx);

      // Clamp insert_index if it's now out of range
      if (insert_index_ >= static_cast<int>(waypoints_.size())) {
        insert_index_ = static_cast<int>(waypoints_.size());
        set_parameter(rclcpp::Parameter("waypoint_dropper.insert_index", insert_index_));
      }

      rebuild_markers();
    }
  }

  // --- Helpers ---

  size_t find_waypoint_index(const std::string& marker_name) const {
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      if (("wp_" + std::to_string(waypoints_[i].id)) == marker_name) {
        return i;
      }
    }
    return waypoints_.size(); // not found
  }

  // --- Members ---

  PointCloud::Ptr map_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_initialpose_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle menu_insert_before_;
  interactive_markers::MenuHandler::EntryHandle menu_insert_after_;
  interactive_markers::MenuHandler::EntryHandle menu_delete_;

  std::vector<Waypoint> waypoints_;
  uint64_t next_id_;
  int insert_index_;

  std::string map_frame_;
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
