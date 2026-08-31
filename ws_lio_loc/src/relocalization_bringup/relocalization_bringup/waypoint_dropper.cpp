// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 California Institute of Technology and Will Compton

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include "map_levels.h"
#include "pcd_path.h"

#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

struct Waypoint {
  uint64_t id;
  double x, y, z, roll, pitch, yaw;
  std::string level;  // Name of the level this waypoint sits on ("" = none)
};

class WaypointDropperNode : public rclcpp::Node {
public:
  WaypointDropperNode() : Node("waypoint_dropper"), next_id_(0), insert_index_(-1) {
    // Parameters
    std::string pcd_file_name =
        declare_parameter<std::string>("waypoint_dropper.pcd_file_name", "");
    map_voxel_size_ =
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

    // Levels ("floors"): see map_levels.h for what a level is and why.
    levels_enable_ =
        declare_parameter<bool>("levels.enable", true);
    level_band_below_ =
        declare_parameter<double>("levels.band_below", 0.25);
    level_band_above_ =
        declare_parameter<double>("levels.band_above", 2.5);
    level_merge_tolerance_ =
        declare_parameter<double>("levels.merge_tolerance", 1.0);
    level_auto_detect_ =
        declare_parameter<bool>("levels.auto_detect", true);
    level_detect_min_fraction_ =
        declare_parameter<double>("levels.detect_min_fraction", 0.02);
    hide_other_levels_ =
        declare_parameter<bool>("levels.hide_other_levels", false);
    auto configured_level_names = declare_parameter<std::vector<std::string>>(
        "levels.names", std::vector<std::string>{});
    auto configured_level_heights = declare_parameter<std::vector<double>>(
        "levels.heights", std::vector<double>{});
    std::string startup_level =
        declare_parameter<std::string>("levels.active", "");
    std::string level_point_topic = declare_parameter<std::string>(
        "topics.level_point_topic", "/clicked_point");

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

    std::string pcd_file_path;
    try {
      pcd_file_path = pcd_path::resolve_pcd_path(pcd_file_name, ROOT_DIR);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }
    map_cloud_ = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile<PointType>(pcd_file_path, *map_cloud_) == -1) {
      RCLCPP_FATAL(get_logger(), "Failed to load PCD: %s",
                   pcd_file_path.c_str());
      throw std::runtime_error("Failed to load PCD: " + pcd_file_path);
    }
    RCLCPP_INFO(get_logger(), "Loaded map with %zu points from %s",
                map_cloud_->size(), pcd_file_path.c_str());

    // Publish downsampled map (latched)
    auto qos = rclcpp::QoS(1).transient_local();
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", qos);
    pub_level_map_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud_level", qos);

    auto downsampled = std::make_shared<PointCloud>();
    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(map_cloud_);
    voxel.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
    voxel.filter(*downsampled);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*downsampled, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    pub_map_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "Published downsampled map (%zu -> %zu points, voxel %.2fm)",
                map_cloud_->size(), downsampled->size(), map_voxel_size_);

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
    menu_snap_level_ = menu_handler_.insert("Snap to Active Level",
        std::bind(&WaypointDropperNode::menu_callback, this,
                  std::placeholders::_1));
    menu_delete_ = menu_handler_.insert("Delete",
        std::bind(&WaypointDropperNode::menu_callback, this,
                  std::placeholders::_1));

    // Build the level list before any waypoint is loaded or dropped, so
    // ground estimation is level-aware from the first waypoint on.
    init_levels(configured_level_names, configured_level_heights);

    // Load waypoints from file if specified (relative paths resolve from outputs dir)
    if (!input_file.empty()) {
      std::string load_path = input_file;
      if (load_path.front() != '/') {
        load_path = output_dir_ + "/" + load_path;
      }
      load_waypoints(load_path);
      rebuild_markers();
    }

    // Select the startup level (also publishes the level slice + markers).
    if (!startup_level.empty()) {
      int idx = find_level(startup_level);
      if (idx < 0) {
        RCLCPP_WARN(get_logger(),
            "levels.active='%s' is not a known level; starting with no active level.",
            startup_level.c_str());
        apply_active_level(-1, /*sync_param=*/true);
      } else {
        apply_active_level(idx, /*sync_param=*/false);
      }
    } else {
      publish_level_cloud();
    }

    // 'levels.active' doubles as a runtime control: setting it by name selects
    // that level, setting it to "" drops back to whole-column ground search.
    param_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&WaypointDropperNode::on_set_parameters, this,
                  std::placeholders::_1));

    // Subscribe to the RViz "Publish Point" tool -> select/create a level
    sub_level_point_ = create_subscription<geometry_msgs::msg::PointStamped>(
        level_point_topic, rclcpp::QoS(1),
        std::bind(&WaypointDropperNode::level_point_callback, this,
                  std::placeholders::_1));

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
                  "Waypoint #%zu (id=%lu): x=%.2f y=%.2f z=%.2f yaw=%.2f level=%s "
                  "(inserted at index %zu, next insert_index=%d)",
                  waypoints_.size(), wp.id, wp.x, wp.y, wp.z, wp.yaw,
                  wp.level.empty() ? "<none>" : wp.level.c_str(),
                  pos, insert_index_);

              rebuild_markers();
            });

    RCLCPP_INFO(get_logger(),
        "Waypoint dropper ready. Use RViz '2D Pose Estimate' to drop waypoints. "
        "Left-click a marker to set insertion point. Right-click for menu "
        "(Insert Before/After, Snap to Active Level, Delete). Press Ctrl+C to save and exit.");
    RCLCPP_INFO(get_logger(),
        "Set insert index: ros2 param set /waypoint_dropper "
        "waypoint_dropper.insert_index <N>  (-1 = append)");
    if (levels_enable_) {
      RCLCPP_INFO(get_logger(),
          "Multi-level maps: click a floor with RViz 'Publish Point' (%s) to select "
          "the level new waypoints land on, or: ros2 param set /waypoint_dropper "
          "levels.active <name>  ('' = no level)",
          level_point_topic.c_str());
    }
  }

  void save_waypoints() {
    // Print to console
    RCLCPP_INFO(get_logger(), "=== Waypoints (%zu) ===", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];
      RCLCPP_INFO(get_logger(),
          "  [%zu] x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f level=%s",
          i, wp.x, wp.y, wp.z, wp.roll, wp.pitch, wp.yaw,
          wp.level.empty() ? "<none>" : wp.level.c_str());
    }

    if (waypoints_.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoints to save.");
      return;
    }

    // Only levels the route actually uses are worth writing out -- the level
    // list may also hold auto-detected floors nobody dropped a waypoint on.
    std::vector<const map_levels::Level*> used_levels;
    for (const auto& level : levels_) {
      size_t count = 0;
      for (const auto& wp : waypoints_) {
        if (wp.level == level.name) ++count;
      }
      if (count > 0) {
        used_levels.push_back(&level);
        RCLCPP_INFO(get_logger(), "  level '%s' (z=%.3f): %zu waypoints",
                    level.name.c_str(), level.z, count);
      }
    }

    // Build YAML
    YAML::Emitter out;
    out << YAML::BeginMap;
    if (!used_levels.empty()) {
      out << YAML::Key << "levels" << YAML::Value << YAML::BeginSeq;
      for (const auto* level : used_levels) {
        out << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << level->name;
        out << YAML::Key << "z" << YAML::Value << level->z;
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }
    out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
    for (const auto& wp : waypoints_) {
      out << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << wp.x;
      out << YAML::Key << "y" << YAML::Value << wp.y;
      out << YAML::Key << "z" << YAML::Value << wp.z;
      out << YAML::Key << "roll" << YAML::Value << wp.roll;
      out << YAML::Key << "pitch" << YAML::Value << wp.pitch;
      out << YAML::Key << "yaw" << YAML::Value << wp.yaw;
      if (!wp.level.empty()) {
        out << YAML::Key << "level" << YAML::Value << wp.level;
      }
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
  // --- Levels ---

  /// Assemble the level list: configured levels first, then floors detected
  /// from a z-histogram of the map. No level is made active here -- until one
  /// is selected, ground estimation searches the whole z column exactly as it
  /// did before levels existed.
  void init_levels(const std::vector<std::string>& names,
                   const std::vector<double>& heights) {
    if (!levels_enable_) {
      RCLCPP_INFO(get_logger(),
          "Levels disabled (levels.enable=false): ground height uses the full z column.");
      return;
    }

    if (!names.empty() || !heights.empty()) {
      if (names.size() != heights.size()) {
        RCLCPP_ERROR(get_logger(),
            "levels.names (%zu) and levels.heights (%zu) must have the same length; "
            "ignoring both.", names.size(), heights.size());
      } else {
        for (size_t i = 0; i < names.size(); ++i) {
          levels_.push_back({names[i], heights[i]});
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu configured level(s).", names.size());
      }
    }

    const auto detected =
        map_levels::detect_levels(*map_cloud_, level_detect_min_fraction_);
    if (detected.empty()) {
      RCLCPP_INFO(get_logger(),
          "No candidate floors detected in the map (levels.detect_min_fraction=%.3f). "
          "Expected on a large outdoor map; on a multi-storey one, lower "
          "levels.detect_min_fraction (try 0.005) or just click a floor with "
          "RViz 'Publish Point' to define a level.",
          level_detect_min_fraction_);
    } else {
      std::string list;
      for (double z : detected) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%s%.2f", list.empty() ? "" : ", ", z);
        list += buf;
      }
      RCLCPP_INFO(get_logger(),
          "Candidate floor heights from map z-histogram: [%s] "
          "(topmost is usually a ceiling -- nothing scans a roof from outside)",
          list.c_str());
    }

    if (level_auto_detect_) {
      for (double z : detected) {
        if (find_level_near(z) < 0) {
          levels_.push_back({next_level_name(), z});
        }
      }
    }

    std::sort(levels_.begin(), levels_.end(),
              [](const map_levels::Level& a, const map_levels::Level& b) {
                return a.z < b.z;
              });

    for (const auto& level : levels_) {
      RCLCPP_INFO(get_logger(), "  level '%s': z=%.3f (band [%.2f, %.2f])",
                  level.name.c_str(), level.z,
                  level.z - level_band_below_, level.z + level_band_above_);
    }
  }

  int find_level(const std::string& name) const {
    for (size_t i = 0; i < levels_.size(); ++i) {
      if (levels_[i].name == name) return static_cast<int>(i);
    }
    return -1;
  }

  /// Index of the level whose height is closest to z, within the merge
  /// tolerance; -1 when the nearest known level is a different storey.
  int find_level_near(double z) const {
    int best = -1;
    double best_dist = level_merge_tolerance_;
    for (size_t i = 0; i < levels_.size(); ++i) {
      const double dist = std::abs(levels_[i].z - z);
      if (dist < best_dist) {
        best_dist = dist;
        best = static_cast<int>(i);
      }
    }
    return best;
  }

  std::string next_level_name() {
    for (size_t n = levels_.size();; ++n) {
      std::string candidate = "L" + std::to_string(n);
      if (find_level(candidate) < 0) return candidate;
    }
  }

  const map_levels::Level* active_level() const {
    if (active_level_ < 0 || active_level_ >= static_cast<int>(levels_.size())) {
      return nullptr;
    }
    return &levels_[active_level_];
  }

  /// Make level `idx` (-1 = none) active, refresh the level slice and markers.
  /// `sync_param` writes the name back to levels.active; it must be false when
  /// called from the parameter callback itself (the value is already being set).
  void apply_active_level(int idx, bool sync_param) {
    active_level_ = idx;

    if (sync_param) {
      const std::string name = idx >= 0 ? levels_[idx].name : std::string();
      syncing_active_param_ = true;
      set_parameter(rclcpp::Parameter("levels.active", name));
      syncing_active_param_ = false;
    }

    if (const auto* level = active_level()) {
      RCLCPP_INFO(get_logger(),
          "Active level '%s': z=%.3f, ground search band [%.2f, %.2f]",
          level->name.c_str(), level->z,
          level->z - level_band_below_, level->z + level_band_above_);
    } else {
      RCLCPP_INFO(get_logger(),
          "No active level: ground height uses the full z column.");
    }

    publish_level_cloud();
    rebuild_markers();
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
      if (param.get_name() != "levels.active") continue;
      if (syncing_active_param_) continue;  // Echo of our own set_parameter().

      if (!levels_enable_) {
        result.successful = false;
        result.reason = "levels are disabled (levels.enable=false)";
        continue;
      }

      const std::string name = param.as_string();
      if (name.empty()) {
        apply_active_level(-1, /*sync_param=*/false);
        continue;
      }
      const int idx = find_level(name);
      if (idx < 0) {
        result.successful = false;
        result.reason = "unknown level '" + name + "'";
        continue;
      }
      apply_active_level(idx, /*sync_param=*/false);
    }
    return result;
  }

  /// RViz "Publish Point" on the map: select the level that surface belongs
  /// to, creating it if this is a storey we have not seen yet.
  ///
  /// The clicked height snaps to the nearest known floor within
  /// levels.merge_tolerance, which is what lets a click on a table top or a
  /// step still select the floor it stands on. Only when no known floor is
  /// that close does the click itself define a new level -- re-estimating the
  /// floor from the click would have to search well below it, and that reaches
  /// through the slab to the ceiling of the storey below.
  ///
  /// Note that a click in a top-down view lands on the topmost surface, which
  /// is a ceiling; select levels from an orbit view, from the already-sliced
  /// map_cloud_level, or by name via the levels.active parameter.
  void level_point_callback(
      const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
    if (!levels_enable_) return;

    const double click_z = msg->point.z;

    const int existing = find_level_near(click_z);
    if (existing >= 0) {
      RCLCPP_INFO(get_logger(),
          "Clicked (%.2f, %.2f, %.2f): selecting level '%s' (z=%.3f)",
          msg->point.x, msg->point.y, click_z,
          levels_[existing].name.c_str(), levels_[existing].z);
      apply_active_level(existing, /*sync_param=*/true);
      return;
    }

    levels_.push_back({next_level_name(), click_z});
    RCLCPP_INFO(get_logger(),
        "Clicked (%.2f, %.2f, %.2f): no known floor within %.2fm, "
        "created level '%s' at the clicked height",
        msg->point.x, msg->point.y, click_z, level_merge_tolerance_,
        levels_.back().name.c_str());
    apply_active_level(static_cast<int>(levels_.size()) - 1, /*sync_param=*/true);
  }

  /// Republish the latched slice of the map belonging to the active level.
  /// An empty cloud is published when no level is active, so a stale slice
  /// never lingers in RViz.
  void publish_level_cloud() {
    auto slice = std::make_shared<PointCloud>();

    const auto* level = active_level();
    if (level != nullptr) {
      const float lo = static_cast<float>(level->z - level_band_below_);
      const float hi = static_cast<float>(level->z + level_band_above_);
      for (const auto& pt : map_cloud_->points) {
        if (pt.z >= lo && pt.z <= hi) {
          slice->push_back(pt);
        }
      }

      if (map_voxel_size_ > 0.0 && !slice->empty()) {
        auto downsampled = std::make_shared<PointCloud>();
        pcl::VoxelGrid<PointType> voxel;
        voxel.setInputCloud(slice);
        voxel.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        voxel.filter(*downsampled);
        slice = downsampled;
      }

      RCLCPP_INFO(get_logger(),
          "Published level '%s' slice: %zu points in z [%.2f, %.2f]",
          level->name.c_str(), slice->size(), lo, hi);
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*slice, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    pub_level_map_->publish(msg);
  }

  // --- Waypoint extraction ---

  Waypoint extract_waypoint(double x, double y,
                            const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll_unused, pitch_unused, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll_unused, pitch_unused, yaw);

    double ground_z = compute_ground_height(x, y);
    const auto* level = active_level();

    return {next_id_++, x, y, ground_z, 0.0, 0.0, yaw,
            level != nullptr ? level->name : std::string()};
  }

  /// Ground height under (x, y). With a level active the search is confined to
  /// that storey's band, so a column crossing several floors cannot pull the
  /// estimate down to the one below.
  double compute_ground_height(double x, double y) const {
    double z_lo = -std::numeric_limits<double>::infinity();
    double z_hi = std::numeric_limits<double>::infinity();
    const auto* level = active_level();
    if (level != nullptr) {
      z_lo = level->z - level_band_below_;
      z_hi = level->z + level_band_above_;
    }

    double ground_z = 0.0;
    size_t n_points = 0;
    if (map_levels::ground_height(
            *map_cloud_, x, y, ground_search_radius_x_, ground_search_radius_y_,
            ground_percentile_, z_lo, z_hi, &ground_z, &n_points)) {
      if (level != nullptr) {
        RCLCPP_INFO(get_logger(),
            "Ground height at (%.2f, %.2f) on level '%s': %.3f "
            "(%zu points in search region)",
            x, y, level->name.c_str(), ground_z, n_points);
      } else {
        RCLCPP_INFO(get_logger(),
            "Ground height at (%.2f, %.2f): %.3f (%zu points in search region)",
            x, y, ground_z, n_points);
      }
      return ground_z;
    }

    if (level != nullptr) {
      RCLCPP_WARN(get_logger(),
          "No map points near (%.2f, %.2f) within (%.1f x %.1f) on level '%s' "
          "(z band [%.2f, %.2f]). Using the level height %.3f.",
          x, y, ground_search_radius_x_, ground_search_radius_y_,
          level->name.c_str(), z_lo, z_hi, level->z);
      return level->z;
    }

    RCLCPP_WARN(get_logger(),
        "No map points near (%.2f, %.2f) within (%.1f x %.1f). Using z=0.",
        x, y, ground_search_radius_x_, ground_search_radius_y_);
    return 0.0;
  }

  // --- YAML loading ---

  void load_waypoints(const std::string& filepath) {
    try {
      YAML::Node config = YAML::LoadFile(filepath);

      // Levels first, so waypoints can reference them by name.
      if (config["levels"] && config["levels"].IsSequence()) {
        for (const auto& entry : config["levels"]) {
          const auto name = entry["name"].as<std::string>();
          const auto z = entry["z"].as<double>();
          const int idx = find_level(name);
          if (idx < 0) {
            levels_.push_back({name, z});
          } else if (std::abs(levels_[idx].z - z) > 1e-3) {
            RCLCPP_WARN(get_logger(),
                "Level '%s' is already defined at z=%.3f; keeping it "
                "(%s says z=%.3f).",
                name.c_str(), levels_[idx].z, filepath.c_str(), z);
          }
        }
      }

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
          wp.level = entry["level"] ? entry["level"].as<std::string>()
                                    : std::string();
          // A waypoint may name a level the file never declared; take the
          // waypoint's own height as that level's definition.
          if (!wp.level.empty() && find_level(wp.level) < 0) {
            levels_.push_back({wp.level, wp.z});
            RCLCPP_WARN(get_logger(),
                "Waypoint references undeclared level '%s'; defining it at z=%.3f.",
                wp.level.c_str(), wp.z);
          }
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

      std::sort(levels_.begin(), levels_.end(),
                [](const map_levels::Level& a, const map_levels::Level& b) {
                  return a.z < b.z;
                });

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

    const auto* level = active_level();

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];

      // Waypoints on other floors are dimmed (or hidden) so the active level
      // stands out in a top-down view, where they all overlap.
      const bool on_active_level = level == nullptr || wp.level == level->name;
      if (!on_active_level && hide_other_levels_) {
        continue;
      }
      const float alpha = on_active_level ? 1.0f : 0.25f;

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
      sphere.color.a = 0.85f * alpha;
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
      arrow.color.a = 0.9f * alpha;
      // Arrow at same height as sphere, orientation follows the interactive marker's yaw
      arrow.pose.position.z = 0.5;
      button_control.markers.push_back(arrow);

      // Text label: index, plus the level when the route spans several floors
      visualization_msgs::msg::Marker text;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.text = std::to_string(i);
      if (!wp.level.empty()) {
        text.text += " [" + wp.level + "]";
      }
      text.scale.z = 0.5;  // text height
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = alpha;
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
    } else if (feedback->menu_entry_id == menu_snap_level_) {
      // Move a waypoint onto the active floor: re-estimate its height inside
      // that level's band. Repairs waypoints loaded from a pre-levels file.
      const auto* level = active_level();
      if (level == nullptr) {
        RCLCPP_WARN(get_logger(),
            "Snap to Active Level: no active level. Click a floor with RViz "
            "'Publish Point' first.");
        return;
      }
      auto& wp = waypoints_[wp_idx];
      const double old_z = wp.z;
      wp.level = level->name;
      wp.z = compute_ground_height(wp.x, wp.y);
      RCLCPP_INFO(get_logger(),
          "Snapped waypoint %zu to level '%s': z %.3f -> %.3f",
          wp_idx, level->name.c_str(), old_z, wp.z);
      rebuild_markers();
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_level_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_initialpose_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      sub_level_point_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle menu_insert_before_;
  interactive_markers::MenuHandler::EntryHandle menu_insert_after_;
  interactive_markers::MenuHandler::EntryHandle menu_snap_level_;
  interactive_markers::MenuHandler::EntryHandle menu_delete_;

  std::vector<Waypoint> waypoints_;
  uint64_t next_id_;
  int insert_index_;

  std::vector<map_levels::Level> levels_;
  int active_level_{-1};
  bool syncing_active_param_{false};

  std::string map_frame_;
  std::string output_dir_;
  std::string body_frame_;
  double map_voxel_size_;
  double ground_search_radius_x_;
  double ground_search_radius_y_;
  double ground_percentile_;

  bool levels_enable_;
  bool level_auto_detect_;
  bool hide_other_levels_;
  double level_band_below_;
  double level_band_above_;
  double level_merge_tolerance_;
  double level_detect_min_fraction_;
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
