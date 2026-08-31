// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 California Institute of Technology and Will Compton
//
// Crop a map PCD down to the region a waypoint route actually visits.
//
// Takes the bounding box of every waypoint in a waypoint YAML (as written by
// waypoint_dropper), pads it, and keeps only the map points inside. The x/y
// footprint is squared up by default, so the padding is *at least* the
// requested buffer on every side. z is padded but never squared -- vertical
// extent is a building's business, not the route's.
//
// The cloud is filtered as a raw PCLPointCloud2, so whatever fields the input
// carries (PointXYZI from ply_to_pcd, PointXYZINormal from consolidate_map,
// ...) survive the crop unchanged.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>

#include <yaml-cpp/yaml.h>

#include "pcd_path.h"

namespace fs = std::filesystem;

namespace {

struct Bounds {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();
  std::size_t count = 0;

  void add(double x, double y, double z) {
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    min_z = std::min(min_z, z);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    max_z = std::max(max_z, z);
    ++count;
  }
};

std::string formatBytes(long long bytes) {
  const char* units[] = {"B", "KB", "MB", "GB"};
  double size = static_cast<double>(bytes);
  int unit = 0;
  while (size > 1024.0 && unit < 3) {
    size /= 1024.0;
    ++unit;
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << size << " " << units[unit];
  return oss.str();
}

/// Accept a path as given (absolute or relative to the caller's cwd, which is
/// what anyone typing a path at a shell expects), and only fall back to the
/// package-relative convention when that misses.
std::string resolveInput(const std::string& raw, const std::string& package_subdir) {
  if (fs::exists(raw)) return raw;
  const std::string expanded = pcd_path::expand_env_vars(raw);
  if (fs::exists(expanded)) return expanded;
  if (!expanded.empty() && expanded.front() == '/') return expanded;
  return std::string(ROOT_DIR) + package_subdir + "/" + expanded;
}

/// Bounding box of every waypoint in a waypoint_dropper YAML. Handles the
/// current `waypoints:` key and the legacy `poses:` / `goals:` ones.
bool loadWaypointBounds(const std::string& path, Bounds* bounds) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] failed to parse " << path << ": " << e.what() << "\n";
    return false;
  }

  for (const char* key : {"waypoints", "poses", "goals"}) {
    const YAML::Node node = config[key];
    if (!node || !node.IsSequence()) continue;
    for (const auto& entry : node) {
      try {
        bounds->add(entry["x"].as<double>(), entry["y"].as<double>(),
                    entry["z"].as<double>());
      } catch (const std::exception& e) {
        std::cerr << "[ERROR] waypoint in " << path
                  << " is missing x/y/z: " << e.what() << "\n";
        return false;
      }
    }
  }

  if (bounds->count == 0) {
    std::cerr << "[ERROR] no waypoints found in " << path
              << " (expected a 'waypoints', 'poses' or 'goals' sequence)\n";
    return false;
  }
  return true;
}

/// Bounding box of a raw PCLPointCloud2, read straight out of the point buffer
/// via the x/y/z field offsets -- no conversion to a typed cloud, which on a
/// 40M-point map would mean copying half a gigabyte to learn six numbers.
/// Returns false if the cloud has no float32 x/y/z fields.
bool cloudBounds(const pcl::PCLPointCloud2& cloud, Bounds* bounds) {
  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto& f : cloud.fields) {
    if (f.datatype != pcl::PCLPointField::FLOAT32) continue;
    if (f.name == "x") off_x = static_cast<int>(f.offset);
    else if (f.name == "y") off_y = static_cast<int>(f.offset);
    else if (f.name == "z") off_z = static_cast<int>(f.offset);
  }
  if (off_x < 0 || off_y < 0 || off_z < 0) return false;

  const std::size_t n = static_cast<std::size_t>(cloud.width) * cloud.height;
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t* pt = cloud.data.data() + i * cloud.point_step;
    float x, y, z;
    std::memcpy(&x, pt + off_x, sizeof(float));
    std::memcpy(&y, pt + off_y, sizeof(float));
    std::memcpy(&z, pt + off_z, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    bounds->add(x, y, z);
  }
  return bounds->count > 0;
}

void printUsage(const char* prog) {
  std::cout << "Usage: " << prog << " WAYPOINTS.yaml INPUT.pcd OUTPUT.pcd [OPTIONS]\n"
            << "\n"
            << "Crop a map PCD to the region a waypoint route visits: the bounding\n"
            << "box of every waypoint, padded by a buffer. The x/y footprint is a\n"
            << "square by default, so the padding is at least --xy-buffer on every\n"
            << "side. All input point fields are preserved.\n"
            << "\n"
            << "WAYPOINTS.yaml and INPUT.pcd are used as given if they exist;\n"
            << "otherwise they resolve under the package's outputs/ and pcd/\n"
            << "directories respectively. $ENV vars are expanded in both.\n"
            << "\n"
            << "Options:\n"
            << "  --xy-buffer M   Minimum padding (meters) around the waypoints in\n"
            << "                  x and y. Default: 40.\n"
            << "  --z-buffer M    Padding (meters) below the lowest and above the\n"
            << "                  highest waypoint. Default: 5.\n"
            << "  --rect          Don't square the footprint: pad the waypoint\n"
            << "                  bounding box by exactly --xy-buffer in x and y.\n"
            << "  --dry-run       Report the crop box and the surviving point count\n"
            << "                  without writing OUTPUT.pcd.\n"
            << "  --ascii         Write ASCII PCD instead of binary.\n"
            << "  --copy-to PATH  After writing OUTPUT.pcd, also copy it to PATH.\n"
            << "                  A directory keeps the original basename; anything\n"
            << "                  else is treated as the target file path.\n"
            << "  -h, --help      Show this help.\n";
}

}  // namespace

int main(int argc, char** argv) {
  double xy_buffer = 40.0;
  double z_buffer = 5.0;
  bool square = true;
  bool dry_run = false;
  bool binary = true;
  std::string copy_to;

  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      printUsage(argv[0]);
      return 0;
    }
    if (a == "--xy-buffer" && i + 1 < argc) {
      xy_buffer = std::stod(argv[++i]);
    } else if (a == "--z-buffer" && i + 1 < argc) {
      z_buffer = std::stod(argv[++i]);
    } else if (a == "--rect") {
      square = false;
    } else if (a == "--dry-run") {
      dry_run = true;
    } else if (a == "--ascii") {
      binary = false;
    } else if (a == "--copy-to" && i + 1 < argc) {
      copy_to = argv[++i];
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "Unknown option: " << a << "\n";
      printUsage(argv[0]);
      return 1;
    } else {
      positional.push_back(a);
    }
  }

  if (positional.size() != 3) {
    printUsage(argv[0]);
    return 1;
  }
  if (xy_buffer < 0.0 || z_buffer < 0.0) {
    std::cerr << "[ERROR] buffers must be >= 0 (got xy=" << xy_buffer
              << ", z=" << z_buffer << ")\n";
    return 1;
  }

  std::string wp_path, in_path;
  try {
    wp_path = resolveInput(positional[0], "outputs");
    in_path = resolveInput(positional[1], "pcd");
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] " << e.what() << "\n";
    return 1;
  }
  const std::string out_path = positional[2];

  if (!fs::exists(wp_path)) {
    std::cerr << "[ERROR] waypoint file not found: " << wp_path << "\n";
    return 1;
  }
  if (!fs::exists(in_path)) {
    std::cerr << "[ERROR] input PCD not found: " << in_path << "\n";
    return 1;
  }

  // --- Waypoint bounds -> crop box ---

  Bounds wp;
  if (!loadWaypointBounds(wp_path, &wp)) return 1;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "[INFO] " << wp.count << " waypoints from " << wp_path << "\n"
            << "[INFO] waypoint bounds: x [" << wp.min_x << ", " << wp.max_x
            << "]  y [" << wp.min_y << ", " << wp.max_y
            << "]  z [" << wp.min_z << ", " << wp.max_z << "]\n";

  double box_min_x, box_max_x, box_min_y, box_max_y;
  if (square) {
    const double cx = 0.5 * (wp.min_x + wp.max_x);
    const double cy = 0.5 * (wp.min_y + wp.max_y);
    const double half =
        0.5 * std::max(wp.max_x - wp.min_x, wp.max_y - wp.min_y) + xy_buffer;
    box_min_x = cx - half;
    box_max_x = cx + half;
    box_min_y = cy - half;
    box_max_y = cy + half;
  } else {
    box_min_x = wp.min_x - xy_buffer;
    box_max_x = wp.max_x + xy_buffer;
    box_min_y = wp.min_y - xy_buffer;
    box_max_y = wp.max_y + xy_buffer;
  }
  const double box_min_z = wp.min_z - z_buffer;
  const double box_max_z = wp.max_z + z_buffer;

  std::cout << "[INFO] crop box (" << (square ? "square" : "rect")
            << ", xy buffer " << xy_buffer << "m, z buffer " << z_buffer << "m): "
            << "x [" << box_min_x << ", " << box_max_x
            << "]  y [" << box_min_y << ", " << box_max_y
            << "]  z [" << box_min_z << ", " << box_max_z << "]\n"
            << "[INFO] crop box size: " << (box_max_x - box_min_x) << " x "
            << (box_max_y - box_min_y) << " x " << (box_max_z - box_min_z) << " m\n";

  // --- Crop ---

  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  std::cout << "[INFO] loading PCD: " << in_path << "\n";
  if (pcl::io::loadPCDFile(in_path, *cloud) == -1) {
    std::cerr << "[ERROR] failed to load PCD (pcl::io::loadPCDFile returned -1)\n";
    return 1;
  }
  const std::size_t in_pts =
      static_cast<std::size_t>(cloud->width) * cloud->height;
  if (in_pts == 0) {
    std::cerr << "[ERROR] loaded PCD has zero points\n";
    return 1;
  }
  const long long in_bytes = fs::file_size(in_path);
  std::cout << "[INFO] loaded " << in_pts << " points ("
            << formatBytes(in_bytes) << " on disk)\n";

  Bounds map_bounds;
  const bool have_map_bounds = cloudBounds(*cloud, &map_bounds);
  if (have_map_bounds) {
    std::cout << "[INFO] map bounds: x [" << map_bounds.min_x << ", "
              << map_bounds.max_x << "]  y [" << map_bounds.min_y << ", "
              << map_bounds.max_y << "]  z [" << map_bounds.min_z << ", "
              << map_bounds.max_z << "]\n";
  }

  auto t0 = std::chrono::steady_clock::now();
  pcl::PCLPointCloud2::Ptr cropped(new pcl::PCLPointCloud2);
  pcl::CropBox<pcl::PCLPointCloud2> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(static_cast<float>(box_min_x),
                              static_cast<float>(box_min_y),
                              static_cast<float>(box_min_z), 1.0f));
  crop.setMax(Eigen::Vector4f(static_cast<float>(box_max_x),
                              static_cast<float>(box_max_y),
                              static_cast<float>(box_max_z), 1.0f));
  crop.filter(*cropped);
  const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0).count();

  const std::size_t out_pts =
      static_cast<std::size_t>(cropped->width) * cropped->height;
  const double kept = 100.0 * static_cast<double>(out_pts) /
                      static_cast<double>(in_pts);
  std::cout << "[INFO] cropped " << in_pts << " -> " << out_pts << " points ("
            << std::setprecision(1) << kept << "% kept) in " << dt << " ms\n"
            << std::setprecision(2);

  if (out_pts == 0) {
    std::cerr << "[ERROR] crop box contains no map points -- do these waypoints "
                 "belong to this map?\n";
    if (have_map_bounds) {
      std::cerr << "[ERROR]   waypoints span x [" << wp.min_x << ", " << wp.max_x
                << "]  y [" << wp.min_y << ", " << wp.max_y << "]  z ["
                << wp.min_z << ", " << wp.max_z << "]\n"
                << "[ERROR]   map spans       x [" << map_bounds.min_x << ", "
                << map_bounds.max_x << "]  y [" << map_bounds.min_y << ", "
                << map_bounds.max_y << "]  z [" << map_bounds.min_z << ", "
                << map_bounds.max_z << "]\n";
    }
    return 1;
  }

  if (dry_run) {
    std::cout << "[INFO] dry run: not writing " << out_path << "\n";
    return 0;
  }

  // --- Write ---

  const fs::path out_fs(out_path);
  if (out_fs.has_parent_path() && !out_fs.parent_path().empty()) {
    std::error_code ec;
    fs::create_directories(out_fs.parent_path(), ec);
  }

  pcl::PCDWriter writer;
  const int rc = binary
      ? writer.writeBinary(out_path, *cropped)
      : writer.writeASCII(out_path, *cropped);
  if (rc != 0) {
    std::cerr << "[ERROR] failed to write " << out_path << "\n";
    return 1;
  }

  const long long out_bytes = fs::file_size(out_path);
  std::cout << "[INFO] wrote " << out_path << " (" << formatBytes(out_bytes)
            << ", was " << formatBytes(in_bytes) << ")\n";

  if (!copy_to.empty()) {
    fs::path target(copy_to);
    if (fs::is_directory(target)) {
      target /= out_fs.filename();
    } else if (target.has_parent_path() && !target.parent_path().empty()) {
      std::error_code ec;
      fs::create_directories(target.parent_path(), ec);
    }
    std::error_code ec;
    fs::copy_file(out_path, target, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      std::cerr << "[ERROR] failed to copy to " << target << ": "
                << ec.message() << "\n";
      return 1;
    }
    std::cout << "[INFO] copied to " << target << "\n";
  }

  return 0;
}
