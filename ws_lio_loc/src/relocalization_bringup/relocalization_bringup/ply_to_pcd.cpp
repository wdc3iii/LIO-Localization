// Convert a PLY (e.g. from GLIM's offline_viewer Export -> Points) into the
// binary PCD format that scan_lock loads. Optionally voxel-downsamples the
// output (default voxel 0.05 m, matching consolidate_map.yaml). Reports the
// point-count and file-size compression achieved.

#include <chrono>
#include <cmath>
#include <climits>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

// scan_lock and consolidate_map both use PointXYZINormal, so we match that
// here. GLIM's PLY export typically only carries XYZ (and sometimes
// intensity); the unmapped fields get zero-padded, which scan_lock tolerates.
using PointType = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointType>;

namespace {

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

// Mirrors consolidate_map.cpp: VoxelGrid uses int32 voxel indices, so on huge
// extents (e.g. multi-floor SLAM maps) we have to chunk the cloud first.
float getMinLeafSize(float ex, float ey, float ez) {
  constexpr double max_voxels = static_cast<double>(std::numeric_limits<int32_t>::max());
  double volume = static_cast<double>(ex) * ey * ez;
  return static_cast<float>(std::cbrt(volume / max_voxels));
}

PointCloud::Ptr voxelDownsample(const PointCloud::Ptr& cloud, float voxel_size) {
  if (cloud->empty()) return cloud;

  PointType min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  float ex = max_pt.x - min_pt.x;
  float ey = max_pt.y - min_pt.y;
  float ez = max_pt.z - min_pt.z;
  float min_leaf = getMinLeafSize(ex, ey, ez);

  if (voxel_size >= min_leaf) {
    PointCloud::Ptr filtered(new PointCloud);
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*filtered);
    return filtered;
  }

  // Cloud is too big for a single VoxelGrid pass; chunk via CropBox.
  double safe_extent = voxel_size * std::cbrt(static_cast<double>(INT32_MAX) * 0.9);
  int nx = std::max(1, static_cast<int>(std::ceil(ex / safe_extent)));
  int ny = std::max(1, static_cast<int>(std::ceil(ey / safe_extent)));
  int nz = std::max(1, static_cast<int>(std::ceil(ez / safe_extent)));
  std::cout << "[INFO] cloud extent " << ex << " x " << ey << " x " << ez
            << " m too large for leaf " << voxel_size << "m; chunking "
            << nx << "x" << ny << "x" << nz << "\n";

  float dx = ex / nx;
  float dy = ey / ny;
  float dz = ez / nz;
  PointCloud::Ptr result(new PointCloud);

  for (int ix = 0; ix < nx; ++ix) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int iz = 0; iz < nz; ++iz) {
        float x_min = min_pt.x + ix * dx;
        float y_min = min_pt.y + iy * dy;
        float z_min = min_pt.z + iz * dz;
        float x_max = (ix == nx - 1) ? max_pt.x : x_min + dx;
        float y_max = (iy == ny - 1) ? max_pt.y : y_min + dy;
        float z_max = (iz == nz - 1) ? max_pt.z : z_min + dz;

        pcl::CropBox<PointType> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0f));
        crop.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0f));
        PointCloud::Ptr chunk(new PointCloud);
        crop.filter(*chunk);
        if (chunk->empty()) continue;

        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(chunk);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        PointCloud::Ptr chunk_filtered(new PointCloud);
        vg.filter(*chunk_filtered);
        *result += *chunk_filtered;
      }
    }
  }
  return result;
}

void printUsage(const char* prog) {
  std::cout << "Usage: " << prog << " INPUT.ply OUTPUT.pcd [OPTIONS]\n"
            << "\n"
            << "Convert a PLY file (e.g. exported by GLIM's offline_viewer) to a\n"
            << "binary PCD (PointXYZINormal) suitable for scan_lock_node.\n"
            << "\n"
            << "Options:\n"
            << "  --voxel SIZE      Voxel-downsample with leaf size SIZE (meters).\n"
            << "                    Default: 0.05. Set to 0 to disable.\n"
            << "  --no-voxel        Equivalent to --voxel 0.\n"
            << "  --ascii           Write ASCII PCD instead of binary.\n"
            << "  -h, --help        Show this help.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string in_path;
  std::string out_path;
  float voxel = 0.05f;
  bool binary = true;

  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      printUsage(argv[0]);
      return 0;
    }
    if (a == "--voxel" && i + 1 < argc) {
      voxel = std::stof(argv[++i]);
    } else if (a == "--no-voxel") {
      voxel = 0.0f;
    } else if (a == "--ascii") {
      binary = false;
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "Unknown option: " << a << "\n";
      printUsage(argv[0]);
      return 1;
    } else {
      positional.push_back(a);
    }
  }

  if (positional.size() != 2) {
    printUsage(argv[0]);
    return 1;
  }
  in_path = positional[0];
  out_path = positional[1];

  if (!fs::exists(in_path)) {
    std::cerr << "[ERROR] input file not found: " << in_path << "\n";
    return 1;
  }
  if (fs::path(in_path).extension() != ".ply") {
    std::cerr << "[WARN] input does not end in .ply: " << in_path
              << " (continuing anyway)\n";
  }

  PointCloud::Ptr cloud(new PointCloud);
  std::cout << "[INFO] loading PLY: " << in_path << "\n";
  if (pcl::io::loadPLYFile<PointType>(in_path, *cloud) == -1) {
    std::cerr << "[ERROR] failed to load PLY (pcl::io::loadPLYFile returned -1)\n";
    return 1;
  }
  if (cloud->empty()) {
    std::cerr << "[ERROR] loaded PLY has zero points\n";
    return 1;
  }
  long long in_bytes = fs::file_size(in_path);
  std::size_t in_pts = cloud->size();
  std::cout << "[INFO] loaded " << in_pts << " points ("
            << formatBytes(in_bytes) << " on disk)\n";

  PointCloud::Ptr out = cloud;
  if (voxel > 0.0f) {
    auto t0 = std::chrono::steady_clock::now();
    out = voxelDownsample(cloud, voxel);
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0)
                  .count();
    std::cout << "[INFO] voxel filtered (leaf " << voxel << " m) in " << dt << " ms\n";
  } else {
    std::cout << "[INFO] voxel filter disabled\n";
  }

  fs::path out_p = out_path;
  if (out_p.has_parent_path()) {
    fs::create_directories(out_p.parent_path());
  }

  int rc;
  if (binary) {
    rc = pcl::io::savePCDFileBinary(out_path, *out);
  } else {
    rc = pcl::io::savePCDFileASCII(out_path, *out);
  }
  if (rc == -1) {
    std::cerr << "[ERROR] failed to save PCD: " << out_path << "\n";
    return 1;
  }

  long long out_bytes = fs::file_size(out_path);
  std::size_t out_pts = out->size();

  double pt_ratio = in_pts > 0 ? static_cast<double>(in_pts) / out_pts : 1.0;
  double size_ratio = in_bytes > 0 ? static_cast<double>(in_bytes) / out_bytes : 1.0;

  std::cout << "\n[SUMMARY]\n"
            << "  input : " << in_pts  << " pts, " << formatBytes(in_bytes)  << "\n"
            << "  output: " << out_pts << " pts, " << formatBytes(out_bytes) << "\n";
  if (voxel > 0.0f) {
    std::cout << "  voxel : " << voxel << " m\n"
              << "  point compression : " << std::fixed << std::setprecision(2)
              << pt_ratio << "x (" << std::setprecision(1)
              << (in_pts > 0 ? 100.0 * (1.0 - static_cast<double>(out_pts) / in_pts) : 0.0)
              << "% reduction)\n"
              << "  file  compression : " << std::fixed << std::setprecision(2)
              << size_ratio << "x (" << std::setprecision(1)
              << (in_bytes > 0 ? 100.0 * (1.0 - static_cast<double>(out_bytes) / in_bytes) : 0.0)
              << "% reduction)\n";
  }
  std::cout << "  wrote " << out_path << " (" << (binary ? "binary" : "ascii") << ")\n";
  return 0;
}
