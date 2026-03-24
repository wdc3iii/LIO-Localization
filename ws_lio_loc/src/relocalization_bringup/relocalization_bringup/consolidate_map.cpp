#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <chrono>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <climits>
#include <iomanip>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using PointType = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointType>;

// Configuration structure
struct Config {
  std::string output_name;
  bool intermediate_filter_enable;
  float intermediate_voxel_size;
  bool final_filter_enable;
  float voxel_size;
  bool delete_source_files;
  bool verbose;
  std::string source_pcd_dir;
  std::string output_base_dir;
};

// Helper function to get current timestamp in YYYY-MM-DD_HHmmss format
std::string getCurrentTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm* tm_info = std::localtime(&time);

  std::ostringstream oss;
  oss << std::put_time(tm_info, "%Y-%m-%d_%H%M%S");
  return oss.str();
}

// Helper function to log messages
void logMessage(const std::string& level, const std::string& msg, bool verbose = true) {
  if (level == "INFO" || level == "ERROR" || level == "WARN" || !verbose) {
    std::cout << "[" << level << "] " << msg << std::endl;
  }
}

// Get the size of a file in bytes
long long getFileSize(const fs::path& filepath) {
  return fs::file_size(filepath);
}

// Get total size of all files in a directory
long long getTotalDirectorySize(const fs::path& dirpath) {
  long long total_size = 0;
  try {
    for (const auto& entry : fs::directory_iterator(dirpath)) {
      if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
        total_size += getFileSize(entry.path());
      }
    }
  } catch (const std::exception& e) {
    logMessage("ERROR", std::string("Error calculating directory size: ") + e.what());
  }
  return total_size;
}

// Format bytes to human-readable string
std::string formatBytes(long long bytes) {
  const char* units[] = {"B", "KB", "MB", "GB"};
  double size = static_cast<double>(bytes);
  int unit_index = 0;

  while (size > 1024.0 && unit_index < 3) {
    size /= 1024.0;
    unit_index++;
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << size << " " << units[unit_index];
  return oss.str();
}

// Load configuration from YAML file
Config loadConfig(const std::string& config_path) {
  Config config;

  try {
    YAML::Node yaml = YAML::LoadFile(config_path);

    if (!yaml["consolidation"] || !yaml["source_pcd_dir"] || !yaml["output_base_dir"]) {
      throw std::runtime_error("Missing required configuration fields");
    }

    config.output_name = yaml["consolidation"]["output_name"].as<std::string>("lidar_map");
    config.intermediate_filter_enable = yaml["consolidation"]["intermediate_filter_enable"].as<bool>(true);
    config.intermediate_voxel_size = yaml["consolidation"]["intermediate_voxel_size"].as<float>(0.1f);
    config.final_filter_enable = yaml["consolidation"]["final_filter_enable"].as<bool>(true);
    config.voxel_size = yaml["consolidation"]["voxel_size"].as<float>(0.05f);
    config.delete_source_files = yaml["consolidation"]["delete_source_files"].as<bool>(true);
    config.verbose = yaml["consolidation"]["verbose"].as<bool>(true);
    config.source_pcd_dir = yaml["source_pcd_dir"].as<std::string>();
    config.output_base_dir = yaml["output_base_dir"].as<std::string>();

    return config;
  } catch (const std::exception& e) {
    logMessage("ERROR", std::string("Failed to load config: ") + e.what());
    throw;
  }
}

// Get vector of all PCD files in a directory
std::vector<fs::path> getPcdFiles(const fs::path& dirpath) {
  std::vector<fs::path> pcd_files;

  try {
    for (const auto& entry : fs::directory_iterator(dirpath)) {
      if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
        pcd_files.push_back(entry.path());
      }
    }
    std::sort(pcd_files.begin(), pcd_files.end());
  } catch (const std::exception& e) {
    logMessage("ERROR", std::string("Error reading PCD directory: ") + e.what());
  }

  return pcd_files;
}

// Copy all PCD files from source to destination
int copyPcdFiles(const fs::path& source_dir, const fs::path& dest_dir, bool verbose) {
  std::vector<fs::path> pcd_files = getPcdFiles(source_dir);

  if (pcd_files.empty()) {
    logMessage("ERROR", "No PCD files found in source directory: " + source_dir.string());
    return 0;
  }

  logMessage("INFO", "Copying " + std::to_string(pcd_files.size()) + " PCD files...", verbose);

  int successful_copies = 0;
  for (size_t i = 0; i < pcd_files.size(); ++i) {
    try {
      fs::path dest_file = dest_dir / pcd_files[i].filename();
      long long file_size = getFileSize(pcd_files[i]);

      logMessage("INFO",
                 "  [" + std::to_string(i + 1) + "/" + std::to_string(pcd_files.size()) + "] " +
                     "Copying " + pcd_files[i].filename().string() + " (" +
                     formatBytes(file_size) + ")...",
                 verbose);

      fs::copy_file(pcd_files[i], dest_file, fs::copy_options::overwrite_existing);
      successful_copies++;
    } catch (const std::exception& e) {
      logMessage("WARN", "Failed to copy " + pcd_files[i].filename().string() + ": " + e.what(),
                 verbose);
    }
  }

  long long total_copied = 0;
  for (const auto& file : pcd_files) {
    total_copied += getFileSize(file);
  }

  logMessage("INFO",
             "Copy complete: " + std::to_string(successful_copies) + " files, " +
                 formatBytes(total_copied) + " total",
             verbose);

  return successful_copies;
}

// Load a single PCD file
PointCloud::Ptr loadPcd(const fs::path& filepath) {
  PointCloud::Ptr cloud(new PointCloud);

  try {
    if (pcl::io::loadPCDFile<PointType>(filepath.string(), *cloud) == -1) {
      logMessage("ERROR", "Failed to load PCD file: " + filepath.string());
      return nullptr;
    }
  } catch (const std::exception& e) {
    logMessage("ERROR", "Exception loading PCD: " + std::string(e.what()));
    return nullptr;
  }

  return cloud;
}

// Compute the minimum voxel leaf size that avoids int32 overflow in VoxelGrid.
// VoxelGrid needs (ex/leaf) * (ey/leaf) * (ez/leaf) < INT32_MAX.
float getMinLeafSize(float ex, float ey, float ez) {
  constexpr double max_voxels = static_cast<double>(std::numeric_limits<int32_t>::max());
  // leaf^3 > (ex * ey * ez) / max_voxels  =>  leaf > cbrt(volume / max_voxels)
  double volume = static_cast<double>(ex) * ey * ez;
  return static_cast<float>(std::cbrt(volume / max_voxels));
}

// Apply voxel grid filter to point cloud, with automatic chunking for large extents.
PointCloud::Ptr filterCloud(const PointCloud::Ptr& cloud, float voxel_size, bool verbose = false) {
  if (cloud->empty()) return cloud;

  try {
    PointType min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    float ex = max_pt.x - min_pt.x;
    float ey = max_pt.y - min_pt.y;
    float ez = max_pt.z - min_pt.z;
    float min_leaf = getMinLeafSize(ex, ey, ez);

    // Fast path: leaf size is safe for the whole cloud
    if (voxel_size >= min_leaf) {
      PointCloud::Ptr filtered(new PointCloud);
      pcl::VoxelGrid<PointType> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(voxel_size, voxel_size, voxel_size);
      vg.filter(*filtered);
      return filtered;
    }

    // Chunked path: subdivide bounding box so each chunk fits in int32 indices
    // safe_extent = max extent per chunk axis that avoids overflow (with 10% margin)
    double safe_extent = voxel_size * std::cbrt(static_cast<double>(INT32_MAX) * 0.9);
    int nx = std::max(1, static_cast<int>(std::ceil(ex / safe_extent)));
    int ny = std::max(1, static_cast<int>(std::ceil(ey / safe_extent)));
    int nz = std::max(1, static_cast<int>(std::ceil(ez / safe_extent)));
    int total_chunks = nx * ny * nz;

    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2);
      oss << "Cloud extent (" << ex << " x " << ey << " x " << ez
          << " m) too large for leaf " << voxel_size << "m. "
          << "Chunking into " << nx << "x" << ny << "x" << nz
          << " = " << total_chunks << " chunks";
      logMessage("INFO", oss.str());
    }

    float dx = ex / nx;
    float dy = ey / ny;
    float dz = ez / nz;

    PointCloud::Ptr result(new PointCloud);
    int chunk_idx = 0;

    for (int ix = 0; ix < nx; ++ix) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int iz = 0; iz < nz; ++iz) {
          ++chunk_idx;

          // Compute chunk bounds with small overlap to avoid boundary artifacts
          float x_min = min_pt.x + ix * dx;
          float y_min = min_pt.y + iy * dy;
          float z_min = min_pt.z + iz * dz;
          float x_max = (ix == nx - 1) ? max_pt.x : x_min + dx;
          float y_max = (iy == ny - 1) ? max_pt.y : y_min + dy;
          float z_max = (iz == nz - 1) ? max_pt.z : z_min + dz;

          // Extract points in this chunk using CropBox
          pcl::CropBox<PointType> crop;
          crop.setInputCloud(cloud);
          crop.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0f));
          crop.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0f));

          PointCloud::Ptr chunk(new PointCloud);
          crop.filter(*chunk);

          if (chunk->empty()) {
            if (verbose) {
              logMessage("INFO", "    Chunk [" + std::to_string(chunk_idx) + "/" +
                         std::to_string(total_chunks) + "] empty, skipping");
            }
            continue;
          }

          // Apply voxel filter to chunk
          pcl::VoxelGrid<PointType> vg;
          vg.setInputCloud(chunk);
          vg.setLeafSize(voxel_size, voxel_size, voxel_size);

          PointCloud::Ptr chunk_filtered(new PointCloud);
          vg.filter(*chunk_filtered);

          *result += *chunk_filtered;

          if (verbose) {
            logMessage("INFO", "    Chunk [" + std::to_string(chunk_idx) + "/" +
                       std::to_string(total_chunks) + "] " +
                       std::to_string(chunk->size()) + " -> " +
                       std::to_string(chunk_filtered->size()) + " points" +
                       " (accumulated: " + std::to_string(result->size()) + ")");
          }
        }
      }
    }

    logMessage("INFO", "    Chunked filtering complete: " +
               std::to_string(cloud->size()) + " -> " +
               std::to_string(result->size()) + " points");

    return result;
  } catch (const std::exception& e) {
    logMessage("ERROR", "Exception during voxel filtering: " + std::string(e.what()));
    return cloud;  // Return unfiltered cloud on error
  }
}

// Consolidate all PCD files into one map
PointCloud::Ptr consolidatePcds(const fs::path& source_dir,
                                 bool intermediate_filter_enable, float intermediate_voxel_size,
                                 bool final_filter_enable, float final_voxel_size,
                                 bool verbose) {
  std::vector<fs::path> pcd_files = getPcdFiles(source_dir);

  if (pcd_files.empty()) {
    logMessage("ERROR", "No PCD files found to consolidate");
    return nullptr;
  }

  logMessage("INFO", "Consolidating " + std::to_string(pcd_files.size()) + " point clouds...",
             verbose);

  PointCloud::Ptr accumulated_cloud(new PointCloud);
  accumulated_cloud->is_dense = false;
  accumulated_cloud->width = 0;
  accumulated_cloud->height = 1;

  for (size_t i = 0; i < pcd_files.size(); ++i) {
    try {
      PointCloud::Ptr cloud = loadPcd(pcd_files[i]);
      if (!cloud || cloud->empty()) {
        logMessage("WARN", "Skipping empty or invalid cloud: " + pcd_files[i].filename().string(),
                   verbose);
        continue;
      }

      uint32_t points_before = cloud->size();

      logMessage("INFO",
                 "  [" + std::to_string(i + 1) + "/" + std::to_string(pcd_files.size()) + "] " +
                     "Loading " + pcd_files[i].filename().string() + " (" +
                     std::to_string(points_before) + " points)",
                 verbose);

      // Apply intermediate voxel filter if enabled
      PointCloud::Ptr to_add = cloud;
      if (intermediate_filter_enable) {
        to_add = filterCloud(cloud, intermediate_voxel_size, verbose);
        logMessage("INFO",
                   "    Voxel filtering (" + std::to_string(intermediate_voxel_size) + "m)... " +
                       std::to_string(to_add->size()) + " points",
                   verbose);
      }

      // Accumulate cloud
      *accumulated_cloud += *to_add;

      logMessage("INFO",
                 "    Accumulated: " + std::to_string(accumulated_cloud->size()) + " points",
                 verbose);
    } catch (const std::exception& e) {
      logMessage("WARN", "Error processing " + pcd_files[i].filename().string() + ": " + e.what(),
                 verbose);
    }
  }

  logMessage("INFO", "", verbose);  // Blank line for readability

  PointCloud::Ptr final_cloud = accumulated_cloud;
  if (final_filter_enable) {
    logMessage("INFO",
               "Final voxel filtering (" + std::to_string(final_voxel_size) + "m)...", verbose);

    uint32_t before_final = accumulated_cloud->size();
    final_cloud = filterCloud(accumulated_cloud, final_voxel_size, verbose);
    uint32_t after_final = final_cloud->size();

    double compression_ratio =
        before_final > 0 ? static_cast<double>(before_final) / after_final : 1.0;

    logMessage("INFO",
               "  Input points: " + std::to_string(before_final) + ", Output: " +
                   std::to_string(after_final),
               verbose);
    logMessage("INFO", "  Compression ratio: " + std::to_string(compression_ratio) + "x", verbose);
  } else {
    logMessage("INFO", "Final voxel filter disabled, skipping", verbose);
  }

  return final_cloud;
}

// Save point cloud to PCD file
bool savePcd(const PointCloud::Ptr& cloud, const fs::path& filepath) {
  try {
    if (pcl::io::savePCDFileBinary(filepath.string(), *cloud) == -1) {
      logMessage("ERROR", "Failed to save PCD file: " + filepath.string());
      return false;
    }

    long long file_size = getFileSize(filepath);
    logMessage("INFO",
               "Saved to " + filepath.filename().string() + " (" + formatBytes(file_size) + ", " +
                   std::to_string(cloud->size()) + " points)");
    return true;
  } catch (const std::exception& e) {
    logMessage("ERROR", "Exception saving PCD: " + std::string(e.what()));
    return false;
  }
}

// Delete all PCD files in a directory
int cleanupSourceFiles(const fs::path& source_dir, bool verbose) {
  std::vector<fs::path> pcd_files = getPcdFiles(source_dir);

  if (pcd_files.empty()) {
    return 0;
  }

  logMessage("INFO", "Deleting " + std::to_string(pcd_files.size()) + " source files...",
             verbose);

  int successful_deletes = 0;
  for (size_t i = 0; i < pcd_files.size(); ++i) {
    try {
      logMessage("INFO",
                 "  [" + std::to_string(i + 1) + "/" + std::to_string(pcd_files.size()) + "] " +
                     "Deleting " + pcd_files[i].filename().string() + "... OK",
                 verbose);
      fs::remove(pcd_files[i]);
      successful_deletes++;
    } catch (const std::exception& e) {
      logMessage("WARN", "Failed to delete " + pcd_files[i].filename().string() + ": " + e.what(),
                 verbose);
    }
  }

  long long total_size = getTotalDirectorySize(source_dir);  // Will be ~0 after deletion
  logMessage("INFO",
             "Cleanup complete: " + std::to_string(successful_deletes) + " files deleted",
             verbose);

  return successful_deletes;
}

// Main function
int main(int argc, char** argv) {
  std::string config_path;
  std::string source_override;
  std::string output_override;
  bool dry_run = false;
  auto start_time = std::chrono::steady_clock::now();

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--source" && i + 1 < argc) {
      source_override = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      output_override = argv[++i];
    } else if (arg == "--dry-run") {
      dry_run = true;
    } else if (arg == "--help") {
      std::cout << "Usage: consolidate_map [OPTIONS]\n"
                << "Options:\n"
                << "  --config PATH     Path to consolidate_map.yaml\n"
                << "                    (default: src/relocalization_bringup/config/consolidate_map.yaml)\n"
                << "  --source PATH     Override source PCD directory\n"
                << "  --output PATH     Override output base directory\n"
                << "  --dry-run         Verify setup without processing\n"
                << "  --help            Show this help message\n";
      return 0;
    }
  }

  if (config_path.empty()) {
    config_path = "src/relocalization_bringup/config/consolidate_map.yaml";
    logMessage("INFO", "No --config specified, using default: " + config_path);
  }

  // Load configuration
  Config config;
  try {
    config = loadConfig(config_path);
  } catch (const std::exception&) {
    return 1;
  }

  // Override paths if specified
  if (!source_override.empty()) {
    config.source_pcd_dir = source_override;
  }
  if (!output_override.empty()) {
    config.output_base_dir = output_override;
  }

  // Convert to absolute paths
  fs::path source_dir = fs::absolute(config.source_pcd_dir);
  fs::path output_base = fs::absolute(config.output_base_dir);

  logMessage("INFO", "Starting PCD consolidation");
  logMessage("INFO", "Config file: " + config_path);
  logMessage("INFO", "Source directory: " + source_dir.string(), config.verbose);
  logMessage("INFO", "Output directory: " + output_base.string(), config.verbose);

  // Validate source directory
  if (!fs::exists(source_dir)) {
    logMessage("ERROR", "Source directory does not exist: " + source_dir.string());
    return 1;
  }

  if (!fs::is_directory(source_dir)) {
    logMessage("ERROR", "Source path is not a directory: " + source_dir.string());
    return 1;
  }

  // Check for PCD files
  std::vector<fs::path> pcd_files = getPcdFiles(source_dir);
  if (pcd_files.empty()) {
    logMessage("ERROR", "No PCD files found in source directory");
    return 1;
  }

  logMessage("INFO", "Found " + std::to_string(pcd_files.size()) + " PCD files", config.verbose);

  // Validate output directory
  if (!fs::exists(output_base)) {
    logMessage("INFO", "Creating output directory: " + output_base.string(), config.verbose);
    try {
      fs::create_directories(output_base);
    } catch (const std::exception& e) {
      logMessage("ERROR", "Failed to create output directory: " + std::string(e.what()));
      return 1;
    }
  }

  // Create timestamped output folder
  std::string timestamp = getCurrentTimestamp();
  fs::path output_dir = output_base / (config.output_name + "_" + timestamp);

  logMessage("INFO", "Output folder: " + output_dir.string(), config.verbose);

  try {
    fs::create_directories(output_dir);
  } catch (const std::exception& e) {
    logMessage("ERROR", "Failed to create output folder: " + std::string(e.what()));
    return 1;
  }

  // Check disk space
  long long total_source_size = getTotalDirectorySize(source_dir);
  long long needed_space = 2 * total_source_size;

  logMessage("INFO", "Disk space check:", config.verbose);
  logMessage("INFO",
             "  Total source size: " + formatBytes(total_source_size), config.verbose);

  try {
    fs::space_info space = fs::space(output_base);
    logMessage("INFO",
               "  Available space: " + formatBytes(space.available), config.verbose);

    if (space.available < needed_space) {
      logMessage("ERROR", "Insufficient disk space. Need " + formatBytes(needed_space) +
                               " but only have " + formatBytes(space.available));
      return 1;
    }
  } catch (const std::exception& e) {
    logMessage("WARN", "Could not check disk space: " + std::string(e.what()), config.verbose);
  }

  logMessage("INFO", "Proceeding...", config.verbose);
  logMessage("INFO", "");  // Blank line

  if (dry_run) {
    logMessage("INFO", "Dry-run mode: verification complete, skipping actual processing");
    return 0;
  }

  // Copy PCD files
  int copied = copyPcdFiles(source_dir, output_dir, config.verbose);
  if (copied == 0) {
    logMessage("ERROR", "Failed to copy any PCD files");
    return 1;
  }

  logMessage("INFO", "");  // Blank line

  // Consolidate clouds
  PointCloud::Ptr consolidated =
      consolidatePcds(output_dir,
                      config.intermediate_filter_enable, config.intermediate_voxel_size,
                      config.final_filter_enable, config.voxel_size,
                      config.verbose);

  if (!consolidated || consolidated->empty()) {
    logMessage("ERROR", "Failed to create consolidated map");
    return 1;
  }

  logMessage("INFO", "");  // Blank line

  // Save final map
  fs::path map_file = output_dir / "map.pcd";
  logMessage("INFO", "Saving final map to: " + map_file.string(), config.verbose);

  if (!savePcd(consolidated, map_file)) {
    logMessage("ERROR", "Failed to save final map");
    return 1;
  }

  logMessage("INFO", "");  // Blank line

  // Cleanup source files
  if (config.delete_source_files) {
    cleanupSourceFiles(source_dir, config.verbose);
  } else {
    logMessage("INFO", "Source file deletion disabled, skipping cleanup", config.verbose);
  }

  // Calculate and log elapsed time
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
  int minutes = duration.count() / 60;
  int seconds = duration.count() % 60;

  logMessage("INFO", "");  // Blank line
  logMessage("INFO", "Consolidation successful!");
  logMessage("INFO", "  Time elapsed: " + std::to_string(minutes) + "m " +
                         std::to_string(seconds) + "s");
  logMessage("INFO", "  Output folder: " + output_dir.filename().string());
  logMessage("INFO", "  Final map: map.pcd (" + std::to_string(consolidated->size()) +
                         " points)");

  return 0;
}
