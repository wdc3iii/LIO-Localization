// Headless replacement for GLIM's offline_viewer "Optimize" + "Export Points"
// menu items. Takes a GLIM dump directory (graph.bin + values.bin + per-submap
// dirs + config/), runs iSAM2 optimization, and writes a binary PLY of the
// concatenated submap points.
//
// Mirrors the lifecycle in glim/src/glim/viewer/offline_viewer.cpp:
//   GlobalConfig::instance(dump/config, true);
//   GlobalMappingParams params; params.enable_optimization = true; ...
//   GlobalMapping(params).load(dump);     // optimization runs here
//   export_points() -> PLY
//
// Designed for cluster use: no GUI, no GUI deps loaded, single-shot CLI.

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <glim/mapping/global_mapping.hpp>
#include <glim/util/config.hpp>

#include <glk/io/ply_io.hpp>

#include <gtsam_points/types/point_cloud.hpp>

namespace fs = std::filesystem;

namespace {

void printUsage(const char* prog) {
  std::cout
      << "Usage: " << prog << " INPUT_DUMP OUTPUT.ply [OPTIONS]\n"
      << "\n"
      << "Load a GLIM session dump, optionally run iSAM2 optimization, and\n"
      << "export the concatenated submap points as a binary PLY.\n"
      << "\n"
      << "Headless equivalent of offline_viewer's File > Save > Export Points\n"
      << "(combined with the 'Do optimization?' prompt that pops up on load).\n"
      << "\n"
      << "Options:\n"
      << "  --no-optimize        Skip iSAM2 optimization on load. Use when you\n"
      << "                       just want a PLY from an already-optimized dump.\n"
      << "  --save-dump PATH     After optimization, also write an updated dump\n"
      << "                       to PATH (mirrors File > Save > Save Map). The\n"
      << "                       PLY export still runs.\n"
      << "  -h, --help           Show this help.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string dump_path;
  std::string out_ply;
  std::string save_dump;
  bool optimize = true;

  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (a == "--no-optimize") {
      optimize = false;
    } else if (a == "--save-dump" && i + 1 < argc) {
      save_dump = argv[++i];
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
  dump_path = positional[0];
  out_ply = positional[1];

  if (!fs::is_directory(dump_path)) {
    std::cerr << "[ERROR] input dump dir not found: " << dump_path << "\n";
    return 1;
  }
  fs::path config_dir = fs::path(dump_path) / "config";
  if (!fs::is_directory(config_dir)) {
    std::cerr << "[ERROR] dump is missing config/ — was it saved with GLIM 1.x?\n"
              << "        looked for: " << config_dir << "\n";
    return 1;
  }
  if (fs::path(out_ply).has_parent_path()) {
    fs::create_directories(fs::path(out_ply).parent_path());
  }

  // The GlobalMappingParams default ctor reads `enable_optimization` etc. from
  // the active GlobalConfig, so the config dir must be installed first.
  glim::GlobalConfig::instance(config_dir.string(), true);

  glim::GlobalMappingParams params;
  // Aggressive relinearization for offline reoptimization — same overrides the
  // GUI viewer uses (see offline_viewer.cpp::load_map).
  params.isam2_relinearize_skip = 1;
  params.isam2_relinearize_thresh = 0.0;
  params.enable_optimization = optimize;

  std::cout << "[INFO] loading dump: " << dump_path
            << "  (optimize=" << (optimize ? "true" : "false") << ")\n";
  auto t0 = std::chrono::steady_clock::now();
  glim::GlobalMapping gm(params);
  if (!gm.load(dump_path)) {
    std::cerr << "[ERROR] GlobalMapping::load failed for " << dump_path << "\n";
    return 1;
  }
  auto load_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - t0)
                     .count();
  std::cout << "[INFO] load+optimize done in " << load_ms << " ms\n";

  if (!save_dump.empty()) {
    fs::create_directories(save_dump);
    std::cout << "[INFO] writing optimized dump to " << save_dump << "\n";
    gm.save(save_dump);
  }

  std::cout << "[INFO] concatenating submap points\n";
  auto points = gm.export_points();
  if (!points || !points->has_points()) {
    std::cerr << "[ERROR] GlobalMapping::export_points returned no points\n";
    return 1;
  }

  glk::PLYData ply;
  ply.vertices.reserve(points->size());
  const bool has_intensities = points->has_intensities();
  if (has_intensities) {
    ply.intensities.reserve(points->size());
  }
  for (size_t i = 0; i < points->size(); ++i) {
    ply.vertices.push_back(points->points[i].head<3>().cast<float>());
    if (has_intensities) {
      ply.intensities.push_back(static_cast<float>(points->intensities[i]));
    }
  }

  std::cout << "[INFO] writing PLY: " << out_ply << " (" << points->size()
            << " points" << (has_intensities ? ", with intensities" : "")
            << ")\n";
  if (!glk::save_ply_binary(out_ply, ply)) {
    std::cerr << "[ERROR] failed to write PLY: " << out_ply << "\n";
    return 1;
  }

  std::cout << "[OK] done\n";
  return 0;
}
