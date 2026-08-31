// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 California Institute of Technology and Will Compton

#pragma once

// Level ("floor") aware ground-height estimation in a prior point cloud map.
//
// A single-storey map lets you take the ground height under (x, y) as a low
// percentile of every map point in a column around it. In a multi-storey
// building that column contains every floor, ceiling and slab, so the
// percentile always lands on the lowest storey. A *level* fixes that: it is a
// named horizontal reference plane (the floor of one storey) at height z, and
// only points within [z - band_below, z + band_above] are considered part of
// it. Estimating the ground inside that band can only ever return a height on
// the selected floor.
//
// This header is duplicated between relocalization_bringup and scan_lock,
// mirroring pcd_path.h -- the two packages live in separate repositories.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>

namespace map_levels {

/// A named horizontal reference plane in the map: the floor of one storey.
struct Level {
  std::string name;
  double z{0.0};
};

/// z-histogram bin size (m) used by detect_levels(). Roughly the thickness of
/// the return band a flat floor produces.
inline constexpr double kDetectBinSize = 0.1;

/// Two histogram peaks closer than this (m) are a storey's ceiling and the
/// floor of the storey above it, separated only by the slab. detect_levels()
/// keeps the upper peak of such a pair -- that is the floor you can stand on.
inline constexpr double kDetectSlabGap = 1.0;

/// Robust ground height at (x, y), restricted to map points whose z lies in
/// [z_lo, z_hi]. Pass +/-infinity for an unbounded (single-storey) search.
///
/// The lower bound is the delicate one: a storey's ceiling sits only a slab
/// (~0.3 m) below the floor above it, and since this returns a *low*
/// percentile, a band reaching past that slab returns the ceiling below rather
/// than the floor you selected. Keep z_lo within a slab of the floor.
///
/// The search region is the axis-aligned box |px - x| < radius_x,
/// |py - y| < radius_y; the returned height is the `percentile` quantile of
/// the z values inside it. Returns false (leaving *ground_z untouched) when no
/// map point falls in the region, so the caller can pick its own fallback.
template <typename PointT>
bool ground_height(const pcl::PointCloud<PointT>& cloud,
                   double x, double y,
                   double radius_x, double radius_y,
                   double percentile,
                   double z_lo, double z_hi,
                   double* ground_z,
                   std::size_t* num_points = nullptr) {
  std::vector<float> z_values;
  z_values.reserve(1024);

  const float fx = static_cast<float>(x);
  const float fy = static_cast<float>(y);
  const float frx = static_cast<float>(radius_x);
  const float fry = static_cast<float>(radius_y);
  const float flo = static_cast<float>(z_lo);
  const float fhi = static_cast<float>(z_hi);

  for (const auto& pt : cloud.points) {
    if (std::abs(pt.x - fx) < frx && std::abs(pt.y - fy) < fry &&
        pt.z >= flo && pt.z <= fhi) {
      z_values.push_back(pt.z);
    }
  }

  if (num_points != nullptr) {
    *num_points = z_values.size();
  }
  if (z_values.empty()) {
    return false;
  }

  const double clamped = std::min(std::max(percentile, 0.0), 1.0);
  std::size_t idx = static_cast<std::size_t>(clamped * (z_values.size() - 1));
  if (idx >= z_values.size()) {
    idx = z_values.size() - 1;
  }
  std::nth_element(z_values.begin(), z_values.begin() + idx, z_values.end());
  *ground_z = z_values[idx];
  return true;
}

/// Candidate floor heights (ascending) found by peak-picking a z-histogram of
/// the whole map. Flat horizontal surfaces -- floors and ceilings -- show up
/// as sharp peaks; a bin qualifies when it holds at least `min_fraction` of
/// all map points and is a local maximum. Each height returned is the mean z
/// of its bin's points, not the bin centre -- callers band a ground search
/// within centimetres of these heights, so half a bin of error matters.
///
/// Floors and ceilings both peak, so peaks less than `slab_gap` apart are
/// collapsed to the upper one: a storey's ceiling sits one slab below the
/// floor of the storey above, and the floor is the useful height.
///
/// This is a hint for an operator, not ground truth -- the topmost peak of an
/// indoor map is usually the top storey's ceiling (nothing scans a roof from
/// outside), and sloped outdoor terrain may produce no peaks at all.
template <typename PointT>
std::vector<double> detect_levels(const pcl::PointCloud<PointT>& cloud,
                                  double min_fraction,
                                  double bin_size = kDetectBinSize,
                                  double slab_gap = kDetectSlabGap) {
  std::vector<double> levels;
  if (cloud.empty() || bin_size <= 0.0) {
    return levels;
  }

  float z_min = std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::lowest();
  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.z)) continue;
    z_min = std::min(z_min, pt.z);
    z_max = std::max(z_max, pt.z);
  }
  if (!(z_max > z_min)) {
    return levels;
  }

  const double extent = static_cast<double>(z_max) - static_cast<double>(z_min);
  const std::size_t n_bins = static_cast<std::size_t>(extent / bin_size) + 1;
  if (n_bins > 1000000) {  // Absurd vertical extent -- refuse rather than allocate.
    return levels;
  }

  std::vector<std::size_t> counts(n_bins, 0);
  std::vector<double> sums(n_bins, 0.0);
  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.z)) continue;
    std::size_t bin = static_cast<std::size_t>((pt.z - z_min) / bin_size);
    if (bin >= n_bins) bin = n_bins - 1;
    ++counts[bin];
    sums[bin] += static_cast<double>(pt.z);
  }

  const double threshold = min_fraction * static_cast<double>(cloud.size());
  std::vector<double> peaks;
  for (std::size_t i = 0; i < n_bins; ++i) {
    if (static_cast<double>(counts[i]) < threshold) continue;
    const std::size_t left = (i == 0) ? 0 : counts[i - 1];
    const std::size_t right = (i + 1 == n_bins) ? 0 : counts[i + 1];
    // >= on the left and > on the right keeps exactly one bin of a plateau.
    if (counts[i] >= left && counts[i] > right) {
      peaks.push_back(sums[i] / static_cast<double>(counts[i]));
    }
  }

  for (double z : peaks) {
    if (!levels.empty() && (z - levels.back()) < slab_gap) {
      levels.back() = z;  // Ceiling + floor above it: keep the floor.
    } else {
      levels.push_back(z);
    }
  }
  return levels;
}

/// Index of the height in `levels` closest to z, or -1 when the nearest known
/// floor is further away than `tolerance` (i.e. a storey we have not seen).
/// The tolerance is what lets a click on a table top or a step still resolve
/// to the floor it is standing on.
inline int nearest_level(const std::vector<double>& levels, double z,
                         double tolerance) {
  int best = -1;
  double best_dist = tolerance;
  for (std::size_t i = 0; i < levels.size(); ++i) {
    const double dist = std::abs(levels[i] - z);
    if (dist < best_dist) {
      best_dist = dist;
      best = static_cast<int>(i);
    }
  }
  return best;
}

}  // namespace map_levels
