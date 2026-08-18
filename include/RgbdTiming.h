#ifndef ORB_SLAM3_RGBD_TIMING_H
#define ORB_SLAM3_RGBD_TIMING_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace ORB_SLAM3 {

constexpr std::size_t kMaxRgbdOrbLevels = 16;

struct RgbdTimingRecord {
  double image_timestamp = 0.0;
  std::uint64_t frame_id = 0;
  int state_before = -1;
  int state_after = -1;
  int keypoints = 0;
  int valid_depths = 0;
  int tracked_inliers = 0;
  int local_mapping_queue_before = 0;
  int local_mapping_queue_after = 0;
  int inserted_keyframe = 0;

  double system_clone_ms = 0.0;
  double system_resize_ms = 0.0;
  double mode_mutex_wait_ms = 0.0;
  double mode_change_ms = 0.0;
  double reset_mutex_wait_ms = 0.0;
  double reset_ms = 0.0;
  double grab_rgbd_ms = 0.0;
  double state_copy_ms = 0.0;
  double system_total_ms = 0.0;

  double gray_convert_ms = 0.0;
  double depth_convert_ms = 0.0;
  double frame_construct_ms = 0.0;
  double orb_extract_ms = 0.0;
  double orb_pyramid_ms = 0.0;
  double orb_keypoints_octree_ms = 0.0;
  double orb_orientation_ms = 0.0;
  double orb_blur_ms = 0.0;
  double orb_descriptor_ms = 0.0;
  double orb_result_assembly_ms = 0.0;
  double orb_fast_initial_ms = 0.0;
  double orb_fast_fallback_ms = 0.0;
  double orb_octree_ms = 0.0;
  double orb_copy_ms = 0.0;
  double orb_gaussian_ms = 0.0;
  std::uint64_t orb_output_hash = 0;
  int orb_mono_index = 0;
  int orb_level_count = 0;
  std::array<double, kMaxRgbdOrbLevels> orb_level_fast_initial_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_fast_fallback_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_octree_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_orientation_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_copy_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_gaussian_ms{};
  std::array<double, kMaxRgbdOrbLevels> orb_level_descriptor_ms{};
  std::array<std::uint32_t, kMaxRgbdOrbLevels> orb_level_candidates{};
  std::array<std::uint32_t, kMaxRgbdOrbLevels> orb_level_fallback_cells{};
  std::array<std::uint32_t, kMaxRgbdOrbLevels> orb_level_octree_splits{};
  double rgbd_depth_assoc_ms = 0.0;
  double track_internal_ms = 0.0;
  double map_mutex_wait_ms = 0.0;
  double initialization_ms = 0.0;
  double reference_kf_ms = 0.0;
  double motion_model_ms = 0.0;
  double relocalization_ms = 0.0;
  double local_map_ms = 0.0;
  double need_new_kf_ms = 0.0;
  double create_new_kf_ms = 0.0;
};

struct LocalMappingTimingRecord {
  std::uint64_t keyframe_id = 0;
  double image_timestamp = 0.0;
  int queue_depth_before_pop = 0;
  double queue_wait_ms = 0.0;
  double bow_ms = 0.0;
  double process_keyframe_ms = 0.0;
  double map_point_culling_ms = 0.0;
  double map_point_creation_ms = 0.0;
  double local_ba_ms = 0.0;
  double keyframe_culling_ms = 0.0;
  double total_ms = 0.0;
  int lba_executed = 0;
  int lba_aborted = 0;
  int lba_edges = 0;
  int lba_optimized_keyframes = 0;
  int lba_fixed_keyframes = 0;
  int lba_map_points = 0;
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_RGBD_TIMING_H
