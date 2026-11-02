#ifndef ORB_SLAM3_STABILITY_DIAGNOSTICS_H
#define ORB_SLAM3_STABILITY_DIAGNOSTICS_H
#include <cstdint>
namespace ORB_SLAM3 {
class Frame; class KeyFrame; class Map;
namespace Stability {
// Configuration is fixed before System starts its worker threads.
void Configure();
bool Enabled();
bool EagerBoW();
void PrepareKeyFrame(KeyFrame* key, bool rgbd);
struct InputIdentity { std::uint64_t frame_id; int keypoints; };
InputIdentity CaptureInput(const Frame& frame);
void SetSource(std::uint64_t ordinal, std::uint64_t stamp,
               std::uint32_t rgb_seq, std::uint32_t depth_seq);
void ClearSource();
void BeginFrame(const Frame& frame, int state_before, float close_depth);
void EndFrame(const Frame& frame, int state_after);
enum Stage { None=0, Motion=1, Reference=2, Local=3, Relocalization=4 };
class StageScope {
 public: explicit StageScope(Stage stage); ~StageScope();
 private: Stage previous_;
};
struct FrameCounts {
 int motion_first=-1, motion_final=-1, motion_map=-1;
 int reference_coarse=-1, reference_map=-1;
 int local_points=-1, search_seen=-1, search_bad=-1, search_projected=-1;
 int search_visible=-1, search_matches=-1, local_inliers=-1, local_threshold=-1;
 int pose_calls=0, pose_before=-1, pose_after=-1, pose_outliers=-1;
 int kf_id=-1, kf_depth_slots=-1, kf_new_points=-1;
 int kf_tracked_close=-1,kf_untracked_close=-1,kf_need_close=-1,kf_ref_matches=-1;
 int kf_c1a=-1,kf_c1b=-1,kf_c1c=-1,kf_c2=-1,kf_mapper_idle=-1,kf_queue=-1;
 int bow_prepares=0;
 double bow_prepare_ms=0;
};
FrameCounts* Counts();
void PoseSnapshot(Frame& frame, bool after);
// The coordinates are the actual isInFrustum result used by local matching.
void SearchProjection(std::uint64_t point, float u, float v);
enum LifeKind { Birth=1, RetireOther=2, RetireRatio=3, RetireLowObs=4,
                RetireObservation=5, RetireReset=6, Replace=7,
                KeyFrameAdd=8, KeyFrameErase=9, ObservationAdd=10,
                ObservationErase=11, RecentGraduate=12 };
void Life(Map* map, LifeKind kind, std::uint64_t point, std::int64_t key,
          std::int64_t value=-1, std::int64_t extra=-1,
          std::int64_t found=-1, std::int64_t visible=-1);
// Closes admission under the same mutex used by every append. Idempotent;
// retains no Map/Frame/KeyFrame pointers. Call after callback ingress stops.
bool StopAndFlush(bool workers_joined);
}
}
#endif
