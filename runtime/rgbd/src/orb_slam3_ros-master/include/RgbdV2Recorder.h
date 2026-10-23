#ifndef RGBD_V2_RECORDER_H
#define RGBD_V2_RECORDER_H

// This is owned by one ROS callback thread. Receipt timestamps come from the
// original MessageEvent, observed at filter callback entry after deserialization.
#include "RgbdTiming.h"
#include "../../../memory/tools/rgbd_v2_timing_csv.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rgbd_v2 {
inline std::uint64_t Now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Source {
  std::uint64_t ordinal = 0, stamp_ns = 0;
  std::uint32_t rgb_seq = 0, depth_seq = 0;
};

// Keep only fields actually serialized by the common L0 writer. The 16-level
// fine arrays are not retained once per frame in an L0 run.
#define RGBD_V2_RETAINED_FIELDS(F) F(l0_collected) \
  F(observation) \
  F(tracking_thread_id) \
  F(frame_id) \
  F(state_before) \
  F(state_after) \
  F(keypoints) \
  F(valid_depths) \
  F(tracked_inliers) \
  F(local_mapping_queue_before) \
  F(local_mapping_queue_after) \
  F(inserted_keyframe) \
  F(keyframe_decision) \
  F(pose) \
  F(projection_match) \
  F(voting_slots) \
  F(voting_associated) \
  F(voting_observation_entries) \
  F(voting_unique_keyframes) \
  F(gather) \
  F(search_associated_slots) \
  F(search_local_slots) \
  F(search_seen_skips) \
  F(search_bad_skips) \
  F(search_projected) \
  F(search_rejected) \
  F(search_visible) \
  F(search_matches) \
  F(local_keyframes_cpu_ms) \
  F(local_points_cpu_ms) \
  F(local_search_cpu_ms) \
  F(system_clone_ms) \
  F(system_resize_ms) \
  F(mode_mutex_wait_ms) \
  F(mode_change_ms) \
  F(reset_mutex_wait_ms) \
  F(reset_ms) \
  F(grab_rgbd_ms) \
  F(state_copy_ms) \
  F(gray_convert_ms) \
  F(depth_convert_ms) \
  F(rgbd_depth_assoc_ms) \
  F(system_total_ms) \
  F(frame_construct_ms) \
  F(frame_construct_cpu_ms) \
  F(orb_level_count) \
  F(orb_level_candidates) \
  F(orb_extract_ms) \
  F(orb_pyramid_ms) \
  F(orb_keypoints_octree_ms) \
  F(orb_orientation_ms) \
  F(orb_copy_ms) \
  F(orb_gaussian_ms) \
  F(orb_descriptor_ms) \
  F(orb_blur_ms) \
  F(orb_result_assembly_ms) \
  F(track_internal_ms) \
  F(map_mutex_wait_ms) \
  F(initialization_ms) \
  F(reference_kf_ms) \
  F(motion_model_ms) \
  F(relocalization_ms) \
  F(local_map_ms) \
  F(local_update_ms) \
  F(local_keyframes_ms) \
  F(local_points_ms) \
  F(local_search_ms) \
  F(local_pose_ms) \
  F(local_keyframes) \
  F(local_points) \
  F(need_new_kf_ms) \
  F(create_new_kf_ms)
struct LightRecord {
  struct CompactOrb {
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.collected) collected{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.detail_collected) detail_collected{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.cpu_valid) cpu_valid{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.extract_caller_cpu_ms) extract_caller_cpu_ms{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.helper_cpu_ms) helper_cpu_ms{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.total_cpu_ms) total_cpu_ms{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.worker_count) worker_count{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.stage_caller_cpu_ms) stage_caller_cpu_ms{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.stage_helper_cpu_ms) stage_helper_cpu_ms{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.worker_cpu_ms) worker_cpu_ms{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.worker_tasks) worker_tasks{};
    decltype(ORB_SLAM3::RgbdTimingRecord().orb.level_outputs) level_outputs{};
  } orb;
#define RGBD_V2_MEMBER(field) decltype(ORB_SLAM3::RgbdTimingRecord().field) field{};
  RGBD_V2_RETAINED_FIELDS(RGBD_V2_MEMBER)
#undef RGBD_V2_MEMBER
  void Capture(const ORB_SLAM3::RgbdTimingRecord& record) {
    orb.collected = record.orb.collected;
    orb.detail_collected = record.orb.detail_collected;
    orb.cpu_valid = record.orb.cpu_valid;
    orb.extract_caller_cpu_ms = record.orb.extract_caller_cpu_ms;
    orb.helper_cpu_ms = record.orb.helper_cpu_ms;
    orb.total_cpu_ms = record.orb.total_cpu_ms;
    orb.worker_count = record.orb.worker_count;
    orb.stage_caller_cpu_ms = record.orb.stage_caller_cpu_ms;
    orb.stage_helper_cpu_ms = record.orb.stage_helper_cpu_ms;
    orb.worker_cpu_ms = record.orb.worker_cpu_ms;
    orb.worker_tasks = record.orb.worker_tasks;
    orb.level_outputs = record.orb.level_outputs;
#define RGBD_V2_COPY(field) field = record.field;
    RGBD_V2_RETAINED_FIELDS(RGBD_V2_COPY)
#undef RGBD_V2_COPY
  }
  ORB_SLAM3::RgbdTimingRecord Expand() const {
    ORB_SLAM3::RgbdTimingRecord record;
    record.orb.collected = orb.collected;
    record.orb.detail_collected = orb.detail_collected;
    record.orb.cpu_valid = orb.cpu_valid;
    record.orb.extract_caller_cpu_ms = orb.extract_caller_cpu_ms;
    record.orb.helper_cpu_ms = orb.helper_cpu_ms;
    record.orb.total_cpu_ms = orb.total_cpu_ms;
    record.orb.worker_count = orb.worker_count;
    record.orb.stage_caller_cpu_ms = orb.stage_caller_cpu_ms;
    record.orb.stage_helper_cpu_ms = orb.stage_helper_cpu_ms;
    record.orb.worker_cpu_ms = orb.worker_cpu_ms;
    record.orb.worker_tasks = orb.worker_tasks;
    record.orb.level_outputs = orb.level_outputs;
#define RGBD_V2_EXPAND(field) record.field = field;
    RGBD_V2_RETAINED_FIELDS(RGBD_V2_EXPAND)
#undef RGBD_V2_EXPAND
    return record;
  }
};
#undef RGBD_V2_RETAINED_FIELDS

enum class EventKind {
  ReceivedRgb, ReceivedDepth, Paired, ProcessedEnter, ProcessedReturn,
  StateOutput, PosePublish, SyncDropRgb, SyncDropDepth, CallbackReturn,
  LateRgb, LateDepth, UnmatchedRgb, UnmatchedDepth, Count
};
struct Event {
  std::uint64_t stamp_ns, steady_ns, receipt_ros_ns;
  std::uint32_t rgb_seq, message_seq;
  EventKind kind;
};
struct Frame {
  Source source;
  std::uint64_t enter_ns = 0, return_ns = 0;
  double track_call_ms = 0, callback_complete_ms = 0;
  int state_after = -1;
  unsigned state_publications = 0;
  bool pose_valid = false, pose_fresh = false, pose_emitted = false;
  bool track_called = false, track_returned = false, callback_exception = false;
  LightRecord light;
};

class Recorder {
 public:
  static constexpr std::size_t MaxFrames = 8192, MaxEvents = 65536;
  static constexpr std::size_t MaxRetainedBytes = 8U * 1024U * 1024U;
  static constexpr std::size_t MaxDetailRetainedBytes = 4U * 1024U * 1024U;
  using Counts = std::array<std::uint32_t, static_cast<std::size_t>(EventKind::Count)>;
  explicit Recorder(const std::string& index_path, bool detail_windows = false, bool collect_core_l0 = true)
      : detail_windows_(detail_windows), core_l0_enabled_(collect_core_l0) {
    if (detail_windows_ && !core_l0_enabled_)
      throw std::invalid_argument("L1 detail requires core L0 enabled");
    std::ifstream input(index_path);
    std::string line;
    const auto read_line = [&]() {
      if (!std::getline(input, line)) return false;
      // getline removes LF but retains CR from Python csv's default CRLF.
      if (!line.empty() && line.back() == '\r') line.pop_back();
      return true;
    };
    if (!input || !read_line() ||
        line != "source_ordinal,source_stamp_ns,rgb_seq,depth_seq")
      throw std::runtime_error("invalid frozen RGBD input index header");
    std::set<std::uint32_t> rgb_sequences, depth_sequences;
    while (read_line()) {
      std::array<std::uint64_t, 4> fields{};
      std::size_t begin = 0;
      for (std::size_t column = 0; column < fields.size(); ++column) {
        const auto comma = line.find(',', begin);
        if ((column < 3) != (comma != std::string::npos))
          throw std::runtime_error("invalid input source field count");
        const auto end = comma == std::string::npos ? line.size() : comma;
        if (end == begin) throw std::runtime_error("empty input source field");
        const std::uint64_t limit = column < 2
            ? std::numeric_limits<std::uint64_t>::max()
            : std::numeric_limits<std::uint32_t>::max();
        for (std::size_t i = begin; i < end; ++i) {
          if (line[i] < '0' || line[i] > '9')
            throw std::runtime_error("invalid input source unsigned decimal");
          const unsigned digit = static_cast<unsigned>(line[i] - '0');
          if (fields[column] > (limit - digit) / 10)
            throw std::runtime_error("input source integer overflow");
          fields[column] = fields[column] * 10 + digit;
        }
        begin = end + 1;
      }
      Source source;
      source.ordinal = fields[0];
      source.stamp_ns = fields[1];
      source.rgb_seq = static_cast<std::uint32_t>(fields[2]);
      source.depth_seq = static_cast<std::uint32_t>(fields[3]);
      if (source.ordinal != sources_.size() + 1 ||
          !source.stamp_ns || source.stamp_ns > std::uint64_t(std::numeric_limits<std::int64_t>::max()) ||
          (!sources_.empty() && source.stamp_ns <= sources_.back().stamp_ns) ||
          !rgb_sequences.insert(source.rgb_seq).second ||
          !depth_sequences.insert(source.depth_seq).second)
        throw std::runtime_error("invalid/duplicate input source identity/order");
      if (sources_.size() == MaxFrames)
        throw std::runtime_error("input count exceeds declared recorder capacity");
      sources_.push_back(source);
    }
    if (input.bad()) throw std::runtime_error("failed to read frozen input index");
    if (sources_.empty()) throw std::runtime_error("empty frozen input index");
    // Freeze capacities before images arrive. Never grow a vector on overflow.
    frames_.reserve(sources_.size());
    events_.reserve(std::min(std::size_t(MaxEvents), sources_.size() * 10 + 16));
    counts_.resize(sources_.size());
    if (L0RetainedBytes() > MaxRetainedBytes)
      throw std::runtime_error("recorder would exceed the declared L0 8 MiB budget");
    if (detail_windows_) details_.reserve(DetailWindowDefinition().expected);
    if (DetailRetainedBytes() > MaxDetailRetainedBytes)
      throw std::runtime_error("recorder would exceed the declared L1 4 MiB budget");
  }
  Source Find(std::uint64_t stamp_ns, std::uint32_t seq, bool rgb) {
    const auto it = std::lower_bound(sources_.begin(), sources_.end(), stamp_ns,
        [](const Source& source, std::uint64_t stamp) { return source.stamp_ns < stamp; });
    if (it == sources_.end() || it->stamp_ns != stamp_ns ||
        (rgb ? it->rgb_seq : it->depth_seq) != seq) {
      ++identity_errors_;
      Source missing;
      missing.stamp_ns = stamp_ns;
      missing.rgb_seq = rgb ? seq : 0;
      return missing;
    }
    return *it;
  }
  void IdentityError() { ++identity_errors_; }
  void Record(EventKind kind, const Source& source, std::uint32_t message_seq,
              std::uint64_t receipt_ros_ns = 0, std::uint64_t steady_ns = 0) {
    if (source.ordinal && source.ordinal <= counts_.size())
      ++counts_[source.ordinal - 1][static_cast<std::size_t>(kind)];
    if (events_.size() == events_.capacity()) { ++event_overflow_; return; }
    events_.push_back({source.stamp_ns, steady_ns ? steady_ns : Now(), receipt_ros_ns,
                       source.rgb_seq, message_seq, kind});
  }
  void Complete(Frame frame) {
    if (frames_.size() == frames_.capacity()) {
      ++frame_overflow_;
      if (core_l0_enabled_ && frame.track_returned) ++core_l0_records_dropped_;
      return;
    }
    if (core_l0_enabled_ && frame.track_returned && (!frame.light.l0_collected ||
        !frame.light.observation.provided ||
        frame.light.observation.source_ordinal != frame.source.ordinal ||
        frame.light.observation.source_stamp_ns != std::int64_t(frame.source.stamp_ns)))
      ++context_errors_;
    if (frame.callback_exception) ++callback_errors_;
    Record(EventKind::CallbackReturn, frame.source, frame.source.rgb_seq);
    frames_.push_back(frame);
    Frame& stored = frames_.back();
    stored.return_ns = Now();
    stored.callback_complete_ms = (stored.return_ns - stored.enter_ns) * 1e-6;
  }
  void RecordDetail(const ORB_SLAM3::RgbdTimingRecord& record) {
    if (!DetailWindow(record.observation.source_ordinal)) return;
    if (!record.l0_collected || !record.observation.provided ||
        !record.observation.detail_window || record.observation.source_stamp_ns !=
          std::int64_t(sources_[record.observation.source_ordinal-1].stamp_ns)) {
      ++context_errors_; return;
    }
    if (details_.size() == details_.capacity()) { ++detail_overflow_; return; }
    details_.push_back(record);
  }
  void SettleUnpaired() {
    if (settled_) return;
    settled_ = true;
    for (std::size_t i = 0; i < sources_.size(); ++i) {
      const auto paired = Count(i, EventKind::Paired);
      const auto rgb_accounted = paired + Count(i, EventKind::SyncDropRgb);
      const auto depth_accounted = paired + Count(i, EventKind::SyncDropDepth);
      for (auto n = rgb_accounted; n < Count(i, EventKind::ReceivedRgb); ++n)
        Record(EventKind::UnmatchedRgb, sources_[i], sources_[i].rgb_seq);
      for (auto n = depth_accounted; n < Count(i, EventKind::ReceivedDepth); ++n)
        Record(EventKind::UnmatchedDepth, sources_[i], sources_[i].depth_seq);
    }
  }
  std::size_t ExpectedInputs() const { return sources_.size(); }
  bool CoreL0Enabled() const { return core_l0_enabled_; }
  std::uint64_t CoreL0RecordsDropped() const { return core_l0_records_dropped_; }
  std::uint64_t CoreL0RecordsMissing() const {
    std::uint64_t returned = 0, retained = 0;
    for (std::size_t i=0; i<sources_.size(); ++i)
      returned += Count(i, EventKind::ProcessedReturn);
    for (const auto& frame : frames_) retained += (frame.track_returned && frame.light.l0_collected) ? 1 : 0;
    return returned > retained ? returned - retained : 0;
  }
  bool InputComplete() const {
    for (std::size_t i = 0; i < sources_.size(); ++i)
      for (auto event : {EventKind::ReceivedRgb, EventKind::ReceivedDepth,
                        EventKind::Paired, EventKind::ProcessedEnter,
                        EventKind::ProcessedReturn, EventKind::CallbackReturn})
        if (Count(i, event) != 1) return false;
    return identity_errors_ == 0;
  }
  bool DetailWindow(std::uint64_t ordinal) const {
    if (!detail_windows_ || !ordinal || ordinal > sources_.size()) return false;
    const auto window = DetailWindowDefinition();
    return ordinal <= window.first_end || ordinal >= window.last_begin;
  }
  void ObserveMaps(std::uint64_t maps) { max_maps_ = std::max(max_maps_, maps); }
  std::uint64_t MaxMaps() const { return max_maps_; }
  bool Write(const std::string& directory) const {
    std::ofstream frames(directory + "/v2_frames.csv");
    std::ofstream events(directory + "/v2_events.csv");
    std::ofstream sources(directory + "/v2_source_status.csv");
    std::ofstream status(directory + "/v2_recorder.json");
    std::ofstream light(directory + "/core_frame_timing.csv");
    std::ofstream detail(directory + "/core_detail_timing.csv");
    if (!frames || !events || !sources || !status || !light || !detail) return false;
    WriteDetailHeader(detail);
    for (const auto& record : details_) WriteDetailRow(detail, record);
    frames << "source_ordinal,source_stamp_ns,rgb_seq,state_after,pose_valid,pose_fresh,pose_emitted,callback_complete_ms,track_call_ms,enter_ns,return_ns,track_called,track_returned,state_publications,callback_exception\n" << std::setprecision(12);
    WriteLightHeader(light);
    for (const auto& f : frames_) {
      frames << f.source.ordinal << ',' << f.source.stamp_ns << ',' << f.source.rgb_seq << ','
             << f.state_after << ',' << f.pose_valid << ',' << f.pose_fresh << ','
             << f.pose_emitted << ',' << f.callback_complete_ms << ',' << f.track_call_ms
             << ',' << f.enter_ns << ',' << f.return_ns << ',' << f.track_called
             << ',' << f.track_returned << ',' << f.state_publications << ',' << f.callback_exception << '\n';
      if (f.track_returned && f.light.l0_collected) WriteLightRow(light, f.light.Expand());
    }
    events << "event,source_stamp_ns,rgb_seq,message_seq,steady_ns,receipt_ros_ns\n";
    static const char* const names[] = {"received_rgb", "received_depth", "paired",
      "processed_enter", "processed_return", "state_output", "pose_publish",
      "sync_drop_rgb", "sync_drop_depth", "callback_return", "late_rgb", "late_depth",
      "unmatched_rgb_at_seal", "unmatched_depth_at_seal"};
    for (const auto& e : events_)
      events << names[static_cast<int>(e.kind)] << ',' << e.stamp_ns << ',' << e.rgb_seq
             << ',' << e.message_seq << ',' << e.steady_ns << ',' << e.receipt_ros_ns << '\n';
    sources << "source_ordinal,source_stamp_ns,rgb_seq,depth_seq,rgb_filter_callback_seen,depth_filter_callback_seen,paired,track_entered,track_returned,callback_returned,state_outputs,raw_pose_outputs,sync_drop_rgb,sync_drop_depth,unmatched_rgb,unmatched_depth,late_rgb,late_depth\n";
    for (std::size_t i = 0; i < sources_.size(); ++i) {
      const auto& s = sources_[i];
      sources << s.ordinal << ',' << s.stamp_ns << ',' << s.rgb_seq << ',' << s.depth_seq;
      for (auto event : {EventKind::ReceivedRgb, EventKind::ReceivedDepth, EventKind::Paired,
          EventKind::ProcessedEnter, EventKind::ProcessedReturn, EventKind::CallbackReturn,
          EventKind::StateOutput, EventKind::PosePublish, EventKind::SyncDropRgb,
          EventKind::SyncDropDepth, EventKind::UnmatchedRgb, EventKind::UnmatchedDepth,
          EventKind::LateRgb, EventKind::LateDepth}) sources << ',' << Count(i, event);
      sources << '\n';
    }
    status << "{\"frame_overflow\":" << frame_overflow_
           << ",\"event_overflow\":" << event_overflow_
           << ",\"identity_errors\":" << identity_errors_
           << ",\"context_errors\":" << context_errors_
           << ",\"callback_errors\":" << callback_errors_
           << ",\"core_l0_records_dropped\":" << CoreL0RecordsDropped()
           << ",\"core_l0_records_missing\":" << CoreL0RecordsMissing()
           << ",\"retained_bytes\":" << RetainedBytes()
           << ",\"l0_retained_bytes\":" << L0RetainedBytes()
           << ",\"l0_retained_budget_bytes\":" << MaxRetainedBytes
           << ",\"detail_retained_bytes\":" << DetailRetainedBytes()
           << ",\"detail_retained_budget_bytes\":" << MaxDetailRetainedBytes
           << ",\"detail_capacity\":" << details_.capacity()
           << ",\"detail_records\":" << details_.size()
           << ",\"detail_expected_window_records\":" << (detail_windows_ ? DetailWindowDefinition().expected : 0)
           << ",\"detail_overflow\":" << detail_overflow_
           << ",\"detail_window_first_end_ordinal\":" << DetailWindowDefinition().first_end
           << ",\"detail_window_last_begin_ordinal\":" << DetailWindowDefinition().last_begin
           << ",\"workspace_total_ceiling_bytes\":33554432"
           << ",\"lock_allocation_diagnostics\":\"not_measured\""
           << ",\"expected_inputs\":" << sources_.size()
           << ",\"frame_capacity\":" << frames_.capacity()
           << ",\"event_capacity\":" << events_.capacity()
           << ",\"detail_windows\":" << (detail_windows_ ? "true" : "false")
           << ",\"core_l0_requested\":" << (core_l0_enabled_ ? "true" : "false")
           << ",\"receipt_scope\":\"MessageEvent ROS receipt and monotonic filter_callback_enter observed after successful deserialization; queue entry/exit and pre-callback drops unobserved\"}\n";
    frames.close(); events.close(); sources.close(); status.close(); light.close(); detail.close();
    return frames.good() && events.good() && sources.good() && status.good() && light.good() && detail.good();
  }
  std::size_t RetainedBytes() const { return L0RetainedBytes() + DetailRetainedBytes(); }
  std::size_t DetailRetainedBytes() const {
    return details_.capacity() * sizeof(ORB_SLAM3::RgbdTimingRecord);
  }
  std::size_t L0RetainedBytes() const {
    return sources_.capacity() * sizeof(Source) + frames_.capacity() * sizeof(Frame) +
           events_.capacity() * sizeof(Event) + counts_.capacity() * sizeof(Counts);
  }
  bool Valid() const {
    return !frame_overflow_ && !event_overflow_ && !detail_overflow_ && !identity_errors_ &&
           !context_errors_ && !callback_errors_;
  }
 private:
  struct Window { std::size_t first_end, last_begin, expected; };
  Window DetailWindowDefinition() const {
    // One definition for selection, reserve and export. The disabled flag
    // gates collection/expected count; bounds still describe the input window.
    if (sources_.size() == 2910) return {0, 2621, 290};
    const auto edge = sources_.size() / 20;
    return {edge, sources_.size() - edge + 1, 2 * edge};
  }
  std::uint32_t Count(std::size_t i, EventKind kind) const {
    return counts_[i][static_cast<std::size_t>(kind)];
  }
  std::vector<Source> sources_;
  std::vector<Frame> frames_;
  std::vector<Event> events_;
  std::vector<Counts> counts_;
  std::vector<ORB_SLAM3::RgbdTimingRecord> details_;
  std::uint64_t frame_overflow_ = 0, event_overflow_ = 0, identity_errors_ = 0;
  std::uint64_t context_errors_ = 0, callback_errors_ = 0, max_maps_ = 0, detail_overflow_ = 0;
  std::uint64_t core_l0_records_dropped_ = 0;
  bool detail_windows_ = false, core_l0_enabled_ = true, settled_ = false;
};
}  // namespace rgbd_v2
#endif
