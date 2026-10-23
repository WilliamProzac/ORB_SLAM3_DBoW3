/**
 * RGB-D-inertial ROS entry point for the S316 camera on RDK X5.
 *
 * RGB and depth use exact header timestamp synchronization. The S316 SDK
 * defines t_imu = t_camera + imu_time_shift, so IMU samples are corrected to
 * the camera clock before being passed to ORB-SLAM3.
 */

#include "common.h"

#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <vector>

namespace
{
double MillisecondsSince(const std::chrono::steady_clock::time_point &start)
{
    return std::chrono::duration_cast<
               std::chrono::duration<double, std::milli>>(
               std::chrono::steady_clock::now() - start)
        .count();
}

bool EnsureDirectory(const std::string &path)
{
    if (path.empty())
        return false;
    std::string current;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        current.push_back(path[i]);
        if (path[i] != '/' || current.size() == 1)
            continue;
        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
            return false;
    }
    return ::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

bool FiniteImu(const sensor_msgs::Imu &message)
{
    return std::isfinite(message.angular_velocity.x) &&
           std::isfinite(message.angular_velocity.y) &&
           std::isfinite(message.angular_velocity.z) &&
           std::isfinite(message.linear_acceleration.x) &&
           std::isfinite(message.linear_acceleration.y) &&
           std::isfinite(message.linear_acceleration.z);
}

struct TimedImuMessage
{
    sensor_msgs::ImuConstPtr message;
    double corrected_stamp = 0.0;
};

struct RgbdPacket
{
    sensor_msgs::ImageConstPtr rgb;
    sensor_msgs::ImageConstPtr depth;
    std::chrono::steady_clock::time_point enqueue_time;
};

struct FrameTimingSample
{
    uint32_t rgb_seq = 0;
    uint32_t depth_seq = 0;
    int64_t rgb_seq_gap = 0;
    double callback_start_ms = 0.0;
    double callback_interval_ms = 0.0;
    double header_period_ms = 0.0;
    double rgb_depth_stamp_delta_ms = 0.0;
    double queue_wait_ms = 0.0;
    double cv_bridge_ms = 0.0;
    double track_call_ms = 0.0;
    double publish_ms = 0.0;
    double callback_total_ms = 0.0;
    std::size_t imu_samples = 0;
    double last_imu_delta_ms = 0.0;
    bool imu_initialized = false;
    double imu_init_time_s = 0.0;
    std::size_t image_queue_depth = 0;
    std::size_t imu_queue_depth = 0;
    ORB_SLAM3::RgbdTimingRecord core;
};

template <typename T, std::size_t N>
void WriteLevelArray(std::ostream &output, const std::array<T, N> &values,
                     int level_count)
{
    for (int level = 0; level < level_count; ++level)
    {
        if (level != 0)
            output << '|';
        output << values[level];
    }
}

class RgbdInertialGrabber
{
public:
    RgbdInertialGrabber(ORB_SLAM3::System *slam, double imu_time_shift,
                        std::size_t image_queue_capacity,
                        std::size_t imu_queue_capacity, bool timing_enabled,
                        const std::string &timing_output_dir)
        : slam_(slam), imu_time_shift_(imu_time_shift),
          image_queue_capacity_(image_queue_capacity),
          imu_queue_capacity_(imu_queue_capacity),
          timing_enabled_(timing_enabled),
          timing_output_dir_(timing_output_dir)
    {
        timing_samples_.reserve(20000);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr &rgb,
                  const sensor_msgs::ImageConstPtr &depth)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        RgbdPacket packet;
        packet.rgb = rgb;
        packet.depth = depth;
        packet.enqueue_time = std::chrono::steady_clock::now();
        image_packets_.push_back(packet);
        if (image_packets_.size() > image_queue_capacity_)
        {
            image_packets_.pop_front();
            ++dropped_image_pairs_;
            ROS_WARN_THROTTLE(2.0,
                              "RGB-D-inertial image queue overflow; dropping oldest pair");
        }
        condition_.notify_one();
    }

    void GrabImu(const sensor_msgs::ImuConstPtr &message)
    {
        if (!FiniteImu(*message))
        {
            ++non_finite_imu_;
            ROS_WARN_THROTTLE(2.0, "Discarding non-finite S316 IMU measurement");
            return;
        }
        const double corrected_stamp =
            message->header.stamp.toSec() - imu_time_shift_;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!imu_messages_.empty() &&
            corrected_stamp <= imu_messages_.back().corrected_stamp)
        {
            ++non_monotonic_imu_;
            ROS_WARN_THROTTLE(2.0,
                              "Discarding non-monotonic IMU timestamp after clock correction");
            return;
        }
        imu_messages_.push_back(TimedImuMessage{message, corrected_stamp});
        if (imu_messages_.size() > imu_queue_capacity_)
        {
            imu_messages_.pop_front();
            ++dropped_imu_;
            ROS_WARN_THROTTLE(2.0, "S316 IMU queue overflow; dropping oldest sample");
        }
        condition_.notify_one();
    }

    void Run()
    {
        while (running_.load() && ros::ok())
        {
            RgbdPacket packet;
            std::vector<ORB_SLAM3::IMU::Point> imu;
            double last_imu_stamp = 0.0;
            std::size_t image_queue_depth = 0;
            std::size_t imu_queue_depth = 0;
            if (!WaitForPacket(packet, imu, last_imu_stamp,
                               image_queue_depth, imu_queue_depth))
                continue;
            Process(packet, imu, last_imu_stamp,
                    image_queue_depth, imu_queue_depth);
        }
        ROS_INFO("RGB-D-inertial worker stopped: frames=%zu imu=%zu "
                 "dropped_pairs=%zu frames_without_imu=%zu dropped_imu=%zu "
                 "non_monotonic_imu=%zu non_finite_imu=%zu",
                 processed_frames_, processed_imu_, dropped_image_pairs_,
                 frames_without_imu_, dropped_imu_, non_monotonic_imu_,
                 non_finite_imu_);
    }

    void Stop()
    {
        running_.store(false);
        condition_.notify_all();
    }

    bool WriteTimingCsv() const;

private:
    bool WaitForPacket(RgbdPacket &packet,
                       std::vector<ORB_SLAM3::IMU::Point> &imu,
                       double &last_imu_stamp,
                       std::size_t &image_queue_depth,
                       std::size_t &imu_queue_depth)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(20), [this] {
            return !running_.load() ||
                   (!image_packets_.empty() && !imu_messages_.empty());
        });
        while (running_.load() && ros::ok())
        {
            if (image_packets_.empty() || imu_messages_.empty())
                return false;
            const double image_stamp =
                image_packets_.front().rgb->header.stamp.toSec();
            if (has_last_image_stamp_ && image_stamp <= last_image_stamp_)
            {
                image_packets_.pop_front();
                ++dropped_image_pairs_;
                ROS_WARN_THROTTLE(2.0, "Dropping non-monotonic RGB-D pair");
                continue;
            }
            if (imu_messages_.back().corrected_stamp < image_stamp)
            {
                condition_.wait_for(lock, std::chrono::milliseconds(10));
                continue;
            }

            packet = image_packets_.front();
            image_packets_.pop_front();
            while (!imu_messages_.empty() &&
                   imu_messages_.front().corrected_stamp <= image_stamp)
            {
                const TimedImuMessage timed = imu_messages_.front();
                imu_messages_.pop_front();
                const sensor_msgs::Imu &sample = *timed.message;
                imu.emplace_back(
                    cv::Point3f(static_cast<float>(sample.linear_acceleration.x),
                                static_cast<float>(sample.linear_acceleration.y),
                                static_cast<float>(sample.linear_acceleration.z)),
                    cv::Point3f(static_cast<float>(sample.angular_velocity.x),
                                static_cast<float>(sample.angular_velocity.y),
                                static_cast<float>(sample.angular_velocity.z)),
                    timed.corrected_stamp);
                last_imu_stamp = timed.corrected_stamp;
            }
            if (imu.empty())
            {
                ++frames_without_imu_;
                ROS_WARN_THROTTLE(2.0,
                                  "Dropping RGB-D pair with no IMU samples at %.9f",
                                  image_stamp);
                packet = RgbdPacket();
                continue;
            }
            last_image_stamp_ = image_stamp;
            has_last_image_stamp_ = true;
            image_queue_depth = image_packets_.size();
            imu_queue_depth = imu_messages_.size();
            return true;
        }
        return false;
    }

    void Process(const RgbdPacket &packet,
                 const std::vector<ORB_SLAM3::IMU::Point> &imu,
                 double last_imu_stamp, std::size_t image_queue_depth,
                 std::size_t imu_queue_depth)
    {
        const auto callback_start = std::chrono::steady_clock::now();
        FrameTimingSample timing;
        timing.rgb_seq = packet.rgb->header.seq;
        timing.depth_seq = packet.depth->header.seq;
        timing.rgb_depth_stamp_delta_ms =
            (packet.depth->header.stamp - packet.rgb->header.stamp).toSec() *
            1000.0;
        timing.queue_wait_ms =
            std::chrono::duration<double, std::milli>(callback_start -
                                                       packet.enqueue_time)
                .count();
        timing.imu_samples = imu.size();
        timing.last_imu_delta_ms =
            (last_imu_stamp - packet.rgb->header.stamp.toSec()) * 1000.0;
        timing.image_queue_depth = image_queue_depth;
        timing.imu_queue_depth = imu_queue_depth;
        if (!have_previous_callback_)
        {
            first_callback_time_ = callback_start;
            previous_callback_time_ = callback_start;
            previous_header_stamp_ = packet.rgb->header.stamp;
            previous_rgb_seq_ = packet.rgb->header.seq;
            have_previous_callback_ = true;
        }
        else
        {
            timing.callback_interval_ms =
                std::chrono::duration<double, std::milli>(
                    callback_start - previous_callback_time_)
                    .count();
            timing.header_period_ms =
                (packet.rgb->header.stamp - previous_header_stamp_).toSec() *
                1000.0;
            timing.rgb_seq_gap =
                static_cast<int64_t>(packet.rgb->header.seq) -
                static_cast<int64_t>(previous_rgb_seq_) - 1;
            previous_callback_time_ = callback_start;
            previous_header_stamp_ = packet.rgb->header.stamp;
            previous_rgb_seq_ = packet.rgb->header.seq;
        }
        timing.callback_start_ms =
            std::chrono::duration<double, std::milli>(callback_start -
                                                       first_callback_time_)
                .count();

        if (packet.rgb->width != packet.depth->width ||
            packet.rgb->height != packet.depth->height)
        {
            ROS_ERROR_THROTTLE(2.0,
                               "RGB/depth size mismatch: rgb=%ux%u depth=%ux%u",
                               packet.rgb->width, packet.rgb->height,
                               packet.depth->width, packet.depth->height);
            return;
        }
        if (packet.depth->encoding !=
                sensor_msgs::image_encodings::TYPE_32FC1 &&
            packet.depth->encoding !=
                sensor_msgs::image_encodings::TYPE_16UC1)
        {
            ROS_ERROR_THROTTLE(2.0, "Unsupported depth encoding: %s",
                               packet.depth->encoding.c_str());
            return;
        }

        cv_bridge::CvImageConstPtr rgb;
        cv_bridge::CvImageConstPtr depth;
        const auto bridge_start = std::chrono::steady_clock::now();
        try
        {
            rgb = cv_bridge::toCvShare(packet.rgb);
            depth = cv_bridge::toCvShare(packet.depth);
        }
        catch (const cv_bridge::Exception &error)
        {
            ROS_ERROR("cv_bridge exception: %s", error.what());
            return;
        }
        timing.cv_bridge_ms = MillisecondsSince(bridge_start);

        const auto track_start = std::chrono::steady_clock::now();
        const Sophus::SE3f Tcw = slam_->TrackRGBD(
            rgb->image, depth->image, packet.rgb->header.stamp.toSec(), imu);
        timing.track_call_ms = MillisecondsSince(track_start);
        timing.core = slam_->GetLastRgbdTiming();
        const int tracking_state = slam_->GetTrackingState();
        const bool tracking_ok = tracking_state == ORB_SLAM3::Tracking::OK ||
                                 tracking_state == ORB_SLAM3::Tracking::OK_KLT;
        const bool pose_ok = tracking_ok && Tcw.translation().allFinite() &&
                             Tcw.rotationMatrix().allFinite();
        timing.imu_init_time_s = slam_->GetTimeFromIMUInit();
        timing.imu_initialized = slam_->IsImuInitialized();

        const auto publish_start = std::chrono::steady_clock::now();
        publish_pose_and_odometry(Tcw, packet.rgb->header.stamp, pose_ok);
        timing.publish_ms = MillisecondsSince(publish_start);
        timing.callback_total_ms = MillisecondsSince(callback_start);
        if (timing_enabled_)
            timing_samples_.push_back(timing);

        ++processed_frames_;
        processed_imu_ += imu.size();
        if (timing.imu_initialized && !reported_imu_initialization_)
        {
            reported_imu_initialization_ = true;
            ROS_INFO("ORB-SLAM3 RGB-D IMU initialization completed at frame %zu "
                     "(initialization interval %.3f s)",
                     processed_frames_, timing.imu_init_time_s);
        }
        if (processed_frames_ == 1 || processed_frames_ % 30 == 0)
        {
            ROS_INFO("RGB-D-inertial progress: frames=%zu imu=%zu state=%d "
                     "imu_initialized=%s track=%.1f ms wait=%.1f ms",
                     processed_frames_, processed_imu_, tracking_state,
                     timing.imu_initialized ? "true" : "false",
                     timing.track_call_ms, timing.queue_wait_ms);
        }
    }

    ORB_SLAM3::System *slam_;
    const double imu_time_shift_;
    const std::size_t image_queue_capacity_;
    const std::size_t imu_queue_capacity_;
    const bool timing_enabled_;
    const std::string timing_output_dir_;
    std::atomic<bool> running_{true};
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<RgbdPacket> image_packets_;
    std::deque<TimedImuMessage> imu_messages_;
    std::vector<FrameTimingSample> timing_samples_;
    bool have_previous_callback_ = false;
    bool has_last_image_stamp_ = false;
    bool reported_imu_initialization_ = false;
    uint32_t previous_rgb_seq_ = 0;
    double last_image_stamp_ = 0.0;
    ros::Time previous_header_stamp_;
    std::chrono::steady_clock::time_point first_callback_time_;
    std::chrono::steady_clock::time_point previous_callback_time_;
    std::size_t processed_frames_ = 0;
    std::size_t processed_imu_ = 0;
    std::size_t dropped_image_pairs_ = 0;
    std::size_t frames_without_imu_ = 0;
    std::size_t dropped_imu_ = 0;
    std::size_t non_monotonic_imu_ = 0;
    std::size_t non_finite_imu_ = 0;
};

bool RgbdInertialGrabber::WriteTimingCsv() const
{
    const std::string filename = timing_output_dir_ + "/rgbd_frame_timing.csv";
    std::ofstream output(filename.c_str());
    if (!output.is_open())
        return false;
    output << "rgb_seq,depth_seq,rgb_seq_gap,image_timestamp,frame_id,"
              "state_before,state_after,keypoints,valid_depths,tracked_inliers,"
              "lm_queue_before,lm_queue_after,inserted_keyframe,"
              "callback_start_ms,callback_interval_ms,header_period_ms,"
              "rgb_depth_stamp_delta_ms,queue_wait_ms,cv_bridge_ms,"
              "track_call_ms,publish_ms,callback_total_ms,imu_samples,"
              "last_imu_delta_ms,imu_initialized,imu_init_time_s,"
              "image_queue_depth,imu_queue_depth,system_clone_ms,"
              "system_resize_ms,mode_mutex_wait_ms,mode_change_ms,"
              "reset_mutex_wait_ms,reset_ms,grab_rgbd_ms,state_copy_ms,"
              "system_total_ms,gray_convert_ms,depth_convert_ms,"
              "frame_construct_ms,orb_extract_ms,orb_pyramid_ms,"
              "orb_keypoints_octree_ms,orb_orientation_ms,orb_blur_ms,"
              "orb_descriptor_ms,orb_result_assembly_ms,orb_fast_initial_ms,"
              "orb_fast_fallback_ms,orb_octree_ms,orb_copy_ms,"
              "orb_gaussian_ms,orb_output_hash,orb_mono_index,"
              "orb_level_count,orb_level_fast_initial_ms,"
              "orb_level_fast_fallback_ms,orb_level_octree_ms,"
              "orb_level_orientation_ms,orb_level_copy_ms,"
              "orb_level_gaussian_ms,orb_level_descriptor_ms,"
              "orb_level_candidates,orb_level_fallback_cells,"
              "orb_level_octree_splits,rgbd_depth_assoc_ms,"
              "track_internal_ms,map_mutex_wait_ms,initialization_ms,"
              "reference_kf_ms,motion_model_ms,relocalization_ms,local_map_ms,"
              "need_new_kf_ms,create_new_kf_ms\n";
    output << std::fixed << std::setprecision(6);
    for (const FrameTimingSample &sample : timing_samples_)
    {
        const ORB_SLAM3::RgbdTimingRecord &r = sample.core;
        output << sample.rgb_seq << ',' << sample.depth_seq << ','
               << sample.rgb_seq_gap << ',' << r.image_timestamp << ','
               << r.frame_id << ',' << r.state_before << ',' << r.state_after
               << ',' << r.keypoints << ',' << r.valid_depths << ','
               << r.tracked_inliers << ',' << r.local_mapping_queue_before
               << ',' << r.local_mapping_queue_after << ','
               << r.inserted_keyframe << ',' << sample.callback_start_ms << ','
               << sample.callback_interval_ms << ',' << sample.header_period_ms
               << ',' << sample.rgb_depth_stamp_delta_ms << ','
               << sample.queue_wait_ms << ',' << sample.cv_bridge_ms << ','
               << sample.track_call_ms << ',' << sample.publish_ms << ','
               << sample.callback_total_ms << ',' << sample.imu_samples << ','
               << sample.last_imu_delta_ms << ','
               << (sample.imu_initialized ? 1 : 0) << ','
               << sample.imu_init_time_s << ',' << sample.image_queue_depth
               << ',' << sample.imu_queue_depth << ',' << r.system_clone_ms
               << ',' << r.system_resize_ms << ',' << r.mode_mutex_wait_ms
               << ',' << r.mode_change_ms << ',' << r.reset_mutex_wait_ms
               << ',' << r.reset_ms << ',' << r.grab_rgbd_ms << ','
               << r.state_copy_ms << ',' << r.system_total_ms << ','
               << r.gray_convert_ms << ',' << r.depth_convert_ms << ','
               << r.frame_construct_ms << ',' << r.orb_extract_ms << ','
               << r.orb_pyramid_ms << ',' << r.orb_keypoints_octree_ms << ','
               << r.orb_orientation_ms << ',' << r.orb_blur_ms << ','
               << r.orb_descriptor_ms << ',' << r.orb_result_assembly_ms << ','
               << r.orb_fast_initial_ms << ',' << r.orb_fast_fallback_ms << ','
               << r.orb_octree_ms << ',' << r.orb_copy_ms << ','
               << r.orb_gaussian_ms << ',' << r.orb_output_hash << ','
               << r.orb_mono_index << ',' << r.orb_level_count << ',';
        WriteLevelArray(output, r.orb_level_fast_initial_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_fast_fallback_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_octree_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_orientation_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_copy_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_gaussian_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_descriptor_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_candidates, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_fallback_cells, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_octree_splits, r.orb_level_count);
        output << ',' << r.rgbd_depth_assoc_ms << ',' << r.track_internal_ms
               << ',' << r.map_mutex_wait_ms << ',' << r.initialization_ms
               << ',' << r.reference_kf_ms << ',' << r.motion_model_ms << ','
               << r.relocalization_ms << ',' << r.local_map_ms << ','
               << r.need_new_kf_ms << ',' << r.create_new_kf_ms << '\n';
    }
    return output.good();
}
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Info);
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    std::string vocabulary_file;
    std::string settings_file;
    std::string rgb_topic;
    std::string depth_topic;
    std::string imu_topic;
    std::string timing_output_dir;
    bool enable_pangolin = false;
    bool timing_enabled = true;
    double imu_time_shift = 0.016355351;
    int image_queue_capacity = 50;
    int imu_queue_capacity = 4000;
    private_node.param<std::string>("voc_file", vocabulary_file, "file_not_set");
    private_node.param<std::string>("settings_file", settings_file, "file_not_set");
    private_node.param<std::string>("rgb_topic", rgb_topic,
                                    "/camera/infra1/image_raw");
    private_node.param<std::string>("depth_topic", depth_topic,
                                    "/camera/depth/image_raw");
    private_node.param<std::string>("imu_topic", imu_topic, "/camera/imu/data");
    private_node.param<std::string>("timing_output_dir", timing_output_dir,
                                    "/tmp/orb_slam3_rgbd_inertial_timing");
    private_node.param<bool>("enable_pangolin", enable_pangolin, false);
    private_node.param<bool>("timing_enabled", timing_enabled, true);
    private_node.param<double>("imu_time_shift", imu_time_shift, 0.016355351);
    private_node.param<int>("image_queue_capacity", image_queue_capacity, 50);
    private_node.param<int>("imu_queue_capacity", imu_queue_capacity, 4000);

    if (vocabulary_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file");
        return 1;
    }
    if (image_queue_capacity < 2 || imu_queue_capacity < 10)
    {
        ROS_ERROR("Invalid RGB-D-inertial queue parameters");
        return 1;
    }

    sensor_type = ORB_SLAM3::System::IMU_RGBD;
    pSLAM = new ORB_SLAM3::System(vocabulary_file, settings_file, sensor_type,
                                  enable_pangolin);
    RgbdInertialGrabber grabber(
        pSLAM, imu_time_shift, static_cast<std::size_t>(image_queue_capacity),
        static_cast<std::size_t>(imu_queue_capacity), timing_enabled,
        timing_output_dir);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node, rgb_topic, 20);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node, depth_topic, 20);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>
        sync(rgb_sub, depth_sub, 20);
    sync.registerCallback(
        boost::bind(&RgbdInertialGrabber::GrabRGBD, &grabber, _1, _2));
    ros::Subscriber imu_sub = node.subscribe(
        imu_topic, 2000, &RgbdInertialGrabber::GrabImu, &grabber,
        ros::TransportHints().tcpNoDelay());

    setup_publishers(node, ros::this_node::getName());
    setup_services(node, ros::this_node::getName());
    ROS_INFO("S316 RGB-D-inertial input: rgb=%s depth=%s imu=%s (exact RGB-D sync)",
             rgb_topic.c_str(), depth_topic.c_str(), imu_topic.c_str());
    ROS_INFO("S316 IMU clock correction: camera_time = imu_time - %.9f s",
             imu_time_shift);

    std::thread worker(&RgbdInertialGrabber::Run, &grabber);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    grabber.Stop();
    if (worker.joinable())
        worker.join();

    pSLAM->Shutdown();
    if (timing_enabled)
    {
        if (!EnsureDirectory(timing_output_dir))
            ROS_ERROR("Cannot create timing output directory: %s",
                      timing_output_dir.c_str());
        else
        {
            const bool frame_ok = grabber.WriteTimingCsv();
            const bool local_mapping_ok =
                pSLAM->WriteDetailedTimingCsv(timing_output_dir);
            ROS_INFO("Timing CSV write: frames=%s local_mapping=%s directory=%s",
                     frame_ok ? "ok" : "failed",
                     local_mapping_ok ? "ok" : "failed",
                     timing_output_dir.c_str());
        }
    }
    ROS_INFO("Final RGB-D-inertial map: map_id=%lu atlas_maps=%lu "
             "keyframes=%lu mappoints=%lu map_change_idx=%d",
             pSLAM->GetCurrentMapId(), pSLAM->GetAtlasMapCount(),
             pSLAM->GetCurrentMapKeyFrames(), pSLAM->GetCurrentMapPoints(),
             pSLAM->GetCurrentMapChangeIndex());
    delete pSLAM;
    pSLAM = nullptr;
    return 0;
}
