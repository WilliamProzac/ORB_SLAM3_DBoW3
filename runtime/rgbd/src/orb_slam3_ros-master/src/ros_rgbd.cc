#include "../../../diagnostics/benchmark.h"
#include "FieldObservation.h"
#include "../../../diagnostics/recorder.h"
#include <ros/message_event.h>
/**
 * ROS RGB-D entry point for the S316 camera on RDK X5.
 */

#include "common.h"
#include "ScaleRgbdExperiment.h"
#include <memory>
static rgbd_scale::Recorder* scale_recorder=nullptr;
#include "RgbdV2Recorder.h"
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <memory>

#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>

#include <cerrno>
#include <cstdio>
#include <stdexcept>
#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

using namespace std;

#include "../../../diagnostics/camera_delivery/bridge.h"
static camera_delivery::Bridge* camera_bridge=nullptr;
static rgbd_v2::Recorder* v2_recorder=nullptr;
static std::uint64_t v2_seal_request_ns=0, v2_cutoff_receipt_ros_ns=0;
static int v2_front_timeout_s=60;
static bool V2Seal(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
 if(!v2_recorder){response.success=false;response.message="v2 disabled";return true;}
 if(!v2_seal_request_ns){
   v2_seal_request_ns=rgbd_v2::Now();
   v2_cutoff_receipt_ros_ns=ros::Time::now().toNSec()+
      std::uint64_t(v2_front_timeout_s)*1000000000ULL;
 }
 response.success=true;
 response.message="source end registered; callback drain continues on the same thread";
 return true;
}
static field::Recorder* field_recorder=nullptr;
static field::Benchmark* field_benchmark=nullptr;
static uint64_t field_ordinal=0,field_previous_stamp=0;
static void FieldInput(bool rgb,const ros::MessageEvent<sensor_msgs::Image const>&event){
 if(v2_recorder){
   const auto& m=event.getMessage();
   const auto source=v2_recorder->Find(m->header.stamp.toNSec(),m->header.seq,rgb);
   const auto receipt=event.getReceiptTime().toNSec();
   v2_recorder->Record(rgb?rgbd_v2::EventKind::ReceivedRgb:rgbd_v2::EventKind::ReceivedDepth,
                       source,m->header.seq,receipt);
   if(v2_cutoff_receipt_ros_ns && receipt>v2_cutoff_receipt_ros_ns)
     v2_recorder->Record(rgb?rgbd_v2::EventKind::LateRgb:rgbd_v2::EventKind::LateDepth,
                         source,m->header.seq,receipt);
 }
 if(field_benchmark)field_benchmark->Input(rgb?0:1,event.getMessage()->header.seq);
 if(!field_recorder)return;
 const uint64_t now=field::Now();const auto&m=event.getMessage();
 uint64_t encoding=m->encoding=="rgb8"?1:m->encoding=="32FC1"?2:m->encoding=="16UC1"?3:0;
 field::Row r{now,m->header.seq,uint16_t(rgb?1:2),31,0,{m->header.stamp.toNSec(),event.getReceiptTime().toNSec(),ros::WallTime::now().toNSec(),(uint64_t(m->width)<<32)|m->height,encoding}};
 field_recorder->Record(r);
}
static void FieldDrop(const sensor_msgs::ImageConstPtr&r,const sensor_msgs::ImageConstPtr&d){
 if(v2_recorder){
   if(r)v2_recorder->Record(rgbd_v2::EventKind::SyncDropRgb,
       v2_recorder->Find(r->header.stamp.toNSec(),r->header.seq,true),r->header.seq);
   if(d)v2_recorder->Record(rgbd_v2::EventKind::SyncDropDepth,
       v2_recorder->Find(d->header.stamp.toNSec(),d->header.seq,false),d->header.seq);
 }
 if(field_benchmark){if(r)++field_benchmark->drops[0];if(d)++field_benchmark->drops[1];}
 if(r) ORB_SLAM3::FieldEmit(5,r->header.seq,0,1,r->header.stamp.toNSec());
 if(d) ORB_SLAM3::FieldEmit(6,d->header.seq,0,1,d->header.stamp.toNSec());
}
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
    for (size_t i = 0; i < path.size(); ++i)
    {
        current.push_back(path[i]);
        if (path[i] != '/' || current.size() == 1)
            continue;
        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
            return false;
    }
    return ::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

struct FrameTimingSample
{
    uint32_t rgb_seq = 0;
    uint32_t depth_seq = 0;
    int64_t rgb_seq_gap = 0;
    double callback_start_ms = 0.0;
    double callback_interval_ms = 0.0;
    double header_period_ms = 0.0;
    double rgb_depth_stamp_delta_ms = 0.0;
    double header_age_ms = 0.0;
    double cv_bridge_ms = 0.0;
    double track_call_ms = 0.0;
    double get_state_ms = 0.0;
    double publish_ms = 0.0;
    double callback_total_ms = 0.0;
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
} // namespace

// This records delivery AFTER roscpp subscription dequeue, never wire arrival.
// Fixed-size stdio buffer; flushed every 256 small records for crash evidence.
class InputDiagnostics
{
public:
    explicit InputDiagnostics(const std::string &directory, bool enabled)
    {
        if (!enabled) return;
        if (EnsureDirectory(directory))
            file_ = std::fopen((directory + "/input_events.csv").c_str(), "w");
        if (!file_) throw std::runtime_error("Cannot open input diagnostics");
        std::setvbuf(file_, buffer_, _IOFBF, sizeof(buffer_));
        std::fprintf(file_, "event,seq,stamp_ns,steady_ns,wall_ns\n");
    }
    ~InputDiagnostics() { if (file_) std::fclose(file_); }
    void Record(const char *event, const sensor_msgs::ImageConstPtr &msg)
    {
        if (!msg || !file_) return;
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        std::fprintf(file_, "%s,%u,%llu,%lld,%llu\n", event, msg->header.seq,
            static_cast<unsigned long long>(msg->header.stamp.toNSec()),
            static_cast<long long>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()),
            static_cast<unsigned long long>(ros::WallTime::now().toNSec()));
        if (++pending_ == 256) { std::fflush(file_); pending_ = 0; }
    }
    void Rgb(const sensor_msgs::ImageConstPtr &msg) { Record("rgb_dequeued", msg); }
    void Depth(const sensor_msgs::ImageConstPtr &msg) { Record("depth_dequeued", msg); }
    void Drop(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
    { Record("sync_drop_rgb", rgb); Record("sync_drop_depth", depth); }
private:
    FILE *file_ = nullptr;
    char buffer_[32768];
    unsigned pending_ = 0;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *slam, bool timing_enabled,
                 const std::string &timing_output_dir)
        : slam_(slam), timing_enabled_(timing_enabled),
          timing_output_dir_(timing_output_dir)
    {
        if (timing_enabled_) timing_samples_.reserve(20000);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msg_rgb,
                  const sensor_msgs::ImageConstPtr &msg_depth);
    bool WriteTimingCsv() const;

private:
    void GrabRGBDBody(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&,
                      rgbd_v2::Frame* frame);
    ORB_SLAM3::System *slam_;
    bool timing_enabled_;
    std::string timing_output_dir_;
    std::vector<FrameTimingSample> timing_samples_;
    bool have_previous_callback_ = false;
    uint32_t previous_rgb_seq_ = 0;
    ros::Time previous_header_stamp_;
    std::chrono::steady_clock::time_point first_callback_time_;
    std::chrono::steady_clock::time_point previous_callback_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Info);
    if (argc > 1)
        ROS_WARN("Arguments supplied via command line are ignored.");

    ros::NodeHandle node_handler;
    const std::string node_name = ros::this_node::getName();

    std::string voc_file;
    std::string settings_file;
    std::string rgb_topic;
    std::string depth_topic;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file,
                                    "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file,
                                    "file_not_set");
    node_handler.param<std::string>(node_name + "/rgb_topic", rgb_topic,
                                    "/camera/infra1/image_raw");
    node_handler.param<std::string>(node_name + "/depth_topic", depth_topic,
                                    "/camera/depth/image_raw");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        return 1;
    }

    bool enable_pangolin = false;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin,
                             false);
    bool timing_enabled = false;
    std::string timing_output_dir;
    node_handler.param<bool>(node_name + "/timing_enabled", timing_enabled,
                             false);
    node_handler.param<std::string>(node_name + "/timing_output_dir",
                                    timing_output_dir,
                                    "/tmp/orb_slam3_rgbd_timing");

    bool scale_enabled=false,scale_detail=true;
    std::string scale_index,scale_directory;
    int scale_front_timeout_s=60;
    node_handler.param<bool>(node_name+"/scale_enabled",scale_enabled,false);
    node_handler.param<bool>(node_name+"/scale_detail",scale_detail,true);
    node_handler.param<std::string>(node_name+"/scale_input_index",scale_index,"");
    node_handler.param<std::string>(node_name+"/scale_output_dir",scale_directory,"");
    node_handler.param<int>(node_name+"/scale_front_timeout_s",scale_front_timeout_s,60);
    std::unique_ptr<rgbd_scale::Recorder> scale_owner;
    if (scale_enabled) {
        if (enable_pangolin || ros::Time::isSimTime() || scale_front_timeout_s<1 ||
            scale_front_timeout_s>600 || !EnsureDirectory(scale_directory)) {
            ROS_ERROR("scale requires viewer off, wall ROS time, valid timeout and output directory");return 2;
        }
        try { scale_owner.reset(new rgbd_scale::Recorder(scale_index,scale_detail)); }
        catch(const std::exception& e) { ROS_ERROR("scale recorder: %s",e.what());return 2; }
        scale_recorder=scale_owner.get();
        timing_enabled=false;
        ORB_SLAM3::SetHeavyDiagnostics(false);
    }

    bool v2_enabled=false;
    std::string v2_index, v2_directory, v2_seal_service;
    int v2_backend_timeout_s=300;
    bool v2_detail_windows=false, v2_collect_core_l0=true;
    node_handler.param<bool>(node_name+"/v2_enabled",v2_enabled,false);
    node_handler.param<std::string>(node_name+"/v2_input_index",v2_index,"");
    node_handler.param<std::string>(node_name+"/v2_output_dir",v2_directory,"");
    node_handler.param<std::string>(node_name+"/v2_seal_service",v2_seal_service,node_name+"/v2_seal");
    node_handler.param<int>(node_name+"/v2_front_timeout_s",v2_front_timeout_s,60);
    node_handler.param<int>(node_name+"/v2_backend_timeout_s",v2_backend_timeout_s,300);
    node_handler.param<bool>(node_name+"/v2_detail_windows",v2_detail_windows,false);
    node_handler.param<bool>(node_name+"/v2_collect_core_l0",v2_collect_core_l0,true);
    std::unique_ptr<rgbd_v2::Recorder> v2_owner;
    if(scale_enabled && v2_enabled){ROS_ERROR("scale and v2 modes are mutually exclusive");return 2;}
    if(scale_enabled) ORB_SLAM3::SetRgbdLightDiagnostics(false);
    if(v2_enabled){
      if(enable_pangolin || ros::Time::isSimTime() || v2_front_timeout_s<1 ||
         v2_front_timeout_s>600 || v2_backend_timeout_s<1 || v2_backend_timeout_s>1200 ||
         !EnsureDirectory(v2_directory)){
        ROS_ERROR("v2 requires viewer off, wall ROS time, valid cutoffs and output directory");return 2;
      }
      try {v2_owner.reset(new rgbd_v2::Recorder(v2_index,v2_detail_windows,v2_collect_core_l0));}
      catch(const std::exception& e){ROS_ERROR("v2 recorder: %s",e.what());return 2;}
      v2_recorder=v2_owner.get();
      timing_enabled=false;
      ORB_SLAM3::SetRgbdObservationCapacity(v2_recorder->ExpectedInputs());
      ORB_SLAM3::SetRgbdLightDiagnostics(v2_collect_core_l0);
    }

    int rgb_queue = 1, depth_queue = 1, sync_queue = 10;
    node_handler.param<int>(node_name + "/rgb_queue", rgb_queue, 1);
    node_handler.param<int>(node_name + "/depth_queue", depth_queue, 1);
    node_handler.param<int>(node_name + "/sync_queue", sync_queue, 10);
    if (rgb_queue < 1 || rgb_queue > 64 || depth_queue < 1 || depth_queue > 64 ||
        sync_queue < 1 || sync_queue > 64)
    { ROS_ERROR("Queue capacities must be in [1,64]"); return 2; }
    if(v2_enabled && (rgb_queue!=8 || depth_queue!=8 || sync_queue!=10)){
      ROS_ERROR("v2 frozen queue configuration requires RGB/depth/sync=8/8/10");return 2;
    }
    ROS_INFO("Effective queues: rgb=%d depth=%d exact_sync=%d; single callback thread", rgb_queue, depth_queue, sync_queue);
    bool benchmark_enabled=false;
    node_handler.param<bool>(node_name+"/field_benchmark_metrics",benchmark_enabled,false);
    if(benchmark_enabled)field_benchmark=new(std::nothrow) field::Benchmark;
    bool field_enabled=false;
    std::string field_directory;
    node_handler.param<bool>(node_name+"/field_diagnostics_enabled",field_enabled,false);
    node_handler.param<std::string>(node_name+"/field_diagnostics_dir",field_directory,"/tmp/rgbd_field_diagnostics");
    if((v2_enabled || scale_enabled) && (field_enabled || benchmark_enabled)){
      ROS_ERROR("v2 L0 cannot share an unregistered legacy field/benchmark recorder");return 2;
    }
    if(field_enabled)timing_enabled=false;
    ORB_SLAM3::SetHeavyDiagnostics(timing_enabled);
    if(field_enabled){
      field::Config cfg;
      node_handler.param<double>(node_name+"/field_before_seconds",cfg.before,30.0);
      node_handler.param<double>(node_name+"/field_after_seconds",cfg.after,10.0);
      field_recorder=new(std::nothrow) field::Recorder(field_directory.c_str(),cfg);
      if(field_recorder&&!field_recorder->Start()){delete field_recorder;field_recorder=nullptr;}
      if(field_recorder)ORB_SLAM3::SetFieldSink(field::Recorder::Sink,field_recorder);
    }
    ROS_INFO("Field diagnostics=%s object=%zu stacks=262144 directory=%s heavy=%s",field_recorder?"enabled":"off",field_recorder?sizeof(field::Recorder):0,field_directory.c_str(),timing_enabled?"enabled":"off");
    std::string camera_shm;
    node_handler.param<std::string>(node_name+"/field_camera_shm",camera_shm,"");
    if(field_recorder&&!camera_shm.empty()){
      camera_bridge=new(std::nothrow) camera_delivery::Bridge(camera_shm.c_str(),field_recorder);
      if(camera_bridge&&!camera_bridge->Start()){delete camera_bridge;camera_bridge=nullptr;}
    }
    InputDiagnostics diagnostics(timing_output_dir,timing_enabled);
    sensor_type = ORB_SLAM3::System::RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type,
                                  enable_pangolin);
    if(v2_enabled && v2_recorder->RetainedBytes()+pSLAM->LocalMappingTimingCapacityBytes()>33554432U){
      ROS_ERROR("v2 recorder plus LocalMapping record buffers exceed the 32 MiB workspace ceiling");
      pSLAM->ShutdownAndJoin();delete pSLAM;pSLAM=nullptr;return 2;
    }

    ImageGrabber grabber(pSLAM, timing_enabled, timing_output_dir);
    ros::CallbackQueue v2_image_queue;
    ros::CallbackQueueInterface* image_queue=v2_enabled?&v2_image_queue:nullptr;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler,
        rgb_topic,rgb_queue,ros::TransportHints(),image_queue);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler,
        depth_topic,depth_queue,ros::TransportHints(),image_queue);
    if (!scale_enabled) {
    rgb_sub.registerCallback(boost::bind(&InputDiagnostics::Rgb, &diagnostics, _1));
    depth_sub.registerCallback(boost::bind(&InputDiagnostics::Depth, &diagnostics, _1));
    }
    if (!scale_enabled) {
    boost::function<void(const ros::MessageEvent<sensor_msgs::Image const>&)> field_rgb_callback=boost::bind(&FieldInput,true,_1);
    boost::function<void(const ros::MessageEvent<sensor_msgs::Image const>&)> field_depth_callback=boost::bind(&FieldInput,false,_1);
    rgb_sub.registerCallback(field_rgb_callback);
    depth_sub.registerCallback(field_depth_callback);
    }
    if (scale_enabled) {
        rgb_sub.registerCallback(boost::bind(&rgbd_scale::Recorder::Rgb,scale_recorder,_1));
        depth_sub.registerCallback(boost::bind(&rgbd_scale::Recorder::Depth,scale_recorder,_1));
    }
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>
        sync(rgb_sub, depth_sub, sync_queue);
    if (!scale_enabled) sync.registerDropCallback(boost::bind(&FieldDrop,_1,_2));
    if (!scale_enabled) sync.registerDropCallback(boost::bind(&InputDiagnostics::Drop, &diagnostics, _1, _2));
    if (scale_enabled) sync.registerDropCallback(boost::bind(&rgbd_scale::Recorder::Drop,scale_recorder,_1,_2));
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &grabber, _1, _2));

    setup_publishers(node_handler, node_name);
    setup_services(node_handler, node_name);

    ROS_INFO("ORB-SLAM3 RGB-D input: rgb=%s depth=%s (exact timestamp sync)",
             rgb_topic.c_str(), depth_topic.c_str());
    if (timing_enabled)
        ROS_INFO("RGB-D timing CSV output: %s", timing_output_dir.c_str());
    bool v2_front_idle=false, v2_front_timeout=false, v2_forced_close=false;
    bool v2_joined=false, v2_write_ok=true, v2_trajectory_saved=false;
    double v2_front_drain_ms=0.0, v2_shutdown_ms=0.0;
    unsigned v2_rgb_publishers=0, v2_depth_publishers=0;
    ORB_SLAM3::System::BackendDrainResult v2_backend;
    ros::ServiceServer v2_seal;
    int scale_exit=0;
    if (scale_enabled) scale_exit=rgbd_scale::SpinAndWrite(node_handler,node_name,rgb_sub,depth_sub,
        *scale_recorder,scale_directory,scale_front_timeout_s);
    else if(!v2_enabled) ros::spin();
    else {
      v2_seal=node_handler.advertiseService(v2_seal_service,V2Seal);
      ROS_INFO("v2 seal service=%s; front cutoff=%ds backend cutoff=%ds recorder=%zu bytes",
               v2_seal_service.c_str(),v2_front_timeout_s,v2_backend_timeout_s,v2_recorder->RetainedBytes());
      while(ros::ok()){
        // Control services and images execute on this one thread. A service
        // request never recursively spins/drains while an image is active.
        ros::spinOnce();
        const auto dispatched=v2_image_queue.callOne(ros::WallDuration(0.001));
        if(!v2_seal_request_ns)continue;
        v2_front_drain_ms=(rgbd_v2::Now()-v2_seal_request_ns)*1e-6;
        v2_rgb_publishers=rgb_sub.getSubscriber().getNumPublishers();
        v2_depth_publishers=depth_sub.getSubscriber().getNumPublishers();
        if(v2_front_drain_ms>=v2_front_timeout_s*1000.0){v2_front_timeout=true;break;}
        if(v2_rgb_publishers==0 && v2_depth_publishers==0 &&
           dispatched==ros::CallbackQueue::Empty && v2_image_queue.isEmpty()){
          v2_front_idle=true;break;
        }
      }
      // Only the publisher-disconnected + actually empty path is a natural
      // front drain. Forced unsubscribe may discard unobserved subscription
      // messages; retain false/null evidence instead of calling them received.
      v2_forced_close=!v2_front_idle;
      rgb_sub.unsubscribe();depth_sub.unsubscribe();
      v2_recorder->SettleUnpaired();
      if(v2_front_idle){
        const auto observation_begin=rgbd_v2::Now();
        const auto before=pSLAM->GetLoopLifecycleCounts();
        // Enabled only by the separately declared activity protocol round.
        // This terminal sample does not alter or cancel background work.
        pSLAM->RecordMemoryStats();
        const auto observation_end=rgbd_v2::Now();
        std::ofstream initial(v2_directory+"/front_idle_background.json");
        initial<<"{\"observation_begin_ns\":"<<observation_begin
          <<",\"observation_end_ns\":"<<observation_end
          <<",\"loop_detected\":"<<before.loop_detected
          <<",\"gba_started\":"<<before.gba_started
          <<",\"gba_completed\":"<<before.gba_completed
          <<",\"gba_returned\":"<<before.gba_returned
          <<",\"gba_joined\":"<<before.gba_joined_count
          <<",\"scope\":\"counter snapshot before DrainBackend; atomic counters are not a simultaneous full backend snapshot\"}\n";
        initial.close();v2_write_ok=bool(initial)&&v2_write_ok;
        v2_backend=pSLAM->DrainBackend(unsigned(v2_backend_timeout_s)*1000U);
      }
      const auto shutdown_start=rgbd_v2::Now();
      v2_joined=pSLAM->ShutdownAndJoin();
      v2_shutdown_ms=(rgbd_v2::Now()-shutdown_start)*1e-6;
      // The inherited exporter dereferences the first keyframe; empty input
      // must remain a measured input failure, not crash during terminal output.
      if(v2_joined && pSLAM->GetCurrentMapKeyFrames()>0){
        const auto trajectory=v2_directory+"/trajectory_corrected.tum";
        pSLAM->SaveTrajectoryTUM(trajectory);
        std::ifstream exported(trajectory);v2_trajectory_saved=bool(exported);
      }
      v2_write_ok=v2_recorder->Write(v2_directory);
      std::ofstream local_timing(v2_directory+"/core_local_mapping_timing.csv");
      rgbd_v2::WriteLocalMappingHeader(local_timing);
      if(v2_joined)
        for(const auto& row:pSLAM->CompletedLocalMappingTimings())
          rgbd_v2::WriteLocalMappingRow(local_timing,row);
      local_timing.close();
      v2_write_ok=bool(local_timing)&&v2_write_ok;
    }

    const auto scale_shutdown_start=scale_enabled?rgbd_scale::Now():0;
    if(!v2_enabled)pSLAM->Shutdown();
    if (scale_enabled && !rgbd_scale::WriteShutdown(scale_directory,scale_shutdown_start)) scale_exit=4;
    if(field_benchmark){field_benchmark->map=pSLAM->GetAtlasMapCount();field_benchmark->kfs=pSLAM->GetCurrentMapKeyFrames();field_benchmark->mps=pSLAM->GetCurrentMapPoints();field_benchmark->Save("benchmark.txt");delete field_benchmark;field_benchmark=nullptr;}
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
    bool camera_bridge_stopped=true;
    if(camera_bridge){camera_bridge_stopped=camera_bridge->Stop();if(camera_bridge_stopped)delete camera_bridge;camera_bridge=nullptr;}
    if(field_recorder&&camera_bridge_stopped){
      ORB_SLAM3::SetFieldSink(nullptr,nullptr);
      if(field_recorder->Stop())delete field_recorder;
      else ROS_WARN("Diagnostic writer shutdown timed out; in-memory records may be lost");
      field_recorder=nullptr;
    }
    int v2_exit=0;
    if(v2_enabled){
      const bool natural=v2_backend.status==ORB_SLAM3::System::BackendDrainResult::COMPLETE;
      const bool timeout=v2_front_timeout ||
          (v2_front_idle && v2_backend.status==ORB_SLAM3::System::BackendDrainResult::TIMEOUT);
      const auto counts=pSLAM->GetLoopLifecycleCounts();
      const auto resets=ORB_SLAM3::GetRgbdResetCounters();
      const auto local_dropped=pSLAM->DroppedLocalMappingTimingRecords();
      const bool l0_enabled=ORB_SLAM3::RgbdLightDiagnostics();
      const auto core_dropped=v2_recorder->CoreL0RecordsDropped();
      const auto core_missing=v2_recorder->CoreL0RecordsMissing();
      const bool measurement_complete=l0_enabled && v2_joined && v2_write_ok &&
          v2_recorder->Valid() && !local_dropped && !core_dropped && !core_missing;
      v2_recorder->ObserveMaps(pSLAM->GetAtlasMapCount());
      v2_exit=(!v2_front_idle || !natural || !v2_joined || !v2_write_ok || !v2_recorder->Valid() ||
               (v2_collect_core_l0 && !measurement_complete))?3:0;
      std::ofstream completion(v2_directory+"/completion.json");
      completion<<std::boolalpha<<std::setprecision(12)
        <<"{\"front_idle\":"<<v2_front_idle<<",\"input_complete\":"<<v2_recorder->InputComplete()
        <<",\"background_natural\":"<<natural<<",\"local_mapping_joined\":"<<v2_joined
        <<",\"loop_closing_joined\":"<<v2_joined<<",\"gba_joined\":"<<v2_joined
        <<",\"orb_helpers_joined\":"<<v2_joined<<",\"timed_out\":"<<timeout
        <<",\"forced_termination\":false,\"forced_input_close\":"<<v2_forced_close
        <<",\"exit_code\":"<<v2_exit<<",\"unexpected_resets\":"<<(resets.reset+resets.active_reset)
        <<",\"max_maps\":"<<v2_recorder->MaxMaps()
        <<",\"loop_detected\":"<<counts.loop_detected<<",\"gba_started\":"<<counts.gba_started
        <<",\"gba_completed\":"<<counts.gba_completed<<",\"gba_returned\":"<<counts.gba_returned
        <<",\"gba_joined_count\":"<<counts.gba_joined_count<<",\"gba_errors\":"<<counts.gba_errors
        <<",\"gba_cancelled\":"<<counts.gba_cancelled
        <<",\"gba_drain_cancel_requests\":"<<(natural?"0":"null")
        <<",\"gba_drain_cancel_evidence\":\"restricted M0 drain only calls JoinIfFinished; no cancel operation\""
        <<",\"trajectory_exported\":"<<v2_trajectory_saved
        <<",\"measurement_schema\":\"rgbd_v2_l0_v1\""
        <<",\"calibration_only\":"<<!v2_collect_core_l0
        <<",\"core_l0_enabled\":"<<l0_enabled<<",\"local_mapping_l0_enabled\":"<<l0_enabled
        <<",\"core_l0_records_dropped\":"<<core_dropped<<",\"core_l0_records_missing\":"<<core_missing
        <<",\"local_mapping_l0_records_dropped\":"<<local_dropped
        <<",\"local_mapping_timing_capacity_records\":"<<pSLAM->LocalMappingTimingCapacityRecords()
        <<",\"local_mapping_timing_capacity_bytes\":"<<pSLAM->LocalMappingTimingCapacityBytes()
        <<",\"recorder_capacity_bytes\":"<<v2_recorder->RetainedBytes()
        <<",\"measurement_buffers_total_bytes\":"<<(v2_recorder->RetainedBytes()+pSLAM->LocalMappingTimingCapacityBytes())
        <<",\"workspace_ceiling_bytes\":33554432"
        <<",\"measurement_complete\":"<<measurement_complete
        <<",\"local_mapping_records_dropped\":"<<local_dropped
        <<",\"seal_requested\":"<<(v2_seal_request_ns!=0)<<",\"front_drain_ms\":"<<v2_front_drain_ms
        <<",\"backend_drain_ms\":"<<v2_backend.wait_ms<<",\"shutdown_join_ms\":"<<v2_shutdown_ms
        <<",\"backend_status\":"<<int(v2_backend.status)
        <<",\"seal_rgb_publishers\":"<<v2_rgb_publishers<<",\"seal_depth_publishers\":"<<v2_depth_publishers
        <<",\"receipt_scope\":\"MessageEvent ROS receipt observed at filter_callback_enter after successful deserialization; queue residence unobserved\""
        <<",\"network_input_complete_measured\":false,\"pose_consumer_complete\":null}\n";
      completion.close();
      if(!completion)v2_exit=4;
      v2_recorder=nullptr;
    }
    delete pSLAM;
    pSLAM = nullptr;
    return scale_enabled?scale_exit:v2_exit;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msg_rgb,
                            const sensor_msgs::ImageConstPtr &msg_depth)
{
    if (scale_recorder) {
        scale_recorder->Grab(msg_rgb,msg_depth,slam_,
            [](const Sophus::SE3f& pose,ros::Time stamp,bool valid,int state) {
                if (state==2 || state==5) rgbd_map_initialized=true;
                publish_pose_and_odometry(pose,stamp,valid);
            });
        return;
    }

    if(!v2_recorder){GrabRGBDBody(msg_rgb,msg_depth,nullptr);return;}
    const auto outer_start=rgbd_v2::Now();
    rgbd_v2::Frame frame;
    frame.enter_ns=outer_start;
    frame.source=v2_recorder->Find(msg_rgb->header.stamp.toNSec(),msg_rgb->header.seq,true);
    const auto depth_source=v2_recorder->Find(msg_depth->header.stamp.toNSec(),msg_depth->header.seq,false);
    if(depth_source.ordinal!=frame.source.ordinal)v2_recorder->IdentityError();
    v2_recorder->Record(rgbd_v2::EventKind::Paired,frame.source,msg_rgb->header.seq);
    try {GrabRGBDBody(msg_rgb,msg_depth,&frame);}
    catch(...){frame.callback_exception=true;v2_recorder->Complete(frame);throw;}
    // All body locals (including cv_bridge owners) and legacy timing push are
    // finished before this outer boundary includes the new recorder append.
    v2_recorder->Complete(frame);
}

void ImageGrabber::GrabRGBDBody(const sensor_msgs::ImageConstPtr &msg_rgb,
                              const sensor_msgs::ImageConstPtr &msg_depth,
                              rgbd_v2::Frame* v2_frame)
{
    const std::chrono::steady_clock::time_point callback_start =
        std::chrono::steady_clock::now();
    const uint64_t field_callback_start=(field_recorder||field_benchmark)?field::Now():0;
    FrameTimingSample sample;
    if (timing_enabled_)
    {
        sample.rgb_seq = msg_rgb->header.seq;
        sample.depth_seq = msg_depth->header.seq;
        sample.rgb_depth_stamp_delta_ms =
            (msg_depth->header.stamp - msg_rgb->header.stamp).toSec() * 1000.0;
        sample.header_age_ms =
            (ros::Time::now() - msg_rgb->header.stamp).toSec() * 1000.0;
        if (!have_previous_callback_)
        {
            first_callback_time_ = callback_start;
            previous_callback_time_ = callback_start;
            previous_header_stamp_ = msg_rgb->header.stamp;
            previous_rgb_seq_ = msg_rgb->header.seq;
            have_previous_callback_ = true;
        }
        else
        {
            sample.callback_interval_ms =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    callback_start - previous_callback_time_)
                    .count();
            sample.header_period_ms =
                (msg_rgb->header.stamp - previous_header_stamp_).toSec() *
                1000.0;
            sample.rgb_seq_gap = static_cast<int64_t>(msg_rgb->header.seq) -
                                 static_cast<int64_t>(previous_rgb_seq_) - 1;
            previous_callback_time_ = callback_start;
            previous_header_stamp_ = msg_rgb->header.stamp;
            previous_rgb_seq_ = msg_rgb->header.seq;
        }
        sample.callback_start_ms =
            std::chrono::duration_cast<
                std::chrono::duration<double, std::milli>>(
                callback_start - first_callback_time_)
                .count();
    }

    if (msg_rgb->width != msg_depth->width ||
        msg_rgb->height != msg_depth->height)
    {
        ORB_SLAM3::FieldEmit(7,field_ordinal,0,1,1);
        ROS_ERROR_THROTTLE(2.0,
                           "RGB/depth size mismatch: rgb=%ux%u depth=%ux%u",
                           msg_rgb->width, msg_rgb->height,
                           msg_depth->width, msg_depth->height);
        return;
    }

    if (msg_depth->encoding != sensor_msgs::image_encodings::TYPE_32FC1 &&
        msg_depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
    {
        ORB_SLAM3::FieldEmit(7,field_ordinal,0,1,2);
        ROS_ERROR_THROTTLE(2.0, "Unsupported depth encoding: %s",
                           msg_depth->encoding.c_str());
        return;
    }

    cv_bridge::CvImageConstPtr rgb;
    cv_bridge::CvImageConstPtr depth;
    const std::chrono::steady_clock::time_point bridge_start =
        std::chrono::steady_clock::now();
    try
    {
        rgb = cv_bridge::toCvShare(msg_rgb);
        depth = cv_bridge::toCvShare(msg_depth);
    }
    catch (const cv_bridge::Exception &error)
    {
        ORB_SLAM3::FieldEmit(7,field_ordinal,0,1,3);
        ROS_ERROR("cv_bridge exception: %s", error.what());
        return;
    }
    if (timing_enabled_)
        sample.cv_bridge_ms = MillisecondsSince(bridge_start);

    const ros::Time stamp = msg_rgb->header.stamp;
    const std::chrono::steady_clock::time_point track_start =
        std::chrono::steady_clock::now();
    if(field_benchmark)field_benchmark->Input(2,msg_rgb->header.seq);
    uint64_t field_start=0;
    if(field_recorder){
      field_start=field::Now();const uint64_t source=stamp.toNSec();++field_ordinal;
      ORB_SLAM3::FieldEmit(3,field_ordinal,0,field_previous_stamp&&source>field_previous_stamp?31:15,source,msg_depth->header.stamp.toNSec(),msg_rgb->header.seq,msg_depth->header.seq,field_previous_stamp&&source>field_previous_stamp?source-field_previous_stamp:0);
      field_previous_stamp=source;
    }
    if(v2_frame){
      const auto ordinal=v2_frame->source.ordinal;
      const bool detail=v2_recorder->DetailWindow(ordinal);
      ORB_SLAM3::SetRgbdObservationContext(stamp.toNSec(),ordinal,detail);
      v2_recorder->Record(rgbd_v2::EventKind::ProcessedEnter,v2_frame->source,msg_rgb->header.seq);
      v2_frame->track_called=true;
    }
    struct ContextClear {
      bool enabled;
      ~ContextClear(){if(enabled)ORB_SLAM3::ClearRgbdObservationContext();}
    } context_clear{v2_frame!=nullptr};
    const auto v2_track_start=v2_frame?rgbd_v2::Now():0;
    const Sophus::SE3f Tcw =
        slam_->TrackRGBD(rgb->image, depth->image, stamp.toSec());
    if(v2_frame){
      v2_frame->track_call_ms=(rgbd_v2::Now()-v2_track_start)*1e-6;
      v2_frame->track_returned=true;
      v2_recorder->Record(rgbd_v2::EventKind::ProcessedReturn,v2_frame->source,msg_rgb->header.seq);
      if(v2_recorder->CoreL0Enabled()){
        const auto timing=slam_->GetLastRgbdTiming();
        v2_frame->light.Capture(timing);
        v2_recorder->RecordDetail(timing);
      }
      v2_recorder->ObserveMaps(slam_->GetAtlasMapCount());
    }
    if (timing_enabled_)
    {
        sample.track_call_ms = MillisecondsSince(track_start);
        sample.core = slam_->GetLastRgbdTiming();
    }
    const std::chrono::steady_clock::time_point state_start =
        std::chrono::steady_clock::now();
    const int tracking_state = slam_->GetTrackingState();
    if (ORB_SLAM3::MemoryStats::Enabled()) {
      static auto next_memory_sample = std::chrono::steady_clock::now();
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_memory_sample) { slam_->RecordMemoryStats(); next_memory_sample = now + std::chrono::seconds(1); }
    }
    if (tracking_state == 2 || tracking_state == 5) rgbd_map_initialized = true;
    if (timing_enabled_)
        sample.get_state_ms = MillisecondsSince(state_start);
    const uint64_t field_end=field_recorder?field::Now():0;
    const bool tracking_ok = tracking_state == ORB_SLAM3::Tracking::OK ||
                             tracking_state == ORB_SLAM3::Tracking::OK_KLT;
    const bool pose_ok = tracking_ok && Tcw.translation().allFinite() &&
                         Tcw.rotationMatrix().allFinite();
    const std::chrono::steady_clock::time_point publish_start =
        std::chrono::steady_clock::now();
    PosePublishOutcome publication;
    publish_pose_and_odometry(Tcw, stamp, pose_ok,v2_frame?&publication:nullptr);
    if(v2_frame){
      v2_frame->state_after=tracking_state;
      v2_frame->pose_valid=pose_ok && publication.raw_slam_valid;
      v2_frame->pose_fresh=publication.raw_slam_fresh;
      v2_frame->pose_emitted=publication.raw_slam_emitted;
      v2_frame->state_publications=publication.state_publications;
      for(unsigned i=0;i<publication.state_publications && i<2;++i)
        v2_recorder->Record(rgbd_v2::EventKind::StateOutput,v2_frame->source,msg_rgb->header.seq,
                            0,publication.state_steady_ns[i]);
      if(publication.raw_slam_emitted)
        v2_recorder->Record(rgbd_v2::EventKind::PosePublish,v2_frame->source,msg_rgb->header.seq,
                            0,publication.raw_slam_steady_ns);
    }
    if(field_benchmark)field_benchmark->End(field::Now()-field_callback_start,tracking_state,stamp.toNSec());
    if(field_recorder){
      ORB_SLAM3::FieldEmit(4,field_ordinal,0,15,field_end-field_start,tracking_state,pose_ok,field::Now()-field_callback_start);
      static uint64_t next_map=0;
      if(field_end>=next_map){ORB_SLAM3::FieldEmit(22,field_ordinal,0,15,slam_->GetCurrentMapId(),slam_->GetAtlasMapCount(),slam_->GetCurrentMapKeyFrames(),slam_->GetCurrentMapPoints());next_map=field_end+1000000000;}
    }
    if (timing_enabled_)
    {
        sample.publish_ms = MillisecondsSince(publish_start);
        sample.callback_total_ms = MillisecondsSince(callback_start);
        if (timing_samples_.size() < 20000)
            timing_samples_.push_back(sample);
        else
            ROS_ERROR_THROTTLE(10.0, "Timing record limit 20000 reached; later timing rows omitted");
    }
}

bool ImageGrabber::WriteTimingCsv() const
{
    const std::string filename = timing_output_dir_ + "/rgbd_frame_timing.csv";
    std::ofstream output(filename.c_str());
    if (!output.is_open())
        return false;

    output << "rgb_seq,depth_seq,rgb_seq_gap,image_timestamp,frame_id,"
              "state_before,state_after,keypoints,valid_depths,tracked_inliers,"
              "lm_queue_before,lm_queue_after,inserted_keyframe,"
              "callback_start_ms,callback_interval_ms,header_period_ms,"
              "rgb_depth_stamp_delta_ms,header_age_ms,cv_bridge_ms,"
              "track_call_ms,get_state_ms,publish_ms,callback_total_ms,"
              "system_clone_ms,system_resize_ms,mode_mutex_wait_ms,"
              "mode_change_ms,reset_mutex_wait_ms,reset_ms,grab_rgbd_ms,"
              "state_copy_ms,system_total_ms,gray_convert_ms,depth_convert_ms,"
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
              "need_new_kf_ms,create_new_kf_ms,local_update_ms,local_keyframes_ms,"
              "local_points_ms,local_search_ms,local_pose_ms,local_keyframes,local_points\n";
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
               << sample.header_age_ms << ',' << sample.cv_bridge_ms << ','
               << sample.track_call_ms << ',' << sample.get_state_ms << ','
               << sample.publish_ms << ',' << sample.callback_total_ms << ','
               << r.system_clone_ms << ',' << r.system_resize_ms << ','
               << r.mode_mutex_wait_ms << ',' << r.mode_change_ms << ','
               << r.reset_mutex_wait_ms << ',' << r.reset_ms << ','
               << r.grab_rgbd_ms << ',' << r.state_copy_ms << ','
               << r.system_total_ms << ',' << r.gray_convert_ms << ','
               << r.depth_convert_ms << ',' << r.frame_construct_ms << ','
               << r.orb_extract_ms << ',' << r.orb_pyramid_ms << ','
               << r.orb_keypoints_octree_ms << ',' << r.orb_orientation_ms
               << ',' << r.orb_blur_ms << ',' << r.orb_descriptor_ms << ','
               << r.orb_result_assembly_ms << ',' << r.orb_fast_initial_ms
               << ',' << r.orb_fast_fallback_ms << ',' << r.orb_octree_ms
               << ',' << r.orb_copy_ms << ',' << r.orb_gaussian_ms << ','
               << r.orb_output_hash << ',' << r.orb_mono_index << ','
               << r.orb_level_count << ',';
        WriteLevelArray(output, r.orb_level_fast_initial_ms,
                        r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_fast_fallback_ms,
                        r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_octree_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_orientation_ms,
                        r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_copy_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_gaussian_ms, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_descriptor_ms,
                        r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_candidates, r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_fallback_cells,
                        r.orb_level_count);
        output << ',';
        WriteLevelArray(output, r.orb_level_octree_splits,
                        r.orb_level_count);
        output << ',' << r.rgbd_depth_assoc_ms << ','
               << r.track_internal_ms << ',' << r.map_mutex_wait_ms << ','
               << r.initialization_ms << ',' << r.reference_kf_ms << ','
               << r.motion_model_ms << ',' << r.relocalization_ms << ','
               << r.local_map_ms << ',' << r.need_new_kf_ms << ','
               << r.create_new_kf_ms << ',' << r.local_update_ms << ','
               << r.local_keyframes_ms << ',' << r.local_points_ms << ','
               << r.local_search_ms << ',' << r.local_pose_ms << ','
               << r.local_keyframes << ',' << r.local_points << '\n';
    }
    return output.good();
}
