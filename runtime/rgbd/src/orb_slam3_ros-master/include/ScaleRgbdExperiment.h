#pragma once

// Matched, buffered observations only. This header is identical in both builds.
#include "MapScaleObservation.h"
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <time.h>
#include <vector>

namespace rgbd_scale {
inline std::uint64_t Now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::int64_t ThreadCpu() {
    timespec t;
    if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t)) return -1;
    return std::int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
}
struct Source {
    std::uint64_t ordinal=0, stamp=0;
    std::uint32_t rgb_seq=0, depth_seq=0;
    std::array<std::uint32_t, 7> counts{};
};
enum Kind { ReceivedRgb, ReceivedDepth, SyncDropRgb, SyncDropDepth, Paired,
            Returned, ValidFresh };
inline const char* KindName(Kind k) {
    static const char* names[]={"received_rgb","received_depth","sync_drop_rgb",
        "sync_drop_depth","paired","returned","valid_fresh"};
    return names[unsigned(k)];
}
struct Event {
    Kind kind=ReceivedRgb;
    std::uint64_t source=0, stamp=0, steady=0;
    std::uint32_t seq=0;
};
struct Frame {
    std::uint64_t source=0, stamp=0, enter=0, complete=0;
    std::uint32_t rgb_seq=0, depth_seq=0;
    std::int64_t cpu_start=-1;
    double callback_ms=0, cpu_ms=-1, bridge_ms=0, track_ms=0, state_ms=0, publish_ms=0;
    int state=-1, error=0;
    bool track_called=false, track_returned=false, publish_called=false;
    bool pose_valid=false, pose_fresh=false;
    std::array<double,7> pose{{NAN,NAN,NAN,NAN,NAN,NAN,NAN}};
    ORB_SLAM3::MapScaleObservationRecord core;
};
class Recorder {
public:
    Recorder(const std::string& index, bool detail): detail_(detail) {
        std::ifstream input(index);
        std::string line;
        if (!std::getline(input,line)) throw std::runtime_error("empty scale input index");
        if (!line.empty() && line.back()=='\r') line.pop_back();
        if (line!="source_ordinal,source_stamp_ns,rgb_seq,depth_seq")
            throw std::runtime_error("unexpected scale index columns");
        while (std::getline(input,line)) {
            if (!line.empty() && line.back()=='\r') line.pop_back();
            std::replace(line.begin(),line.end(),',',' ');
            std::istringstream row(line);
            Source s;
            std::string extra;
            if (!(row>>s.ordinal>>s.stamp>>s.rgb_seq>>s.depth_seq) || (row>>extra) ||
                s.ordinal!=sources_.size()+1 || !s.stamp ||
                (!sources_.empty() && s.stamp<=sources_.back().stamp))
                throw std::runtime_error("invalid scale source identity/order");
            sources_.push_back(s);
        }
        if (sources_.empty()) throw std::runtime_error("scale requires nonempty index");
        frames_.reserve(sources_.size()*2+64);
        events_.reserve(sources_.size()*8+64);
    }
    bool Detail() const { return detail_; }
    std::size_t Expected() const { return sources_.size(); }
    std::size_t RetainedBytes() const {
        return sources_.capacity()*sizeof(Source)+frames_.capacity()*sizeof(Frame)+
               events_.capacity()*sizeof(Event);
    }
    std::uint64_t Find(std::uint64_t stamp, std::uint32_t seq, bool rgb) {
        const auto it=std::lower_bound(sources_.begin(),sources_.end(),stamp,
            [](const Source& s,std::uint64_t value){return s.stamp<value;});
        if (it==sources_.end() || it->stamp!=stamp ||
            (rgb?it->rgb_seq:it->depth_seq)!=seq) { ++identity_errors_; return 0; }
        return it->ordinal;
    }
    void Record(Kind kind, std::uint64_t source, std::uint32_t seq,
                std::uint64_t stamp) {
        if (source) ++sources_[source-1].counts[unsigned(kind)];
        if (events_.size()==events_.capacity()) { ++event_overflow_; return; }
        events_.push_back(Event{kind,source,stamp,Now(),seq});
    }
    void Rgb(const sensor_msgs::ImageConstPtr& m) {
        Record(ReceivedRgb,Find(m->header.stamp.toNSec(),m->header.seq,true),
               m->header.seq,m->header.stamp.toNSec());
    }
    void Depth(const sensor_msgs::ImageConstPtr& m) {
        Record(ReceivedDepth,Find(m->header.stamp.toNSec(),m->header.seq,false),
               m->header.seq,m->header.stamp.toNSec());
    }
    void Drop(const sensor_msgs::ImageConstPtr& rgb,const sensor_msgs::ImageConstPtr& depth) {
        if (rgb) Record(SyncDropRgb,Find(rgb->header.stamp.toNSec(),rgb->header.seq,true),
                         rgb->header.seq,rgb->header.stamp.toNSec());
        if (depth) Record(SyncDropDepth,Find(depth->header.stamp.toNSec(),depth->header.seq,false),
                           depth->header.seq,depth->header.stamp.toNSec());
    }
    bool Seal(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
        if (!seal_ns_) seal_ns_=Now();
        response.success=true;
        response.message="source end registered; global single-thread queue drain continues";
        return true;
    }
    std::uint64_t SealNs() const { return seal_ns_; }
    template<class Publish>
    void Grab(const sensor_msgs::ImageConstPtr& rgb,const sensor_msgs::ImageConstPtr& depth,
              ORB_SLAM3::System* slam, Publish publish) {
        const auto enter=Now();
        const auto cpu=ThreadCpu();
        Frame frame;
        frame.enter=enter;frame.cpu_start=cpu;
        frame.stamp=rgb->header.stamp.toNSec();
        frame.rgb_seq=rgb->header.seq;frame.depth_seq=depth->header.seq;
        frame.source=Find(frame.stamp,frame.rgb_seq,true);
        if (Find(depth->header.stamp.toNSec(),frame.depth_seq,false)!=frame.source) {
            ++identity_errors_;frame.error=5;
        }
        Record(Paired,frame.source,frame.rgb_seq,frame.stamp);
        ORB_SLAM3::ResetMapScaleObservation(detail_);
        try { Body(rgb,depth,slam,publish,frame); }
        catch (...) { frame.error=4; ros::shutdown(); }
        if (detail_) frame.core=ORB_SLAM3::GetMapScaleObservation();
        ORB_SLAM3::DisableMapScaleObservation();
        // Body locals, notably both cv_bridge owners, are destroyed before
        // this append and the complete timestamp. No callback file IO.
        if (frame.track_returned) Record(Returned,frame.source,frame.rgb_seq,frame.stamp);
        if (frame.pose_fresh) Record(ValidFresh,frame.source,frame.rgb_seq,frame.stamp);
        if (frame.error) ++callback_errors_;
        if (frames_.size()==frames_.capacity()) { ++frame_overflow_;return; }
        frames_.push_back(frame);
        Frame& saved=frames_.back();
        const auto cpu_end=ThreadCpu();
        if (saved.cpu_start>=0 && cpu_end>=saved.cpu_start)
            saved.cpu_ms=double(cpu_end-saved.cpu_start)*1e-6;
        else ++cpu_clock_errors_;
        saved.complete=Now();
        saved.callback_ms=double(saved.complete-saved.enter)*1e-6;
    }
    bool Write(const std::string& directory,bool front_idle,bool front_timeout,
               unsigned rgb_publishers,unsigned depth_publishers) const {
        std::ofstream frames(directory+"/scale_frames.csv");
        frames<<"source_ordinal,source_stamp_ns,rgb_seq,depth_seq,enter_ns,complete_ns,"
            "callback_complete_ms,callback_cpu_ms,cv_bridge_ms,track_call_ms,get_state_ms,"
            "publish_ms,state_after,track_called,track_returned,publish_called,pose_valid,"
            "pose_fresh,callback_error,tx,ty,tz,qx,qy,qz,qw";
        ORB_SLAM3::WriteMapScaleHeader(frames);
        frames<<'\n'<<std::setprecision(17);
        for (const auto& f:frames_) {
            frames<<f.source<<','<<f.stamp<<','<<f.rgb_seq<<','<<f.depth_seq<<','
                  <<f.enter<<','<<f.complete<<','<<f.callback_ms<<','<<f.cpu_ms<<','
                  <<f.bridge_ms<<','<<f.track_ms<<','<<f.state_ms<<','<<f.publish_ms<<','
                  <<f.state<<','<<f.track_called<<','<<f.track_returned<<','
                  <<f.publish_called<<','<<f.pose_valid<<','<<f.pose_fresh<<','<<f.error;
            for (const auto v:f.pose) frames<<','<<v;
            ORB_SLAM3::WriteMapScaleRow(frames,f.core);frames<<'\n';
        }
        frames.close();
        std::ofstream sources(directory+"/scale_sources.csv");
        sources<<"source_ordinal,source_stamp_ns,rgb_seq,depth_seq,received_rgb,received_depth,"
            "sync_drop_rgb,sync_drop_depth,paired,returned,valid_fresh\n";
        std::size_t processed=0,valid=0;
        for (const auto& s:sources_) {
            sources<<s.ordinal<<','<<s.stamp<<','<<s.rgb_seq<<','<<s.depth_seq;
            for (auto c:s.counts) sources<<','<<c;
            sources<<'\n';
            if (s.counts[Returned]) ++processed;
            if (s.counts[ValidFresh]) ++valid;
        }
        sources.close();
        std::ofstream events(directory+"/scale_events.csv");
        events<<"kind,source_ordinal,seq,stamp_ns,steady_ns\n";
        for (const auto& e:events_) events<<KindName(e.kind)<<','<<e.source<<','<<e.seq<<','
            <<e.stamp<<','<<e.steady<<'\n';
        events.close();
        std::ofstream trajectory(directory+"/trajectory_online.tum");
        trajectory<<std::fixed<<std::setprecision(9);
        for (const auto& f:frames_) if(f.pose_fresh) {
            trajectory<<double(f.stamp)*1e-9;
            for(auto v:f.pose) trajectory<<' '<<v;
            trajectory<<'\n';
        }
        trajectory.close();
        const bool files_ok=bool(frames)&&bool(sources)&&bool(events)&&bool(trajectory);
        std::ofstream completion(directory+"/completion.json");
        completion<<std::boolalpha<<"{\"schema\":\"rgbd_scale_v1\",\"scale_detail\":"<<detail_
            <<",\"expected_inputs\":"<<sources_.size()<<",\"frame_records\":"<<frames_.size()
            <<",\"processed_sources\":"<<processed<<",\"valid_fresh_sources\":"<<valid
            <<",\"input_complete\":"<<(processed==sources_.size())
            <<",\"front_idle\":"<<front_idle<<",\"front_timeout\":"<<front_timeout
            <<",\"seal_requested\":"<<(seal_ns_!=0)<<",\"seal_ns\":"<<seal_ns_
            <<",\"front_end_ns\":"<<Now()<<",\"rgb_publishers\":"<<rgb_publishers
            <<",\"depth_publishers\":"<<depth_publishers<<",\"files_ok\":"<<files_ok
            <<",\"frame_overflow\":"<<frame_overflow_<<",\"event_overflow\":"<<event_overflow_
            <<",\"identity_errors\":"<<identity_errors_<<",\"callback_errors\":"<<callback_errors_
            <<",\"cpu_clock_errors\":"<<cpu_clock_errors_<<",\"recorder_bytes\":"<<RetainedBytes()
            <<",\"last_callback_complete_ns\":"<<(frames_.empty()?0:frames_.back().complete)
            <<",\"written_before_shutdown\":true,\"backend_joined\":null,"
              "\"pose_consumer_complete\":null,"
              "\"pose_scope\":\"fresh finite TrackRGBD Twc offered to existing publisher; consumer delivery is independently observed\","
              "\"input_scope\":\"filter callback after successful deserialization; not wire arrival or queue residence\"}\n";
        completion.close();
        return files_ok && bool(completion);
    }
    bool Valid() const { return !frame_overflow_&&!event_overflow_&&!identity_errors_&&!callback_errors_; }
private:
    template<class Publish>
    void Body(const sensor_msgs::ImageConstPtr& rgb_msg,const sensor_msgs::ImageConstPtr& depth_msg,
              ORB_SLAM3::System* slam,Publish publish,Frame& f) {
        if(rgb_msg->width!=depth_msg->width || rgb_msg->height!=depth_msg->height) { f.error=1;return; }
        if(depth_msg->encoding!=sensor_msgs::image_encodings::TYPE_32FC1 &&
           depth_msg->encoding!=sensor_msgs::image_encodings::TYPE_16UC1) { f.error=2;return; }
        const auto bridge_start=Now();
        cv_bridge::CvImageConstPtr rgb,depth;
        try { rgb=cv_bridge::toCvShare(rgb_msg);depth=cv_bridge::toCvShare(depth_msg); }
        catch(const cv_bridge::Exception&) { f.error=3;return; }
        f.bridge_ms=double(Now()-bridge_start)*1e-6;
        f.track_called=true;
        const auto track_start=Now();
        const Sophus::SE3f Tcw=slam->TrackRGBD(rgb->image,depth->image,rgb_msg->header.stamp.toSec());
        f.track_ms=double(Now()-track_start)*1e-6;
        f.track_returned=true;
        const auto state_start=Now();
        f.state=slam->GetTrackingState();
        f.state_ms=double(Now()-state_start)*1e-6;
        f.pose_valid=(f.state==ORB_SLAM3::Tracking::OK || f.state==ORB_SLAM3::Tracking::OK_KLT) &&
            Tcw.translation().allFinite() && Tcw.rotationMatrix().allFinite();
        const auto publish_start=Now();
        publish(Tcw,rgb_msg->header.stamp,f.pose_valid,f.state);
        f.publish_ms=double(Now()-publish_start)*1e-6;
        f.publish_called=true;
        if(f.pose_valid) {
            const Sophus::SE3f Twc=Tcw.inverse();
            Eigen::Quaternionf q(Twc.rotationMatrix());q.normalize();
            const auto t=Twc.translation();
            f.pose_valid=t.allFinite() && q.coeffs().allFinite();
            if(f.pose_valid) f.pose={{t.x(),t.y(),t.z(),q.x(),q.y(),q.z(),q.w()}};
        }
        f.pose_fresh=f.pose_valid && f.publish_called;
    }
    bool detail_;
    std::vector<Source> sources_;
    std::vector<Frame> frames_;
    std::vector<Event> events_;
    std::uint64_t seal_ns_=0,frame_overflow_=0,event_overflow_=0,identity_errors_=0;
    std::uint64_t callback_errors_=0,cpu_clock_errors_=0;
};
inline int SpinAndWrite(ros::NodeHandle& node,const std::string& node_name,
                        message_filters::Subscriber<sensor_msgs::Image>& rgb,
                        message_filters::Subscriber<sensor_msgs::Image>& depth,
                        Recorder& recorder,const std::string& directory,int timeout_s) {
    auto seal=node.advertiseService(node_name+"/scale_seal",&Recorder::Seal,&recorder);
    ROS_INFO("Scale observation ready: detail=%s expected=%zu bytes=%zu seal=%s/scale_seal",
             recorder.Detail()?"on":"off",recorder.Expected(),recorder.RetainedBytes(),node_name.c_str());
    bool idle=false,timeout=false;
    unsigned rgb_publishers=0,depth_publishers=0;
    while (ros::ok()) {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
        if (!recorder.SealNs()) continue;
        rgb_publishers=rgb.getSubscriber().getNumPublishers();
        depth_publishers=depth.getSubscriber().getNumPublishers();
        if (Now()-recorder.SealNs()>=std::uint64_t(timeout_s)*1000000000ULL) {
            timeout=true;break;
        }
        if (!rgb_publishers && !depth_publishers && ros::getGlobalCallbackQueue()->isEmpty()) {
            idle=true;break;
        }
    }
    rgb.unsubscribe();depth.unsubscribe();
    const bool written=recorder.Write(directory,idle,timeout,rgb_publishers,depth_publishers);
    return idle && written && recorder.Valid()?0:3;
}
inline bool WriteShutdown(const std::string& directory,std::uint64_t start) {
    std::ofstream out(directory+"/shutdown.json");
    out<<"{\"shutdown_returned\":true,\"shutdown_ms\":"<<std::setprecision(17)
       <<double(Now()-start)*1e-6<<",\"joined_verified\":null}\n";
    out.close();return bool(out);
}

} // namespace rgbd_scale
