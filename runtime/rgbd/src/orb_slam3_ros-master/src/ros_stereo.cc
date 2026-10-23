/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
* Refactored with common module integration
*/

#include "common.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#ifdef ORB_SLAM3_WITH_XFEAT
#include "xfeat_hybrid_depth_provider.hpp"
#include "xfeat_stereo_feature_provider.hpp"
#endif

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, const std::string &stats_path) : mpSLAM(pSLAM)
    {
        if (!stats_path.empty())
        {
            stats_stream.open(stats_path, std::ios::out | std::ios::trunc);
            if (!stats_stream)
                throw std::runtime_error("Cannot open frontend stats CSV: " + stats_path);
            stats_stream << "frame_id,stamp_ns,tracking_state,keypoints,valid_depths,"
                            "valid_depth_ratio,depth_grid_coverage,tracked_inliers,"
                            "feature_extract_ms,stereo_match_ms,"
                            "hybrid_candidates,hybrid_fine_matches,hybrid_accepted,"
                            "hybrid_added,hybrid_replaced,hybrid_rejected,hybrid_ms,"
                            "frontend_ms,track_total_ms,pose_valid,"
                            "tcw_tx,tcw_ty,tcw_tz,tcw_qx,tcw_qy,tcw_qz,tcw_qw\n";
        }
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);

private:
    ORB_SLAM3::System* mpSLAM;
    std::ofstream stats_stream;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;

    // 必需的参数 - VOC文件和设置文件
    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    } 

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, false);
    
    cv::FileStorage settings(settings_file, cv::FileStorage::READ);
    if (!settings.isOpened())
    {
        ROS_ERROR("Cannot open settings file: %s", settings_file.c_str());
        return 1;
    }
    std::string feature_type = "ORB";
    if (!settings["Feature.Type"].empty())
        settings["Feature.Type"] >> feature_type;
    if (feature_type == "XFeat" &&
        (settings["loopClosing"].empty() ||
         static_cast<int>(settings["loopClosing"]) != 0))
    {
        ROS_ERROR("Feature.Type=XFeat requires loopClosing: 0 until F24 adds XFeat vocabulary support");
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    if (feature_type == "Hybrid")
    {
#ifdef ORB_SLAM3_WITH_XFEAT
        ORB_SLAM3::XFeatHybridDepthOptions options;
        settings["XFeat.BackboneModel"] >> options.backbone_model;
        settings["XFeat.FineModel"] >> options.fine_model;
        if (!settings["XFeat.MaxFeatures"].empty()) settings["XFeat.MaxFeatures"] >> options.max_features;
        if (!settings["XFeat.MinimumCosine"].empty()) settings["XFeat.MinimumCosine"] >> options.minimum_cosine;
        if (!settings["XFeat.RatioThreshold"].empty()) settings["XFeat.RatioThreshold"] >> options.ratio_threshold;
        if (!settings["XFeat.FineConfidence"].empty()) settings["XFeat.FineConfidence"] >> options.fine_confidence;
        if (!settings["XFeat.MaximumDisparity"].empty()) settings["XFeat.MaximumDisparity"] >> options.maximum_disparity;
        if (!settings["XFeat.VerticalTolerance"].empty()) settings["XFeat.VerticalTolerance"] >> options.vertical_tolerance;
        if (!settings["XFeat.MinimumPatchNCC"].empty()) settings["XFeat.MinimumPatchNCC"] >> options.minimum_patch_ncc;
        if (!settings["XFeat.AssociationRadius"].empty()) settings["XFeat.AssociationRadius"] >> options.association_radius;
        if (!settings["XFeat.ReplacementNCCMargin"].empty()) settings["XFeat.ReplacementNCCMargin"] >> options.replacement_ncc_margin;
        if (!settings["XFeat.MatchGridMaximumPerCell"].empty()) settings["XFeat.MatchGridMaximumPerCell"] >> options.match_grid_maximum_per_cell;
        try
        {
            std::shared_ptr<ORB_SLAM3::StereoDepthProvider> provider(
                new ORB_SLAM3::XFeatHybridDepthProvider(options));
            pSLAM->SetStereoDepthProvider(provider);
            ROS_INFO("Feature.Type=Hybrid: XFeat Fine depth refinement enabled");
        }
        catch (const std::exception &error)
        {
            ROS_ERROR("Cannot initialize Hybrid XFeat: %s", error.what());
            pSLAM->Shutdown();
            delete pSLAM;
            return 1;
        }
#else
        ROS_ERROR("Feature.Type=Hybrid requested, but this binary has no XFeat support");
        pSLAM->Shutdown();
        delete pSLAM;
        return 1;
#endif
    }
    else if (feature_type == "XFeat")
    {
#ifdef ORB_SLAM3_WITH_XFEAT
        ORB_SLAM3::XFeatStereoFeatureOptions options;
        settings["XFeat.BackboneModel"] >> options.backbone_model;
        settings["XFeat.FineModel"] >> options.fine_model;
        if (!settings["XFeat.MaxFeatures"].empty()) settings["XFeat.MaxFeatures"] >> options.max_features;
        if (!settings["XFeat.FineBatchSize"].empty()) settings["XFeat.FineBatchSize"] >> options.fine_batch_size;
        if (!settings["XFeat.MinimumCosine"].empty()) settings["XFeat.MinimumCosine"] >> options.minimum_cosine;
        if (!settings["XFeat.RatioThreshold"].empty()) settings["XFeat.RatioThreshold"] >> options.ratio_threshold;
        if (!settings["XFeat.FineConfidence"].empty()) settings["XFeat.FineConfidence"] >> options.fine_confidence;
        if (!settings["XFeat.MaximumDisparity"].empty()) settings["XFeat.MaximumDisparity"] >> options.maximum_disparity;
        if (!settings["XFeat.VerticalTolerance"].empty()) settings["XFeat.VerticalTolerance"] >> options.vertical_tolerance;
        if (!settings["XFeat.MinimumPatchNCC"].empty()) settings["XFeat.MinimumPatchNCC"] >> options.minimum_patch_ncc;
        if (!settings["XFeat.ExtractionGridMaximumPerCell"].empty()) settings["XFeat.ExtractionGridMaximumPerCell"] >> options.extraction_grid_maximum_per_cell;
        if (!settings["XFeat.MatchGridMaximumPerCell"].empty()) settings["XFeat.MatchGridMaximumPerCell"] >> options.match_grid_maximum_per_cell;
        try
        {
            std::shared_ptr<ORB_SLAM3::StereoFeatureProvider> provider(
                new ORB_SLAM3::XFeatStereoFeatureProvider(options));
            pSLAM->SetStereoFeatureProvider(provider);
            ROS_INFO("Feature.Type=XFeat: ORB detection, descriptors, and stereo matching replaced by XFeat");
        }
        catch (const std::exception &error)
        {
            ROS_ERROR("Cannot initialize full XFeat front-end: %s", error.what());
            pSLAM->Shutdown();
            delete pSLAM;
            return 1;
        }
#else
        ROS_ERROR("Feature.Type=XFeat requested, but this binary has no XFeat support");
        pSLAM->Shutdown();
        delete pSLAM;
        return 1;
#endif
    }
    else if (feature_type != "ORB")
    {
        ROS_ERROR("Unsupported Feature.Type '%s'; expected ORB, Hybrid, or XFeat", feature_type.c_str());
        pSLAM->Shutdown();
        delete pSLAM;
        return 1;
    }

    std::string frontend_stats_csv;
    node_handler.param<std::string>(node_name + "/frontend_stats_csv",
                                    frontend_stats_csv,
                                    "/tmp/orb_slam3_frontend.csv");
    ImageGrabber igb(pSLAM, frontend_stats_csv);

    message_filters::Subscriber<sensor_msgs::Image> sub_img_left(node_handler, "/camera/infra1/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_right(node_handler, "/camera/infra2/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    setup_publishers(node_handler, node_name);
    setup_services(node_handler, node_name);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    ros::Time msg_time = msgLeft->header.stamp;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackStereo()
    const auto track_start = std::chrono::steady_clock::now();
    Sophus::SE3f Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msg_time.toSec());
    const auto track_end = std::chrono::steady_clock::now();
    const double track_total_ms =
        std::chrono::duration<double, std::milli>(track_end - track_start).count();
    const ORB_SLAM3::FrontendStats stats = mpSLAM->GetLastFrontendStats(track_total_ms);
    const int tracking_state = mpSLAM->GetTrackingState();
    const bool tracking_ok = tracking_state == ORB_SLAM3::Tracking::OK ||
                           tracking_state == ORB_SLAM3::Tracking::OK_KLT;
    const bool pose_ok = tracking_ok && Tcw.translation().allFinite() &&
                         Tcw.rotationMatrix().allFinite();
    const Eigen::Quaternionf quaternion(Tcw.rotationMatrix());
    if (stats_stream && stats.valid)
    {
        stats_stream << stats.frame_id << ',' << msg_time.toNSec() << ','
                     << stats.tracking_state << ',' << stats.keypoints << ','
                     << stats.valid_depths << ',' << stats.valid_depth_ratio << ','
                     << stats.depth_grid_coverage << ',' << stats.tracked_inliers << ','
                     << std::fixed << std::setprecision(6)
                     << stats.feature_extract_ms << ',' << stats.stereo_match_ms << ','
                     << stats.hybrid_candidates << ',' << stats.hybrid_fine_matches << ','
                     << stats.hybrid_accepted << ',' << stats.hybrid_added << ','
                     << stats.hybrid_replaced << ',' << stats.hybrid_rejected << ','
                     << stats.hybrid_ms << ','
                     << (stats.feature_extract_ms + stats.stereo_match_ms) << ','
                     << stats.track_total_ms << ',' << (pose_ok ? 1 : 0) << ','
                     << Tcw.translation().x() << ',' << Tcw.translation().y() << ','
                     << Tcw.translation().z() << ',' << quaternion.x() << ','
                     << quaternion.y() << ',' << quaternion.z() << ','
                     << quaternion.w() << '\n';
        stats_stream.flush();
    }

    publish_pose_and_odometry(Tcw, msg_time, pose_ok);
    
}
