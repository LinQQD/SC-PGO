#include <fstream>
#include <math.h>
#include <vector>
#include <deque>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <atomic>
#include <limits>
#include <csignal>
#include <csetjmp>
#include <execinfo.h>
#include <cstdlib>
#include <algorithm>
#include <filesystem>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <eigen3/Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "lio_loop_pgo/common.h"
#include "lio_loop_pgo/tic_toc.h"

#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

void sigsegv_handler(int sig) {
    std::cerr << "\n=== SIGSEGV (signal " << sig << ") at: ===" << std::endl;
    void* array[32];
    int size = backtrace(array, 32);
    char** symbols = backtrace_symbols(array, size);
    for (int i = 0; i < size; ++i) {
        std::cerr << "  #" << i << ": " << symbols[i] << std::endl;
    }
    free(symbols);
    std::cerr << "===========================" << std::endl;
    std::_Exit(139); // 128+11 = 139 (SIGSEGV exit code convention)
}

// sigsetjmp buffer for recovering from SIGSEGV inside ICP
static sigjmp_buf icp_jmpbuf;
static std::atomic<bool> icp_in_sigsegv {false};

// SIGSEGV handler for ICP recovery — longjmp back to safe point
void icp_sigsegv_handler(int sig) {
    icp_in_sigsegv.store(true);
    siglongjmp(icp_jmpbuf, 1);
}

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
struct TimedScanContextCloud {
    double stamp;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
};
std::deque<TimedScanContextCloud> scanContextBuf;
std::queue<sensor_msgs::msg::NavSatFix::ConstSharedPtr> gpsBuf;
struct SCLoopCandidate {
    int prev_idx;
    int curr_idx;
    float yaw_diff_rad;
};

std::queue<SCLoopCandidate> scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

std::atomic<double> lastOdomWallTime {-1.0};
std::atomic<double> lastFullResCloudWallTime {-1.0};
std::atomic<double> lastScanContextCloudWallTime {-1.0};
std::atomic<double> lastKeyframeWallTime {-1.0};
std::atomic<double> lastLoopFactorWallTime {-1.0};
std::atomic<double> lastIsamUpdateWallTime {-1.0};
std::atomic<bool> scPgoIdleReported {false};
const double SC_PGO_IDLE_SEC = 10.0;

static inline void markScPgoActive()
{
    scPgoIdleReported.store(false);
}

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
std::atomic<bool> gtSAMgraphMade {false};
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;
std::mutex mtxSC;
std::atomic<int> recentIdxUpdatedAtomic {0};
std::atomic<bool> loopQueuedOrRunning {false};
std::atomic<bool> pgoMapRefreshRequested {false};

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

// bool useGPS = true;
bool useGPS = false;
sensor_msgs::msg::NavSatFix::ConstSharedPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false; 
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

std::shared_ptr<rclcpp::Node> g_node;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subScanContextCloud;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr g_save_map_service;

std::string save_directory;
std::string map_save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
std::fstream pgTimeSaveStream;

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    std::vector<Pose6D> keyframePosesSnapshot;
    {
        std::lock_guard<std::mutex> lock(mKF);
        keyframePosesSnapshot = keyframePoses;
    }

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePosesSnapshot) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr &_laserOdometry)
{
    std::lock_guard<std::mutex> lock(mBuf);
    odometryBuf.push(_laserOdometry);
    lastOdomWallTime.store(rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
    markScPgoActive();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &_laserCloudFullRes)
{
    std::lock_guard<std::mutex> lock(mBuf);
    fullResBuf.push(_laserCloudFullRes);
    lastFullResCloudWallTime.store(rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
    markScPgoActive();
} // laserCloudFullResHandler

void scanContextCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &_scanContextCloud)
{
    std::lock_guard<std::mutex> lock(mBuf);
    scanContextBuf.push_back(TimedScanContextCloud{rclcpp::Time(_scanContextCloud->header.stamp).seconds(), _scanContextCloud});
    lastScanContextCloudWallTime.store(rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
    markScPgoActive();
    while (scanContextBuf.size() > 2000) {
        scanContextBuf.pop_front();
    }
} // scanContextCloudHandler

void gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &_gps)
{
    if(useGPS) {
        std::lock_guard<std::mutex> lock(mBuf);
        gpsBuf.push(_gps);
        markScPgoActive();
    }
} // gpsHandler

void reportScPgoIdleIfReady()
{
    if (scPgoIdleReported.load() || lastOdomWallTime.load() < 0.0)
        return;

    const double now = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();
    const double most_recent_input = std::max({
        lastOdomWallTime.load(),
        lastFullResCloudWallTime.load(),
        lastScanContextCloudWallTime.load()
    });
    const double most_recent_work = std::max({
        lastKeyframeWallTime.load(),
        lastLoopFactorWallTime.load(),
        lastIsamUpdateWallTime.load(),
        most_recent_input
    });

    size_t odom_queue_size = 0;
    size_t fullres_queue_size = 0;
    size_t icp_queue_size = 0;
    {
        std::lock_guard<std::mutex> lock(mBuf);
        odom_queue_size = odometryBuf.size();
        fullres_queue_size = fullResBuf.size();
        icp_queue_size = scLoopICPBuf.size();
    }

    size_t pending_factors = 0;
    size_t pending_initial = 0;
    {
        std::lock_guard<std::mutex> graph_lock(mtxPosegraph);
        pending_factors = gtSAMgraph.size();
        pending_initial = initialEstimate.size();
    }

    const bool no_recent_input = (now - most_recent_input) > SC_PGO_IDLE_SEC;
    const bool no_recent_work = (now - most_recent_work) > SC_PGO_IDLE_SEC;
    const bool buffers_drained = odom_queue_size == 0 && fullres_queue_size == 0;
    const bool loop_drained = icp_queue_size == 0 && !loopQueuedOrRunning.load();
    const bool graph_drained = pending_factors == 0 && pending_initial == 0;
    const bool map_refreshed = !pgoMapRefreshRequested.load();

    if (no_recent_input && no_recent_work && buffers_drained && loop_drained && graph_drained && map_refreshed) {
        RCLCPP_WARN(g_node->get_logger(), "[LIO-LoopPGO] idle: no new odom/cloud for %.1f s, loop/ICP queue drained, graph optimized. Safe to call: ros2 service call /save_map std_srvs/srv/Empty. remaining_odom=%zu remaining_cloud=%zu remaining_icp=%zu",
                 SC_PGO_IDLE_SEC, odom_queue_size, fullres_queue_size, icp_queue_size);
        scPgoIdleReported.store(true);
    }
}

void process_idle_status(void)
{
    rclcpp::WallRate rate(1.0);
    while (rclcpp::ok()) {
        rate.sleep();
        reportScPgoIdleIfReady();
    }
}

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D getOdom(nav_msgs::msg::Odometry::ConstSharedPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>& cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn.size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 4;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn.points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubPath( void )
{
    std::vector<Pose6D> keyframePosesUpdatedSnapshot;
    std::vector<double> keyframeTimesSnapshot;
    {
        std::lock_guard<std::mutex> lock(mKF);
        keyframePosesUpdatedSnapshot = keyframePosesUpdated;
        keyframeTimesSnapshot = keyframeTimes;
    }

    int publish_count = std::min(recentIdxUpdatedAtomic.load(), int(std::min(keyframePosesUpdatedSnapshot.size(), keyframeTimesSnapshot.size())));
    if (publish_count <= 0)
        return;

    // pub odom and path
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";
    for (int node_idx=0; node_idx < publish_count; node_idx++)
    {
        const Pose6D& pose_est = keyframePosesUpdatedSnapshot.at(node_idx); // upodated poses

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "/aft_pgo";
        odomAftPGOthis.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimesSnapshot.at(node_idx) * 1e9), RCL_ROS_TIME);
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        tf2::Quaternion q_tf;
        q_tf.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation = tf2::toMsg(q_tf);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    pubOdomAftPGO->publish(odomAftPGO); // last pose
    pubPathAftPGO->publish(pathAftPGO); // poses

    // TF deduplication — same logic as ROS1: skip if stamp + pose unchanged
    static rclcpp::Time last_tf_stamp;
    static Pose6D last_tf_pose {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double current_roll, current_pitch, current_yaw;
    tf2::Matrix3x3(tf2::Quaternion(
        odomAftPGO.pose.pose.orientation.x,
        odomAftPGO.pose.pose.orientation.y,
        odomAftPGO.pose.pose.orientation.z,
        odomAftPGO.pose.pose.orientation.w)).getRPY(current_roll, current_pitch, current_yaw);
    const Pose6D current_tf_pose {
        odomAftPGO.pose.pose.position.x,
        odomAftPGO.pose.pose.position.y,
        odomAftPGO.pose.pose.position.z,
        current_roll,
        current_pitch,
        current_yaw
    };

    rclcpp::Time current_tf_stamp(odomAftPGO.header.stamp);
    const bool same_stamp = (last_tf_stamp.nanoseconds() != 0) && (current_tf_stamp.nanoseconds() == last_tf_stamp.nanoseconds());
    const bool same_pose = std::abs(current_tf_pose.x - last_tf_pose.x) < 1e-9 &&
                           std::abs(current_tf_pose.y - last_tf_pose.y) < 1e-9 &&
                           std::abs(current_tf_pose.z - last_tf_pose.z) < 1e-9 &&
                           std::abs(current_tf_pose.roll - last_tf_pose.roll) < 1e-9 &&
                           std::abs(current_tf_pose.pitch - last_tf_pose.pitch) < 1e-9 &&
                           std::abs(current_tf_pose.yaw - last_tf_pose.yaw) < 1e-9;
    if (!(same_stamp && same_pose)) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = odomAftPGO.header.stamp;
        transform_stamped.header.frame_id = "camera_init";
        transform_stamped.child_frame_id = "/aft_pgo";
        transform_stamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
        transform_stamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
        transform_stamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
        transform_stamped.transform.rotation = odomAftPGO.pose.pose.orientation;
        g_tf_broadcaster->sendTransform(transform_stamped);
        last_tf_stamp = current_tf_stamp;
        last_tf_pose = current_tf_pose;
    }
} // pubPath

void updatePoses(void)
{
    std::lock_guard<std::mutex> lock(mKF);
    int estimate_size = std::min(int(isamCurrentEstimate.size()), int(keyframePosesUpdated.size()));
    for (int node_idx=0; node_idx < estimate_size; node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }

    if (estimate_size <= 0)
        return;

    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(estimate_size - 1);
    {
        std::lock_guard<std::mutex> pose_lock(mtxRecentPose);
        recentOptimizedX = lastOptimizedPose.translation().x();
        recentOptimizedY = lastOptimizedPose.translation().y();
    }

    recentIdxUpdatedAtomic.store(estimate_size);
    recentIdxUpdated = estimate_size;
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

pcl::PointCloud<PointType> makeICPCloud(const int& key, const int& submap_size)
{
    // Deep-copy cloud and pose data within lock — no shared_ptr manipulation.
    // Keyframe clouds are already downsampled (leaf=0.4) when stored,
    // so we skip the redundant VoxelGrid pass that caused SIGSEGV via
    // setInputCloud() holding a shared_ptr to a stack-referenced Ptr&.
    std::vector<pcl::PointCloud<PointType>> localCloudCopies;
    std::vector<Pose6D> localPoses;
    localCloudCopies.reserve(2 * submap_size + 1);
    localPoses.reserve(2 * submap_size + 1);

    {
        std::lock_guard<std::mutex> lock(mKF);
        for (int i = -submap_size; i <= submap_size; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()))
                continue;
            if (keyNear >= int(keyframePosesUpdated.size()))
                continue;
            // Dereference shared_ptr to get const ref — no shared_ptr copy,
            // no refcount change. Then deep-copy the points vector.
            const auto& src_cloud = *keyframeLaserClouds[keyNear];
            localCloudCopies.emplace_back();
            localCloudCopies.back().width = src_cloud.width;
            localCloudCopies.back().height = src_cloud.height;
            localCloudCopies.back().is_dense = src_cloud.is_dense;
            localCloudCopies.back().sensor_origin_ = src_cloud.sensor_origin_;
            localCloudCopies.back().sensor_orientation_ = src_cloud.sensor_orientation_;
            localCloudCopies.back().points = src_cloud.points; // deep copies the point vector
            localPoses.push_back(keyframePosesUpdated[keyNear]);
        }
    }
    RCLCPP_INFO(g_node->get_logger(), "[SC loop] Deep-copied %zu clouds for key=%d submap=%d", localCloudCopies.size(), key, submap_size);

    pcl::PointCloud<PointType> result;
    for (size_t i = 0; i < localCloudCopies.size(); ++i) {
        try {
            auto transformed = local2global(localCloudCopies[i], localPoses[i]);
            if (transformed->empty())
                continue;
            result += *transformed;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(g_node->get_logger(), "[SC loop] Exception transforming cloud %zu: %s. Skip.", i, e.what());
            continue;
        }
    }
    RCLCPP_INFO(g_node->get_logger(), "[SC loop] Merged cloud has %zu points", result.size());

    if (result.empty())
        return result;

    // Remove NaN
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(result, result, nan_indices);
    result.is_dense = true;

    return result;
} // makeICPCloud


std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx, float _yaw_diff_rad)
{
    // Build source (current keyframe alone) and target (history submap) clouds.
    // makeICPCloud returns by value — no shared_ptr refcount manipulation.
    int historyKeyframeSearchNum = 5;
    pcl::PointCloud<PointType> cureKeyframeCloud = makeICPCloud(_curr_kf_idx, 0);
    pcl::PointCloud<PointType> targetKeyframeCloud = makeICPCloud(_loop_kf_idx, historyKeyframeSearchNum);
    if (cureKeyframeCloud.empty() || targetKeyframeCloud.empty()) {
        RCLCPP_WARN(g_node->get_logger(), "[SC loop] Empty cloud for ICP. Skip this loop.");
        return std::nullopt;
    }

    std::vector<int> indices_source, indices_target;
    pcl::removeNaNFromPointCloud(cureKeyframeCloud, cureKeyframeCloud, indices_source);
    pcl::removeNaNFromPointCloud(targetKeyframeCloud, targetKeyframeCloud, indices_target);

    if (cureKeyframeCloud.size() < 50 || targetKeyframeCloud.size() < 200) {
        RCLCPP_WARN(g_node->get_logger(), "[SC loop] Too few valid points for ICP. source=%zu target=%zu",
                 cureKeyframeCloud.size(), targetKeyframeCloud.size());
        return std::nullopt;
    }

    // loop verification via ROS pub
    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopScanLocal->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(50);
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(5);

    // Wrap shared_ptrs around our value clouds for PCL's shared_ptr-based API
    // These are lightweight wrappers created just before use — no storage or aliasing.
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloudPtr(new pcl::PointCloud<PointType>(cureKeyframeCloud));
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloudPtr(new pcl::PointCloud<PointType>(targetKeyframeCloud));

    Pose6D curr_pose_est;
    Pose6D loop_pose_est;
    {
        std::lock_guard<std::mutex> lock(mKF);
        if (_curr_kf_idx < 0 || _loop_kf_idx < 0 ||
            _curr_kf_idx >= int(keyframePosesUpdated.size()) ||
            _loop_kf_idx >= int(keyframePosesUpdated.size())) {
            RCLCPP_WARN(g_node->get_logger(), "[SC loop] Invalid pose indices for ICP init. curr=%d loop=%d size=%zu",
                     _curr_kf_idx, _loop_kf_idx, keyframePosesUpdated.size());
            return std::nullopt;
        }
        curr_pose_est = keyframePosesUpdated[_curr_kf_idx];
        loop_pose_est = keyframePosesUpdated[_loop_kf_idx];
    }

    const float dx = static_cast<float>(loop_pose_est.x - curr_pose_est.x);
    const float dy = static_cast<float>(loop_pose_est.y - curr_pose_est.y);
    const float dz = static_cast<float>(loop_pose_est.z - curr_pose_est.z);

    icp.setInputSource(cureKeyframeCloudPtr);
    icp.setInputTarget(targetKeyframeCloudPtr);

    struct ICPInitGuessCandidate {
        Eigen::Matrix4f guess;
    };

    std::vector<ICPInitGuessCandidate> init_guess_candidates;
    // Final behavior: Scan Context only selects the loop candidate pair. Its yaw
    // is not used in ICP initialization; ICP receives translation only.
    Eigen::Affine3f guess_trans_zero_yaw = pcl::getTransformation(dx, dy, dz, 0.0f, 0.0f, 0.0f);
    init_guess_candidates.push_back({guess_trans_zero_yaw.matrix()});

    bool best_converged = false;
    double best_score = std::numeric_limits<double>::max();
    Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();

    RCLCPP_INFO(g_node->get_logger(), "[SC loop] ICP input source=%zu target=%zu trans_init=[%.3f %.3f %.3f]",
             cureKeyframeCloudPtr->size(), targetKeyframeCloudPtr->size(),
             dx, dy, dz);

    for (const auto& candidate : init_guess_candidates) {
        pcl::IterativeClosestPoint<PointType, PointType> icp_try;
        icp_try.setMaxCorrespondenceDistance(50);
        icp_try.setMaximumIterations(30);
        icp_try.setTransformationEpsilon(1e-6);
        icp_try.setEuclideanFitnessEpsilon(1e-6);
        icp_try.setRANSACIterations(5);
        icp_try.setInputSource(cureKeyframeCloudPtr);
        icp_try.setInputTarget(targetKeyframeCloudPtr);

        pcl::PointCloud<PointType> unused_result;
        icp_try.align(unused_result, candidate.guess);

        const bool converged = icp_try.hasConverged();
        const double score = icp_try.getFitnessScore();

        if (converged && std::isfinite(score) && score < best_score) {
            best_converged = true;
            best_score = score;
            best_transform = icp_try.getFinalTransformation();
        }
    }

    float loopFitnessScoreThreshold = 0.3;
    if (best_converged == false || best_score > loopFitnessScoreThreshold) {
        RCLCPP_INFO(g_node->get_logger(), "[SC loop] failed score=%f threshold=%.3f", best_score, loopFitnessScoreThreshold);
        return std::nullopt;
    } else {
        RCLCPP_INFO(g_node->get_logger(), "[SC loop] accepted score=%f threshold=%.3f", best_score, loopFitnessScoreThreshold);
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Matrix4f transform_matrix = best_transform;
    if (!transform_matrix.allFinite()) {
        RCLCPP_WARN(g_node->get_logger(), "[SC loop] ICP returned non-finite transform. Skip.");
        return std::nullopt;
    }

    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame.matrix() = transform_matrix;
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);

    // ICP was run between clouds that were already transformed to the global
    // camera_init frame. PCL therefore returns a global-frame correction A that
    // maps the current/source global cloud onto the history/target global cloud:
    //     p_history_world ~= A * p_current_world
    // The BetweenFactor(prev, curr) measurement must instead be the relative
    // pose from the history keyframe to the corrected current keyframe:
    //     Z_prev_curr = T_world_prev^-1 * (A * T_world_curr)
    gtsam::Pose3 icpCorrection = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 posePrev = Pose6DtoGTSAMPose3(loop_pose_est);
    gtsam::Pose3 poseCurr = Pose6DtoGTSAMPose3(curr_pose_est);
    gtsam::Pose3 correctedPoseCurr = icpCorrection.compose(poseCurr);

    return posePrev.between(correctedPoseCurr);
} // doICPVirtualRelative

std::optional<sensor_msgs::msg::PointCloud2::ConstSharedPtr> findNearestScanContextCloud(
    const double target_stamp,
    const double max_time_diff_sec)
{
    std::lock_guard<std::mutex> lock(mBuf);

    while (!scanContextBuf.empty() && scanContextBuf.front().stamp < target_stamp - max_time_diff_sec) {
        scanContextBuf.pop_front();
    }

    if (scanContextBuf.empty()) {
        return std::nullopt;
    }

    auto best_it = scanContextBuf.end();
    double best_diff = max_time_diff_sec;
    for (auto it = scanContextBuf.begin(); it != scanContextBuf.end(); ++it) {
        const double diff = std::abs(it->stamp - target_stamp);
        if (diff <= best_diff) {
            best_diff = diff;
            best_it = it;
        }
        if (it->stamp > target_stamp + max_time_diff_sec) {
            break;
        }
    }

    if (best_it == scanContextBuf.end()) {
        return std::nullopt;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr best_msg = best_it->msg;
    scanContextBuf.erase(scanContextBuf.begin(), std::next(best_it));
    return best_msg;
}

void process_pg()
{
    while(rclcpp::ok())
    {
        while ( true )
        {
            //
            // pop and check keyframe is or not
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr thisScanContextCloud(new pcl::PointCloud<PointType>());
            Pose6D pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            bool should_break = false;
            double thisKeyframeStamp = 0.0;
            {
                std::lock_guard<std::mutex> lock(mBuf);
                if (odometryBuf.empty() || fullResBuf.empty())
                {
                    should_break = true;
                }
                else
                {
                    while (!odometryBuf.empty() && rclcpp::Time(odometryBuf.front()->header.stamp).seconds() < rclcpp::Time(fullResBuf.front()->header.stamp).seconds())
                        odometryBuf.pop();
                    if (odometryBuf.empty())
                    {
                        should_break = true;
                    }
                    else
                    {
                    // Time equal check
                    timeLaserOdometry = rclcpp::Time(odometryBuf.front()->header.stamp).seconds();
                    timeLaser = rclcpp::Time(fullResBuf.front()->header.stamp).seconds();
                    thisKeyframeStamp = timeLaserOdometry;

                    laserCloudFullRes->clear();
                    pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
                    fullResBuf.pop();

                    pose_curr = getOdom(odometryBuf.front());
                    odometryBuf.pop();

                    // find nearest gps
                    hasGPSforThisKF = false;
                    double eps = 0.1; // find a gps topioc arrived within eps second
                    while (!gpsBuf.empty()) {
                        auto thisGPS = gpsBuf.front();
                        auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
                        if( abs(thisGPSTime - timeLaserOdometry) < eps ) {
                            currGPS = thisGPS;
                            hasGPSforThisKF = true;
                            break;
                        }
                        gpsBuf.pop();
                    }
                    }
                }
            }
            if (should_break)
                break;

            constexpr double kScanContextMatchToleranceSec = 0.05;
            const auto scan_context_msg = findNearestScanContextCloud(thisKeyframeStamp, kScanContextMatchToleranceSec);
            if (scan_context_msg) {
                pcl::fromROSMsg(*scan_context_msg.value(), *thisScanContextCloud);
            }

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
                continue; 

            if( !gpsOffsetInitialized ) {
                if(hasGPSforThisKF) { // if the very first frame 
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                } 
            }

            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            pcl::PointCloud<PointType>::Ptr thisScanContextCloudDS(new pcl::PointCloud<PointType>());
            if (!thisScanContextCloud->empty()) {
                downSizeFilterScancontext.setInputCloud(thisScanContextCloud);
                downSizeFilterScancontext.filter(*thisScanContextCloudDS);
            }

            int prev_node_idx = -1;
            int curr_node_idx = -1;
            Pose6D pose_prev_copy;
            Pose6D pose_curr_copy;
            bool has_prev_node = false;
            {
                std::lock_guard<std::mutex> lock(mKF);
                keyframeLaserClouds.push_back(thisKeyFrameDS);
                keyframePoses.push_back(pose_curr);
                keyframePosesUpdated.push_back(pose_curr); // init
                keyframeTimes.push_back(timeLaserOdometry);

                curr_node_idx = int(keyframePoses.size()) - 1;
                if (curr_node_idx > 0) {
                    prev_node_idx = curr_node_idx - 1;
                    pose_prev_copy = keyframePoses.at(prev_node_idx);
                    has_prev_node = true;
                }
                pose_curr_copy = keyframePoses.at(curr_node_idx);
                laserCloudMapPGORedraw = true;
                pgoMapRefreshRequested.store(true);
                lastKeyframeWallTime.store(rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
                markScPgoActive();
            }

            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0;
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(pose_curr_copy);

                std::lock_guard<std::mutex> graph_lock(mtxPosegraph);
                gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                initialEstimate.insert(init_node_idx, poseOrigin);
                gtSAMgraphMade.store(true);

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else if (has_prev_node) {
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(pose_prev_copy);
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(pose_curr_copy);

                std::lock_guard<std::mutex> graph_lock(mtxPosegraph);
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                if(hasGPSforThisKF) {
                    double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                    double recent_x, recent_y;
                    {
                        std::lock_guard<std::mutex> pose_lock(mtxRecentPose);
                        recent_x = recentOptimizedX;
                        recent_y = recentOptimizedY;
                    }
                    gtsam::Point3 gpsConstraint(recent_x, recent_y, curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set)
                    gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                    cout << "GPS factor added at node " << curr_node_idx << endl;
                }
                initialEstimate.insert(curr_node_idx, poseTo);

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }

            {
                std::lock_guard<std::mutex> lock(mtxSC);
                const pcl::PointCloud<PointType>::Ptr& scInputCloud = thisScanContextCloudDS->empty() ? thisKeyFrameDS : thisScanContextCloudDS;
                scManager.makeAndSaveScancontextAndKeys(*scInputCloud);
            }

            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

            // save utility
            pgTimeSaveStream << timeLaser << std::endl; // path 
        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void performSCLoopClosure(void)
{
    int keyframe_count;
    {
        std::lock_guard<std::mutex> lock(mKF);
        keyframe_count = int(keyframePoses.size());
    }

    if( keyframe_count < scManager.NUM_EXCLUDE_RECENT) // do not try too early
        return;

    std::pair<int, float> detectResult;
    int curr_sc_idx = -1;
    {
        std::lock_guard<std::mutex> lock(mtxSC);
        detectResult = scManager.detectLoopClosureID();
        curr_sc_idx = int(scManager.polarcontexts_.size()) - 1;
    }

    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) {
        int curr_node_idx;
        {
            std::lock_guard<std::mutex> lock(mKF);
            if (curr_sc_idx < 0 || curr_sc_idx >= int(keyframePoses.size())) {
                RCLCPP_WARN(g_node->get_logger(), "[SC loop] Scan Context index %d is inconsistent with %zu keyframes. Skip.", curr_sc_idx, keyframePoses.size());
                return;
            }
            curr_node_idx = curr_sc_idx;
        }

        if( loopQueuedOrRunning.load() )
            return;

        const int prev_node_idx = SCclosestHistoryFrameID;
        const float yaw_diff_rad = detectResult.second;
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        {
            std::lock_guard<std::mutex> lock(mBuf);
            scLoopICPBuf.push(SCLoopCandidate{prev_node_idx, curr_node_idx, yaw_diff_rad});
            loopQueuedOrRunning.store(true);
            markScPgoActive();
        }
        // addding actual 6D constraints in the other thread, icp_calculation.
    }
} // performSCLoopClosure

void process_lcd(void)
{
    float loopClosureFrequency = 1.0; // can change 
    rclcpp::WallRate rate(loopClosureFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void process_icp(void)
{
    while(rclcpp::ok())
    {
        SCLoopCandidate loop_candidate;
        bool has_loop_candidate = false;
        {
            std::lock_guard<std::mutex> lock(mBuf);
            if( scLoopICPBuf.size() > 30 ) {
                RCLCPP_WARN(g_node->get_logger(), "Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }
            if ( !scLoopICPBuf.empty() ) {
                loop_candidate = scLoopICPBuf.front();
                scLoopICPBuf.pop();
                has_loop_candidate = true;
            }
        }

        if (has_loop_candidate)
        {
            markScPgoActive();
            const int prev_node_idx = loop_candidate.prev_idx;
            const int curr_node_idx = loop_candidate.curr_idx;
            const float yaw_diff_rad = loop_candidate.yaw_diff_rad;

            {
                std::lock_guard<std::mutex> lock(mKF);
                if (prev_node_idx < 0 || curr_node_idx < 0 ||
                    prev_node_idx >= int(keyframeLaserClouds.size()) ||
                    curr_node_idx >= int(keyframeLaserClouds.size()) ||
                    prev_node_idx >= int(keyframePosesUpdated.size()) ||
                    curr_node_idx >= int(keyframePosesUpdated.size())) {
                    RCLCPP_WARN(g_node->get_logger(), "[SC loop] Invalid keyframe index pair (%d, %d), skip.", prev_node_idx, curr_node_idx);
                    loopQueuedOrRunning.store(false);
                    continue;
                }
            }

            RCLCPP_INFO(g_node->get_logger(), "[SC loop] Start ICP for (%d, %d).", prev_node_idx, curr_node_idx);
            std::optional<gtsam::Pose3> relative_pose_optional = std::nullopt;

            // Install ICP SIGSEGV handler and set jump point for recovery
            struct sigaction icp_act, old_act;
            icp_act.sa_handler = icp_sigsegv_handler;
            sigemptyset(&icp_act.sa_mask);
            icp_act.sa_flags = 0;
            sigaction(SIGSEGV, &icp_act, &old_act);

            icp_in_sigsegv.store(false);
            if (sigsetjmp(icp_jmpbuf, 1) == 0) {
                // Normal path — run ICP
                relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx, yaw_diff_rad);
            } else {
                // SIGSEGV recovered — skip this loop candidate
                RCLCPP_ERROR(g_node->get_logger(), "[SC loop] Recovered from SIGSEGV during ICP (%d, %d). Skipping loop.",
                          prev_node_idx, curr_node_idx);
            }

            // Restore default SIGSEGV handler
            sigaction(SIGSEGV, &old_act, nullptr);

            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                {
                    std::lock_guard<std::mutex> graph_lock(mtxPosegraph);
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                }
                lastLoopFactorWallTime.store(rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
                markScPgoActive();
                pgoMapRefreshRequested.store(true);
                RCLCPP_INFO(g_node->get_logger(), "[SC loop] Loop factor inserted for (%d, %d).", prev_node_idx, curr_node_idx);
            }
            loopQueuedOrRunning.store(false);
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void process_viz_path(void)
{
    float hz = 10.0;
    rclcpp::WallRate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdatedAtomic.load() > 1) {
            pubPath();
        }
    }
}

void process_isam(void)
{
    float hz = 1;
    rclcpp::WallRate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();

        bool has_pending_graph = false;
        {
            std::lock_guard<std::mutex> graph_lock(mtxPosegraph);
            has_pending_graph = gtSAMgraphMade && (gtSAMgraph.size() > 0 || initialEstimate.size() > 0);
        }
        if (!has_pending_graph)
            continue;

        gtsam::Values estimateSnapshot;
        try {
            std::lock_guard<std::mutex> graph_lock(mtxPosegraph);
            RCLCPP_INFO(g_node->get_logger(), "[PGO] Start ISAM2 optimization.");
            runISAM2opt();
            lastIsamUpdateWallTime.store(rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
            markScPgoActive();
            pgoMapRefreshRequested.store(true);
            estimateSnapshot = isamCurrentEstimate;
            RCLCPP_INFO(g_node->get_logger(), "[PGO] Finished ISAM2 optimization with %zu estimates.", estimateSnapshot.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(g_node->get_logger(), "[PGO] ISAM2 optimization failed: %s", e.what());
            continue;
        } catch (...) {
            RCLCPP_ERROR(g_node->get_logger(), "[PGO] ISAM2 optimization failed with unknown exception.");
            continue;
        }

        saveOptimizedVerticesKITTIformat(estimateSnapshot, pgKITTIformat); // pose
        saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 2; // sparse map visulalization to save computations
    int counter = 0;

    std::vector<pcl::PointCloud<PointType>::Ptr> cloudsSnapshot;
    std::vector<Pose6D> posesSnapshot;
    {
        std::lock_guard<std::mutex> lock(mKF);
        int map_count = std::min(recentIdxUpdatedAtomic.load(), int(std::min(keyframeLaserClouds.size(), keyframePosesUpdated.size())));
        cloudsSnapshot.reserve(map_count);
        posesSnapshot.reserve(map_count);
        for (int node_idx=0; node_idx < map_count; node_idx++) {
            cloudsSnapshot.push_back(keyframeLaserClouds[node_idx]);
            posesSnapshot.push_back(keyframePosesUpdated[node_idx]);
        }
    }

    laserCloudMapPGO->clear();
    for (size_t node_idx=0; node_idx < cloudsSnapshot.size(); node_idx++) {
        if(counter % SKIP_FRAMES == 0 && cloudsSnapshot[node_idx]) {
            *laserCloudMapPGO += *local2global(*cloudsSnapshot[node_idx], posesSnapshot[node_idx]);
        }
        counter++;
    }

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*laserCloudMapPGO, *laserCloudMapPGO, nan_indices);

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    nan_indices.clear();
    pcl::removeNaNFromPointCloud(*laserCloudMapPGO, *laserCloudMapPGO, nan_indices);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "camera_init";
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

/// Build and save two PCD maps for comparison:
///   - corrected_map.pcd  — keyframe clouds transformed by ISAM2-optimized poses
///   - uncorrected_map.pcd — same keyframe clouds transformed by raw odometry poses
void saveBothMaps(void)
{
    // Snapshot all data under lock
    std::vector<pcl::PointCloud<PointType>::Ptr> cloudsSnapshot;
    std::vector<Pose6D> odomPoses;     // raw odometry poses (uncorrected)
    std::vector<Pose6D> optPoses;      // ISAM2-optimized poses (corrected)
    {
        std::lock_guard<std::mutex> lock(mKF);
        int n = std::min(keyframeLaserClouds.size(), keyframePoses.size());
        n = std::min(int(n), int(keyframePosesUpdated.size()));
        cloudsSnapshot.reserve(n);
        odomPoses.reserve(n);
        optPoses.reserve(n);
        for (int i = 0; i < n; i++) {
            cloudsSnapshot.push_back(keyframeLaserClouds[i]);
            odomPoses.push_back(keyframePoses[i]);
            optPoses.push_back(keyframePosesUpdated[i]);
        }
    }

    if (cloudsSnapshot.empty()) {
        RCLCPP_WARN(g_node->get_logger(), "[saveBothMaps] No keyframes to save.");
        return;
    }

    // --- Corrected map (after PGO) ---
    pcl::PointCloud<PointType>::Ptr correctedMap(new pcl::PointCloud<PointType>());
    for (size_t i = 0; i < cloudsSnapshot.size(); i++) {
        if (cloudsSnapshot[i])
            *correctedMap += *local2global(*cloudsSnapshot[i], optPoses[i]);
    }
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(0.4f, 0.4f, 0.4f);
    vg.setInputCloud(correctedMap);
    pcl::PointCloud<PointType>::Ptr filteredCorrected(new pcl::PointCloud<PointType>());
    vg.filter(*filteredCorrected);

    std::string correctedPath = map_save_directory + "corrected_map.pcd";
    if (pcl::io::savePCDFileASCII(correctedPath, *filteredCorrected) == 0)
        RCLCPP_INFO(g_node->get_logger(), "[saveBothMaps] Corrected map saved: %s (%zu points)", correctedPath.c_str(), filteredCorrected->size());
    else
        RCLCPP_ERROR(g_node->get_logger(), "[saveBothMaps] Failed to save corrected map to %s", correctedPath.c_str());

    // --- Uncorrected map (raw odometry only) ---
    pcl::PointCloud<PointType>::Ptr uncorrectedMap(new pcl::PointCloud<PointType>());
    for (size_t i = 0; i < cloudsSnapshot.size(); i++) {
        if (cloudsSnapshot[i])
            *uncorrectedMap += *local2global(*cloudsSnapshot[i], odomPoses[i]);
    }
    vg.setInputCloud(uncorrectedMap);
    pcl::PointCloud<PointType>::Ptr filteredUncorrected(new pcl::PointCloud<PointType>());
    vg.filter(*filteredUncorrected);

    std::string uncorrectedPath = map_save_directory + "uncorrected_map.pcd";
    if (pcl::io::savePCDFileASCII(uncorrectedPath, *filteredUncorrected) == 0)
        RCLCPP_INFO(g_node->get_logger(), "[saveBothMaps] Uncorrected map saved: %s (%zu points)", uncorrectedPath.c_str(), filteredUncorrected->size());
    else
        RCLCPP_ERROR(g_node->get_logger(), "[saveBothMaps] Failed to save uncorrected map to %s", uncorrectedPath.c_str());

    // Also save a version of the corrected map downsampled to 0.1 m for CloudCompare viewing
    pcl::PointCloud<PointType>::Ptr denseCorrected(new pcl::PointCloud<PointType>());
    *denseCorrected = *correctedMap;
    pcl::VoxelGrid<PointType> vgDense;
    vgDense.setLeafSize(0.1f, 0.1f, 0.1f);
    vgDense.setInputCloud(denseCorrected);
    pcl::PointCloud<PointType>::Ptr denseFiltered(new pcl::PointCloud<PointType>());
    vgDense.filter(*denseFiltered);
    std::string densePath = map_save_directory + "corrected_map_dense.pcd";
    pcl::io::savePCDFileASCII(densePath, *denseFiltered);

    RCLCPP_INFO(g_node->get_logger(), "[saveBothMaps] Maps saved under %s", map_save_directory.c_str());
}

// ROS service callback: /save_map
void saveMapCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
    RCLCPP_INFO(g_node->get_logger(), "[saveMapCallback] Triggered by service call.");
    saveBothMaps();
}

void process_viz_map(void)
{
    float vizmapFrequency = 0.5; // refresh map visualization every 2s when requested
    rclcpp::WallRate rate(vizmapFrequency);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdatedAtomic.load() > 1 && pgoMapRefreshRequested.exchange(false)) {
            pubMap();
        }
    }
} // pointcloud_viz


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("laserPGO");
    g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);

    // Install SIGSEGV handler for debugging loop-closure crashes
    signal(SIGSEGV, sigsegv_handler);

    g_node->declare_parameter<std::string>("save_directory", "/");
    save_directory = g_node->get_parameter("save_directory").as_string();
    g_node->declare_parameter<std::string>("map_save_directory", save_directory);
    map_save_directory = g_node->get_parameter("map_save_directory").as_string();
    pgKITTIformat = save_directory + "optimized_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";
    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out);
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
    pgScansDirectory = save_directory + "Scans/";
    std::error_code fs_error;
    std::filesystem::remove_all(pgScansDirectory, fs_error);
    std::filesystem::create_directories(pgScansDirectory, fs_error);
    std::filesystem::create_directories(map_save_directory, fs_error);

    g_node->declare_parameter<double>("keyframe_meter_gap", 2.0);
    keyframeMeterGap = g_node->get_parameter("keyframe_meter_gap").as_double();
    g_node->declare_parameter<double>("keyframe_deg_gap", 10.0);
    keyframeDegGap = g_node->get_parameter("keyframe_deg_gap").as_double();
    keyframeRadGap = deg2rad(keyframeDegGap);

    g_node->declare_parameter<double>("sc_dist_thres", 0.2);
    scDistThres = g_node->get_parameter("sc_dist_thres").as_double();
    g_node->declare_parameter<double>("sc_max_radius", 80.0);
    scMaximumRadius = g_node->get_parameter("sc_max_radius").as_double();

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    float filter_size = 0.4;
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
    g_node->declare_parameter<double>("mapviz_filter_size", 0.4);
    mapVizFilterSize = g_node->get_parameter("mapviz_filter_size").as_double();
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

    subLaserCloudFullRes = g_node->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
    subScanContextCloud = g_node->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_for_scancontext", 100, scanContextCloudHandler);
    subLaserOdometry = g_node->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
    subGPS = g_node->create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", 100, gpsHandler);

    pubOdomAftPGO = g_node->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
    pubOdomRepubVerifier = g_node->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", 100);
    pubPathAftPGO = g_node->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
    pubMapAftPGO = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);

    pubLoopScanLocal = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);

    std::thread posegraph_slam {process_pg}; // pose graph construction
    std::thread lc_detection {process_lcd}; // loop closure detection
    // Service to trigger map saving on demand:  ros2 service call /save_map std_srvs/srv/Empty
    g_save_map_service = g_node->create_service<std_srvs::srv::Empty>("/save_map", saveMapCallback);

    std::thread icp_calculation {process_icp}; // loop constraint calculation via icp
    std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added.

    std::thread viz_map {process_viz_map}; // visualization - map (refresh only after loop insertion)
    std::thread viz_path {process_viz_path}; // visualization - path (high frequency)
    std::thread idle_status {process_idle_status}; // one-shot idle prompt for safe map saving

    rclcpp::spin(g_node);
    // Auto-save maps on shutdown (after spin returns)
    RCLCPP_INFO(g_node->get_logger(), "[main] Shutting down, saving both maps...");
    saveBothMaps();
    rclcpp::shutdown();

    posegraph_slam.join();
    lc_detection.join();
    icp_calculation.join();
    isam_update.join();
    viz_map.join();
    viz_path.join();
    idle_status.join();

    return 0;
}
