// scan_fuser_node.cpp
//
// Fuses an RGB-D depth PointCloud2 and a 2D LiDAR LaserScan into a single
// height-gated, floor-rejected, self-masked LaserScan (/scan_fused) expressed
// in base_footprint, suitable for slam_toolbox + the Nav2 global costmap.
//
// =============================================================================
// REVIEW FIXES APPLIED (and contradiction resolutions)
// -----------------------------------------------------------------------------
// [MAJOR] timerCb now wrapped in try/catch (no exception may escape any
//         callback). All time-age math uses same-clock doubles, never
//         rclcpp::Time subtraction across possibly-mixed sources.
// [MAJOR] Depth staleness is measured on the NODE clock: processCloud records
//         last_depth_recv_ = this->now() (node clock) at commit time; timerCb
//         compares now() against that. This is robust to camera/node clock skew
//         and to real RGB-D pipeline latency (Astra). The sensor stamp is used
//         ONLY for the TF lookup, never for staleness. (slam-validity + rclcpp)
// [MAJOR] Duplicate-stamp suppression: the fused scan is published only when the
//         cached LiDAR stamp has ADVANCED since the last publish, so slam_toolbox
//         never sees non-increasing/duplicate stamps even though publish_rate
//         (12 Hz) > LiDAR rate (~10 Hz). The free-running timer is retained for
//         depth-staleness re-evaluation but is now idempotent w.r.t. stamps.
// [MAJOR] subsampleToPcl strides the RAW PointCloud2 byte buffer via
//         PointCloud2ConstIterator (skipping rows AND cols by pixel_step); the
//         full cloud is never deserialized, so pixel_step actually saves
//         ~pixel_step^2 of deserialize+alloc cost on the Pi5 hot path.
// [MAJOR] Cloud subscription QoS is KeepLast(1) BEST_EFFORT so the middleware
//         itself drops all but the newest cloud (true drop-not-queue); /scan
//         stays at SensorDataQoS (depth 5).
// [minor] MultiThreadedExecutor sized to >= 2 threads explicitly so the two
//         callback groups never collapse to serial on a low-core Pi5.
// [minor] Publish timer uses create_timer (node-clock) not create_wall_timer,
//         so cadence honors use_sim_time.
// [minor] RANSAC: setMaxIterations(50) + setProbability(0.95) for early exit;
//         processCloud wall-time is measured and WARN_THROTTLE'd if it exceeds
//         the depth period so depth-starvation is visible on-robot.
// [minor] Parsed ignore sectors are logged; near-degenerate (lo==hi) sectors
//         warn; a pair whose raw span >= 360 deg is treated as full-circle mask.
// [minor] canTransform timeout kept at 0.05s but documented: cloudCb runs in its
//         own group on a dedicated thread, and the executor is sized >=2, so this
//         wait cannot starve the LiDAR/publish group.
// [nit]   #include <Eigen/Core> made explicit. scan_time/time_increment now
//         reflect the backbone scan (scan_time = source scan_time, time_increment
//         = 0 for an instantaneous resample). Eigen normal-z guard comment fixed
//         (eps_angle is the real wall protection; |nz| guard is a degenerate
//         backstop).
//
// CONTRADICTION RESOLUTIONS:
//  - sub-range_min clamp: kept SNAP-to-range_min (safety: never hide a too-close
//    obstacle from the costmap). In practice only LiDAR returns in
//    [scan.range_min, range_min) reach this; depth_range_min=0.58 > range_min.
//  - We keep BOTH a free-running node-clock timer (re-evaluates depth staleness
//    so depth dropping out promptly collapses to LiDAR-only) AND stamp-advance
//    gating (so no duplicate stamps reach slam). This satisfies both the
//    "timer at publish_rate" spec and the "no duplicate stamps" requirement.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Normalize an angle into [-PI, PI).
inline double normAngle(double a)
{
  a = std::fmod(a + kPi, kTwoPi);
  if (a < 0.0) {
    a += kTwoPi;
  }
  return a - kPi;
}
}  // namespace

class ScanFuser : public rclcpp::Node
{
public:
  ScanFuser()
  : rclcpp::Node("scan_fuser")
  {
    declareParams();
    loadParams();

    // Precompute the output angular grid (constant for node lifetime).
    angle_min_ = -kPi;
    angle_increment_ = kTwoPi / static_cast<double>(num_bins_);
    angle_max_ = angle_min_ + (num_bins_ - 1) * angle_increment_;

    depth_bins_.assign(num_bins_, std::numeric_limits<float>::infinity());
    has_depth_ = false;
    has_scan_ = false;

    parseIgnoreSectors();

    RCLCPP_INFO(
      this->get_logger(), "use_sim_time=%s node_clock_type=%d",
      (this->get_parameter("use_sim_time").as_bool() ? "true" : "false"),
      static_cast<int>(this->get_clock()->get_clock_type()));

    // --- TF ---
    // Buffer uses the node clock; TransformListener spins its OWN internal thread
    // (spin_thread=true by default), so canTransform timeouts are serviced
    // independently of this node's executor. Do NOT pass `this` to the listener.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Callback groups ---
    // Heavy PCL work (VoxelGrid/RANSAC/SOR) lives in its own MutuallyExclusive
    // group so a long cloudCb cannot starve the publish timer or scanCb, which
    // share a second MutuallyExclusive group. Shared state is guarded by
    // data_mutex_; the lock is never held across PCL work, TF, or publish.
    cloud_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_scan_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- QoS ---
    // /scan : SensorDataQoS (BEST_EFFORT, depth 5) - tiny LaserScan, fine.
    // /cloud: KeepLast(1) BEST_EFFORT so the middleware drops all but the newest
    //         cloud (true drop-not-queue; bounds RAM and keeps frames fresh).
    // /out  : reliable QoS(5) so slam_toolbox / costmap (reliable) connect.
    auto scan_qos = rclcpp::SensorDataQoS();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto pub_qos = rclcpp::QoS(5);  // reliable by default

    rclcpp::SubscriptionOptions cloud_opts;
    cloud_opts.callback_group = cloud_group_;
    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = timer_scan_group_;

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&ScanFuser::cloudCb, this, std::placeholders::_1), cloud_opts);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos,
      std::bind(&ScanFuser::scanCb, this, std::placeholders::_1), scan_opts);

    fused_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(fused_topic_, pub_qos);

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_));
    // Node-clock timer (honors use_sim_time), in the timer+scan group.
    timer_ = this->create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ScanFuser::timerCb, this), timer_scan_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "scan_fuser up: target=%s cloud=%s scan=%s out=%s bins=%d pub=%.1fHz depth=%.1fHz",
      target_frame_.c_str(), cloud_topic_.c_str(), scan_topic_.c_str(),
      fused_topic_.c_str(), num_bins_, publish_rate_, depth_process_rate_);
  }

private:
  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  void declareParams()
  {
    this->declare_parameter<std::string>("target_frame", "base_footprint");
    this->declare_parameter<std::string>("cloud_topic", "/camera/depth/points");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("fused_topic", "/scan_fused");

    this->declare_parameter<int>("num_bins", 720);
    this->declare_parameter<double>("range_min", 0.12);
    this->declare_parameter<double>("range_max", 8.0);
    this->declare_parameter<double>("publish_rate", 12.0);

    this->declare_parameter<double>("depth_process_rate", 6.0);
    this->declare_parameter<int>("pixel_step", 2);
    this->declare_parameter<double>("voxel_leaf", 0.02);
    this->declare_parameter<int>("voxel_min_points", 3);
    this->declare_parameter<int>("sor_mean_k", 10);
    this->declare_parameter<double>("sor_stddev", 1.0);

    this->declare_parameter<bool>("ground_enable", true);
    this->declare_parameter<double>("ground_max_z", 0.10);
    this->declare_parameter<double>("ground_dist_thresh", 0.02);
    this->declare_parameter<double>("ground_normal_z_min", 0.95);
    this->declare_parameter<double>("ground_eps_angle_deg", 10.0);

    this->declare_parameter<double>("gate_near_range", 1.2);
    this->declare_parameter<double>("gate_min_height_near", 0.02);
    this->declare_parameter<double>("gate_min_height_far", 0.05);
    this->declare_parameter<double>("gate_max_height", 0.25);
    this->declare_parameter<double>("depth_range_min", 0.58);
    this->declare_parameter<double>("depth_range_max", 3.0);
    this->declare_parameter<double>("depth_angle_min", -0.5236);
    this->declare_parameter<double>("depth_angle_max", 0.5236);

    // dynamic_typing so an EMPTY yaml array ([]) — which rcl can only type as
    // NOT_SET — is accepted instead of crashing the node at declaration. Filled
    // arrays still arrive as DOUBLE_ARRAY (or INTEGER_ARRAY); handled in load.
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.dynamic_typing = true;
      this->declare_parameter(
        "lidar_ignore_sectors_deg", rclcpp::ParameterValue(std::vector<double>{}), desc);
    }

    this->declare_parameter<double>("depth_stale_timeout", 0.2);
  }

  void loadParams()
  {
    target_frame_ = this->get_parameter("target_frame").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    fused_topic_ = this->get_parameter("fused_topic").as_string();

    num_bins_ = this->get_parameter("num_bins").as_int();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_double();

    depth_process_rate_ = this->get_parameter("depth_process_rate").as_double();
    pixel_step_ = this->get_parameter("pixel_step").as_int();
    voxel_leaf_ = this->get_parameter("voxel_leaf").as_double();
    voxel_min_points_ = this->get_parameter("voxel_min_points").as_int();
    sor_mean_k_ = this->get_parameter("sor_mean_k").as_int();
    sor_stddev_ = this->get_parameter("sor_stddev").as_double();

    ground_enable_ = this->get_parameter("ground_enable").as_bool();
    ground_max_z_ = this->get_parameter("ground_max_z").as_double();
    ground_dist_thresh_ = this->get_parameter("ground_dist_thresh").as_double();
    ground_normal_z_min_ = this->get_parameter("ground_normal_z_min").as_double();
    ground_eps_angle_deg_ = this->get_parameter("ground_eps_angle_deg").as_double();

    gate_near_range_ = this->get_parameter("gate_near_range").as_double();
    gate_min_height_near_ = this->get_parameter("gate_min_height_near").as_double();
    gate_min_height_far_ = this->get_parameter("gate_min_height_far").as_double();
    gate_max_height_ = this->get_parameter("gate_max_height").as_double();
    depth_range_min_ = this->get_parameter("depth_range_min").as_double();
    depth_range_max_ = this->get_parameter("depth_range_max").as_double();
    depth_angle_min_ = this->get_parameter("depth_angle_min").as_double();
    depth_angle_max_ = this->get_parameter("depth_angle_max").as_double();

    // With dynamic_typing the value may be DOUBLE_ARRAY (filled), INTEGER_ARRAY
    // (user wrote ints), or NOT_SET (empty [] / absent). Resolve all three.
    {
      const auto p = this->get_parameter("lidar_ignore_sectors_deg");
      ignore_sectors_deg_.clear();
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        ignore_sectors_deg_ = p.as_double_array();
      } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        for (const long v : p.as_integer_array()) {
          ignore_sectors_deg_.push_back(static_cast<double>(v));
        }
        RCLCPP_WARN(
          this->get_logger(),
          "lidar_ignore_sectors_deg given as integers; prefer floats (e.g. 170.0)");
      }
      // else: NOT_SET (empty/absent) -> no sectors.
    }
    depth_stale_timeout_ = this->get_parameter("depth_stale_timeout").as_double();

    if (num_bins_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "num_bins<=0; forcing 720");
      num_bins_ = 720;
    }
    if (pixel_step_ < 1) {
      pixel_step_ = 1;
    }
  }

  // Parse the flat [min,max,min,max,...] degree pairs into radian sectors in
  // base_footprint, normalized to [-PI, PI). Sectors may wrap across +-PI.
  // A pair whose RAW span (hi-lo, pre-normalization) is >= 360 deg is treated
  // as a full-circle mask. A pair that normalizes to lo==hi is warned (near no-op
  // unless it is the full-circle case above).
  void parseIgnoreSectors()
  {
    ignore_sectors_rad_.clear();
    mask_all_ = false;
    if (ignore_sectors_deg_.size() % 2 != 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "lidar_ignore_sectors_deg has odd length (%zu); dropping last value",
        ignore_sectors_deg_.size());
    }
    const size_t n = ignore_sectors_deg_.size() / 2;
    for (size_t k = 0; k < n; ++k) {
      const double lo_deg = ignore_sectors_deg_[2 * k];
      const double hi_deg = ignore_sectors_deg_[2 * k + 1];
      if (std::abs(hi_deg - lo_deg) >= 360.0) {
        mask_all_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "ignore sector [%.1f,%.1f] deg spans >=360 -> full-circle mask",
          lo_deg, hi_deg);
        continue;
      }
      const double lo = normAngle(lo_deg * kPi / 180.0);
      const double hi = normAngle(hi_deg * kPi / 180.0);
      if (lo == hi) {
        RCLCPP_WARN(
          this->get_logger(),
          "ignore sector [%.1f,%.1f] deg normalizes to a single angle "
          "(near no-op); check your mask spec",
          lo_deg, hi_deg);
      }
      ignore_sectors_rad_.emplace_back(lo, hi);
      RCLCPP_INFO(
        this->get_logger(),
        "ignore sector parsed: [%.1f,%.1f] deg -> [%.3f,%.3f] rad%s",
        lo_deg, hi_deg, lo, hi, (lo > hi ? " (wrapped across +-PI)" : ""));
    }
    if (ignore_sectors_rad_.empty() && !mask_all_) {
      RCLCPP_INFO(this->get_logger(), "no lidar ignore sectors configured");
    }
  }

  // True if angle th (rad, [-PI,PI)) falls inside any ignore sector. A sector
  // [lo,hi] that wraps (lo>hi) is the union of [lo,PI) and [-PI,hi].
  bool inIgnoreSector(double th) const
  {
    if (mask_all_) {
      return true;
    }
    for (const auto & s : ignore_sectors_rad_) {
      const double lo = s.first;
      const double hi = s.second;
      if (lo <= hi) {
        if (th >= lo && th <= hi) {
          return true;
        }
      } else {
        if (th >= lo || th <= hi) {  // wrapped sector
          return true;
        }
      }
    }
    return false;
  }

  // Map an angle in [-PI,PI) to a bin index in [0, num_bins).
  inline int angleToBin(double th) const
  {
    int i = static_cast<int>(std::lround((th - angle_min_) / angle_increment_));
    i %= num_bins_;          // handles i==num_bins from rounding and any drift
    if (i < 0) {
      i += num_bins_;
    }
    return i;
  }

  // ---------------------------------------------------------------------------
  // DEPTH path
  // ---------------------------------------------------------------------------
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // Throttle by node time vs last processed; DROP frames (no buffering).
    // last_proc_time_ is touched only in the serialized cloud group -> no lock.
    const rclcpp::Time now = this->now();
    const double min_period = 1.0 / std::max(1e-3, depth_process_rate_);
    if (last_proc_valid_) {
      if ((now.seconds() - last_proc_time_.seconds()) < min_period) {
        return;  // drop this frame
      }
    }
    last_proc_time_ = now;
    last_proc_valid_ = true;

    try {
      const auto t0 = std::chrono::steady_clock::now();
      processCloud(msg);
      const auto t1 = std::chrono::steady_clock::now();
      const double secs = std::chrono::duration<double>(t1 - t0).count();
      if (secs > min_period) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "processCloud took %.1f ms > depth period %.1f ms; depth may go stale "
          "and fusion will degrade to LiDAR-only",
          secs * 1e3, min_period * 1e3);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "cloudCb exception: %s", e.what());
    } catch (...) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "cloudCb unknown exception");
    }
  }

  void processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // TF guard: height reasoning is in base_footprint (z=0=floor), so transform
    // the cloud to target_frame BEFORE ground removal/gating. Look up at the
    // SENSOR stamp (the sensor stamp is used ONLY here, never for staleness).
    geometry_msgs::msg::TransformStamped tf;
    const rclcpp::Time sensor_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
    // Prefer the sensor-stamped TF for time-correctness; fall back to the LATEST
    // TF if the stamped one isn't ready within the wait window. target_frame_<-camera
    // is a FIXED mount (static TF) so the latest transform is geometrically identical.
    // Under full-stack CPU load the driver's camera TF can lag past 50ms, which
    // otherwise drops EVERY depth cloud and silently collapses to LiDAR-only fusion.
    bool have_tf = false;
    try {
      // 50ms wait is safe: cloudCb runs in its own group on its own executor
      // thread (executor sized >=2), so this cannot starve the LiDAR/publish group.
      if (tf_buffer_->canTransform(
          target_frame_, msg->header.frame_id, sensor_stamp,
          rclcpp::Duration::from_seconds(0.05)))
      {
        tf = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, sensor_stamp);
        have_tf = true;
      }
    } catch (const tf2::TransformException &) {
      // fall through to the latest-time lookup below
    }
    if (!have_tf) {
      try {
        tf = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF %s<-%s unavailable (stamp+latest); dropping cloud: %s",
          target_frame_.c_str(), msg->header.frame_id.c_str(), e.what());
        return;
      }
    }

    // 1. Structured subsample straight off the raw byte buffer (no full convert).
    // Pipeline: subsample -> transform to base_footprint -> VoxelGrid -> RANSAC
    // ground -> SOR -> gate/bin. Transform before VoxelGrid so the voxel grid and
    // all gravity-aligned (z=0=floor) reasoning happen in base_footprint.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    subsampleToPcl(*msg, *cloud_in);
    if (cloud_in->empty()) {
      return;
    }

    // Transform to target_frame using the looked-up TF; output is all-finite.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    transformCloud(*cloud_in, *cloud, tf);
    if (cloud->empty()) {
      return;
    }

    // 2. VoxelGrid downsample.
    if (voxel_leaf_ > 0.0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr vox(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(
        static_cast<float>(voxel_leaf_),
        static_cast<float>(voxel_leaf_),
        static_cast<float>(voxel_leaf_));
      vg.setMinimumPointsNumberPerVoxel(static_cast<unsigned int>(std::max(1, voxel_min_points_)));
      vg.filter(*vox);
      cloud.swap(vox);
    }
    if (cloud->empty()) {
      return;
    }

    // 3. RANSAC ground-plane removal (in base_footprint).
    if (ground_enable_) {
      removeGround(cloud);
      if (cloud->empty()) {
        return;
      }
    }

    // 4. StatisticalOutlierRemoval.
    if (sor_mean_k_ > 0 && cloud->size() > static_cast<size_t>(sor_mean_k_)) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr sorc(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_);
      sor.filter(*sorc);
      cloud.swap(sorc);
    }

    // 5 & 6. Two-tier height gate + bin into a local depth-bin buffer.
    std::vector<float> local_bins(num_bins_, std::numeric_limits<float>::infinity());
    for (const auto & p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const double r = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
      if (r < depth_range_min_ || r > depth_range_max_) {
        continue;
      }
      const double th = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
      if (th < depth_angle_min_ || th > depth_angle_max_) {
        continue;
      }
      const double minh = (r <= gate_near_range_) ? gate_min_height_near_ : gate_min_height_far_;
      if (p.z < minh || p.z > gate_max_height_) {
        continue;
      }
      // th is already in [-PI,PI] (atan2) and inside the forward cone; bin directly.
      const int bin = angleToBin(th);
      const float rf = static_cast<float>(r);
      if (rf < local_bins[bin]) {
        local_bins[bin] = rf;
      }
    }

    // Commit to shared state under lock. Staleness clock = NODE clock receive
    // time, independent of the sensor stamp (robust to camera/node skew).
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      depth_bins_.swap(local_bins);
      last_depth_recv_ = this->now();
      has_depth_ = true;
    }
  }

  // Structured subsample (u,v stride) straight off the raw PointCloud2 buffer
  // via PointCloud2ConstIterator. The full cloud is NEVER deserialized; only
  // every pixel_step-th column AND row is read, cutting deserialize+alloc by
  // ~pixel_step^2. Falls back to whole-cloud iteration for unorganized clouds.
  void subsampleToPcl(
    const sensor_msgs::msg::PointCloud2 & msg,
    pcl::PointCloud<pcl::PointXYZ> & out) const
  {
    out.clear();
    out.is_dense = false;  // we drop non-finite points below

    const uint32_t W = msg.width;
    const uint32_t H = msg.height;
    if (W == 0 || (H == 0)) {
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

    const uint32_t step = static_cast<uint32_t>(std::max(1, pixel_step_));

    if (H <= 1 || step <= 1) {
      // Unorganized cloud (or no striding): iterate all points sequentially.
      const size_t total = static_cast<size_t>(W) * static_cast<size_t>(std::max<uint32_t>(H, 1));
      out.reserve(total);
      for (size_t i = 0; i < total; ++i, ++it_x, ++it_y, ++it_z) {
        const float x = *it_x, y = *it_y, z = *it_z;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
          out.emplace_back(x, y, z);
        }
      }
      return;
    }

    // Organized cloud: index arithmetic to skip rows and columns by `step`.
    // PointCloud2ConstIterator advances element-by-element, so re-seat per row.
    out.reserve((static_cast<size_t>(W) / step + 1) * (static_cast<size_t>(H) / step + 1));
    for (uint32_t v = 0; v < H; v += step) {
      sensor_msgs::PointCloud2ConstIterator<float> rx(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> ry(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> rz(msg, "z");
      const size_t row_off = static_cast<size_t>(v) * static_cast<size_t>(W);
      rx += row_off; ry += row_off; rz += row_off;
      for (uint32_t u = 0; u < W; u += step) {
        const float x = rx[0], y = ry[0], z = rz[0];
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
          out.emplace_back(x, y, z);
        }
        rx += step; ry += step; rz += step;
      }
    }
  }

  // Apply a geometry_msgs TransformStamped to every point of a PCL cloud,
  // dropping any non-finite input. Output is guaranteed finite.
  void transformCloud(
    const pcl::PointCloud<pcl::PointXYZ> & in,
    pcl::PointCloud<pcl::PointXYZ> & out,
    const geometry_msgs::msg::TransformStamped & tf) const
  {
    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - z * w);
    const double r02 = 2.0 * (x * z + y * w);
    const double r10 = 2.0 * (x * y + z * w);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    const double r12 = 2.0 * (y * z - x * w);
    const double r20 = 2.0 * (x * z - y * w);
    const double r21 = 2.0 * (y * z + x * w);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    out.clear();
    out.reserve(in.size());
    for (const auto & p : in.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      pcl::PointXYZ q2;
      q2.x = static_cast<float>(r00 * p.x + r01 * p.y + r02 * p.z + t.x);
      q2.y = static_cast<float>(r10 * p.x + r11 * p.y + r12 * p.z + t.y);
      q2.z = static_cast<float>(r20 * p.x + r21 * p.y + r22 * p.z + t.z);
      out.push_back(q2);
    }
    // All survivors are finite; let downstream filters skip NaN re-checks.
    out.is_dense = true;
  }

  // RANSAC perpendicular-plane ground removal, restricted to points below
  // ground_max_z. The real wall protection is setEpsAngle (only planes whose
  // normal is within eps of +Z are ever fit); the post-hoc |nz| check is a
  // degenerate/near-vertical-refit backstop. Inliers are dropped only when
  // |nz| > ground_normal_z_min; otherwise the frame is gate-only (keeps all).
  void removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
  {
    pcl::PointIndices::Ptr candidates(new pcl::PointIndices);
    candidates->indices.reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (cloud->points[i].z < ground_max_z_) {
        candidates->indices.push_back(static_cast<int>(i));
      }
    }
    if (candidates->indices.size() < 10) {
      return;  // too few floor candidates; nothing to remove
    }

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(ground_eps_angle_deg_ * kPi / 180.0);
    seg.setDistanceThreshold(ground_dist_thresh_);
    seg.setMaxIterations(50);       // Pi5 budget; perpendicular model is cheap
    seg.setProbability(0.95);       // allow early exit
    seg.setInputCloud(cloud);
    seg.setIndices(candidates);     // fit only to near-floor band
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty() || coeff->values.size() < 4) {
      return;  // no plane found; keep all (gate-only)
    }
    const double nz = std::abs(coeff->values[2]);
    if (nz <= ground_normal_z_min_) {
      return;  // degenerate near-vertical refit; keep all this frame
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);    // inliers index into `cloud` (original)
    extract.setNegative(true);      // drop the ground inliers
    extract.filter(*out);
    cloud.swap(out);
  }

  // ---------------------------------------------------------------------------
  // LiDAR path
  // ---------------------------------------------------------------------------
  void scanCb(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    try {
      // Cache the latest scan only; resample/mask happen in the timer.
      std::lock_guard<std::mutex> lk(data_mutex_);
      last_scan_ = msg;
      has_scan_ = true;
    } catch (...) {
      // lock_guard cannot realistically throw; guard anyway per "no escape".
    }
  }

  // Resample a cached LiDAR scan into base_footprint bins (min range), applying
  // the self-occlusion ignore sectors. rplidar_link has yaw=PI vs base_footprint
  // (xy offset ~1.7cm ignored), so th_bf = ray_angle + PI (normalized).
  void scanToBins(
    const sensor_msgs::msg::LaserScan & scan,
    std::vector<float> & bins) const
  {
    bins.assign(num_bins_, std::numeric_limits<float>::infinity());
    const size_t n = scan.ranges.size();
    for (size_t j = 0; j < n; ++j) {
      const float r = scan.ranges[j];
      if (!std::isfinite(r) || r == 0.0f) {  // rplidar emits exactly 0.0 = no return
        continue;
      }
      if (r < scan.range_min || r > scan.range_max) {  // also rejects negatives
        continue;
      }
      const double ray = scan.angle_min + static_cast<double>(j) * scan.angle_increment;
      const double th_bf = normAngle(ray + kPi);  // +PI yaw offset to base_footprint
      if (inIgnoreSector(th_bf)) {
        continue;  // robot self-return / body-blocked sector
      }
      const int bin = angleToBin(th_bf);
      if (r < bins[bin]) {
        bins[bin] = r;
      }
    }
  }

  // ---------------------------------------------------------------------------
  // FUSE + publish
  // ---------------------------------------------------------------------------
  void timerCb()
  {
    try {
      timerCbImpl();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "timerCb exception: %s", e.what());
    } catch (...) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "timerCb unknown exception");
    }
  }

  void timerCbImpl()
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    std::vector<float> depth_copy;
    rclcpp::Time depth_recv;
    bool depth_ok = false;

    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      if (!has_scan_ || !last_scan_) {
        return;  // never publish without a LiDAR backbone
      }
      scan = last_scan_;
      if (has_depth_) {
        depth_copy = depth_bins_;
        depth_recv = last_depth_recv_;
        depth_ok = true;
      }
    }

    // Suppress duplicate/non-increasing stamps: only publish when the cached
    // LiDAR stamp has advanced. Prevents slam_toolbox from de-duping/double-
    // counting when publish_rate (12 Hz) > LiDAR rate (~10 Hz).
    const rclcpp::Time scan_stamp(scan->header.stamp, this->get_clock()->get_clock_type());
    if (have_pub_ && scan_stamp <= last_pub_stamp_) {
      return;
    }

    // Depth freshness: same-clock (node clock) age, robust to camera latency.
    bool depth_fresh = false;
    if (depth_ok) {
      const double age = this->now().seconds() - depth_recv.seconds();
      depth_fresh = (age >= 0.0) && (age <= depth_stale_timeout_);
    }

    // LiDAR -> bins (in base_footprint).
    std::vector<float> lidar_bins;
    scanToBins(*scan, lidar_bins);

    // Build the fused scan.
    sensor_msgs::msg::LaserScan out;
    out.header.frame_id = target_frame_;
    out.header.stamp = scan->header.stamp;  // spatial backbone = LiDAR stamp
    out.angle_min = static_cast<float>(angle_min_);
    out.angle_increment = static_cast<float>(angle_increment_);
    out.angle_max = static_cast<float>(angle_max_);
    out.time_increment = 0.0f;            // instantaneous resample
    out.scan_time = scan->scan_time;      // advisory: source scan period
    out.range_min = static_cast<float>(range_min_);
    out.range_max = static_cast<float>(range_max_);
    out.ranges.assign(num_bins_, std::numeric_limits<float>::infinity());
    // intensities intentionally left empty

    const float fmin = static_cast<float>(range_min_);
    const float fmax = static_cast<float>(range_max_);
    for (int i = 0; i < num_bins_; ++i) {
      float best = std::numeric_limits<float>::infinity();
      const float lr = lidar_bins[i];
      if (std::isfinite(lr)) {
        best = lr;
      }
      if (depth_fresh) {
        const float dr = depth_copy[i];
        if (std::isfinite(dr) && dr < best) {
          best = dr;
        }
      }
      if (std::isfinite(best)) {
        if (best < fmin) {
          best = fmin;            // snap too-close returns up (safety: keep them)
        } else if (best > fmax) {
          best = std::numeric_limits<float>::infinity();  // beyond max = no return
        }
      }
      out.ranges[i] = best;       // empty bins stay +inf
    }

    fused_pub_->publish(out);
    last_pub_stamp_ = scan_stamp;
    have_pub_ = true;
  }

  // ---------------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------------
  // params
  std::string target_frame_, cloud_topic_, scan_topic_, fused_topic_;
  int num_bins_{720};
  double range_min_{0.12}, range_max_{8.0}, publish_rate_{12.0};
  double depth_process_rate_{6.0};
  int pixel_step_{2};
  double voxel_leaf_{0.02};
  int voxel_min_points_{3};
  int sor_mean_k_{10};
  double sor_stddev_{1.0};
  bool ground_enable_{true};
  double ground_max_z_{0.10}, ground_dist_thresh_{0.02};
  double ground_normal_z_min_{0.95}, ground_eps_angle_deg_{10.0};
  double gate_near_range_{1.2}, gate_min_height_near_{0.02};
  double gate_min_height_far_{0.05}, gate_max_height_{0.25};
  double depth_range_min_{0.58}, depth_range_max_{3.0};
  double depth_angle_min_{-0.5236}, depth_angle_max_{0.5236};
  std::vector<double> ignore_sectors_deg_;
  double depth_stale_timeout_{0.2};

  // derived grid
  double angle_min_{-kPi}, angle_increment_{0.0}, angle_max_{0.0};
  std::vector<std::pair<double, double>> ignore_sectors_rad_;
  bool mask_all_{false};

  // shared state (guarded by data_mutex_)
  std::mutex data_mutex_;
  std::vector<float> depth_bins_;
  rclcpp::Time last_depth_recv_{0, 0, RCL_ROS_TIME};  // node-clock receive time
  bool has_depth_{false};
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_scan_;
  bool has_scan_{false};

  // cloud throttle (touched only in the serialized cloud group)
  rclcpp::Time last_proc_time_{0, 0, RCL_ROS_TIME};
  bool last_proc_valid_{false};

  // publish stamp gating (touched only in the timer group)
  rclcpp::Time last_pub_stamp_{0, 0, RCL_ROS_TIME};
  bool have_pub_{false};

  // ROS entities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  rclcpp::CallbackGroup::SharedPtr timer_scan_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanFuser>();
  // MultiThreadedExecutor sized to >= 2 threads so the heavy cloud group runs
  // concurrently with the timer+scan group even on a low-core Pi5 (a default
  // executor can under-report cores and collapse the groups back to serial,
  // silently negating the starvation-avoidance design). Per-callback safety is
  // provided by data_mutex_.
  const unsigned int hw = std::thread::hardware_concurrency();
  const size_t threads = std::max<unsigned int>(2u, hw);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), threads);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
