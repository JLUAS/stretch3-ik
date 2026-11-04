#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "stretch_collision_planning/collision_geometry.hpp"
#include "stretch_collision_planning/octomap_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"

namespace stretch_collision_planning {
namespace {
struct Point3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    const auto hx = std::hash<std::int64_t>{}(key.x);
    const auto hy = std::hash<std::int64_t>{}(key.y);
    const auto hz = std::hash<std::int64_t>{}(key.z);
    return hx ^ (hy << 1) ^ (hz << 7);
  }
};

struct SelfFilterBox {
  std::string frame;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d map_to_box = Eigen::Isometry3d::Identity();
  bool transform_valid = false;
};

Eigen::Isometry3d transformToEigen(
    const geometry_msgs::msg::TransformStamped& transform) {
  const auto& t = transform.transform.translation;
  const auto& r = transform.transform.rotation;
  Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
  if (q.norm() == 0.0) q = Eigen::Quaterniond::Identity();
  q.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.toRotationMatrix();
  result.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  return result;
}

bool validProbability(double p) { return std::isfinite(p) && p > 0.0 && p < 1.0; }

double probabilityParam(rclcpp::Node& node, const std::string& name,
                        double default_value) {
  const double value = node.declare_parameter<double>(name, default_value);
  return validProbability(value) ? value : default_value;
}
}  // namespace

class OctomapBuilderNode : public rclcpp::Node {
 public:
  OctomapBuilderNode()
      : Node("octomap_builder"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    input_topic_ = declare_parameter<std::string>(
        "input_cloud_topic", "/camera/depth/color/points");
    filtered_topic_ = declare_parameter<std::string>(
        "filtered_cloud_topic", "/perception/filtered_points");
    octomap_topic_ =
        declare_parameter<std::string>("octomap_topic", "/perception/octomap_binary");
    marker_topic_ = declare_parameter<std::string>(
        "marker_topic", "/perception/octomap_markers");
    status_topic_ = declare_parameter<std::string>(
        "scene_status_topic", "/perception/scene_status");
    map_frame_ = declare_parameter<std::string>("map_frame", "base_link");

    resolution_ = declare_parameter<double>("resolution", 0.04);
    if (!std::isfinite(resolution_) || resolution_ <= 0.0) resolution_ = 0.04;
    min_range_ = declare_parameter<double>("min_range", 0.15);
    max_sensor_range_ = declare_parameter<double>("max_sensor_range", 2.5);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.02);
    freshness_timeout_ = declare_parameter<double>("freshness_timeout", 1.0);
    reset_map_each_cloud_ = declare_parameter<bool>("reset_map_each_cloud", false);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    // A persistent map grows; serializing it after every cloud is wasted CPU.
    publish_period_ = declare_parameter<double>("publish_period", 0.0);
    max_marker_voxels_ = declare_parameter<int>("max_marker_voxels", 30000);

    occupancy_threshold_ =
        probabilityParam(*this, "occupancy_threshold", 0.50);
    prob_hit_ = probabilityParam(*this, "prob_hit", 0.70);
    prob_miss_ = probabilityParam(*this, "prob_miss", 0.40);
    clamping_min_ = probabilityParam(*this, "clamping_min", 0.12);
    clamping_max_ = probabilityParam(*this, "clamping_max", 0.97);

    floor_filter_enabled_ =
        declare_parameter<bool>("floor_filter.enabled", false);
    floor_filter_min_z_ =
        declare_parameter<double>("floor_filter.min_z", -0.05);
    floor_filter_max_z_ =
        declare_parameter<double>("floor_filter.max_z", 0.04);

    self_filter_enabled_ =
        declare_parameter<bool>("self_filter.enabled", true);
    clear_self_filter_volumes_ = declare_parameter<bool>(
        "self_filter.clear_robot_volume", true);
    const auto self_frames = declare_parameter<std::vector<std::string>>(
        "self_filter.frames",
        {"base_link", "link_mast", "camera_link", "gripper_camera_link"});
    const auto self_boxes = declare_parameter<std::vector<double>>(
        "self_filter.boxes",
        {0.0, 0.0, 0.10, 0.55, 0.45, 0.25,
         0.0, 0.0, 0.35, 0.16, 0.16, 0.70,
         0.0, -0.0175, 0.0, 0.08, 0.12, 0.06,
         -0.0078, -0.009, 0.0, 0.07, 0.08, 0.08});
    configureSelfFilter(self_frames, self_boxes);
    // The arm telescopes, so fixed link boxes cannot cover it; the planner's
    // own capsules (inflated) follow the arm at any extension.
    self_filter_capsules_ =
        declare_parameter<bool>("self_filter.capsules", true);
    capsule_margin_ =
        declare_parameter<double>("self_filter.capsule_margin", 0.03);
    gripper_envelope_radius_ =
        declare_parameter<double>("self_filter.gripper_envelope_radius", 0.07);
    target_box_sub_ = create_subscription<visualization_msgs::msg::Marker>(
        "~/target_box", rclcpp::QoS(1).reliable().transient_local(),
        [this](visualization_msgs::msg::Marker::ConstSharedPtr m) {
          // The grasp target is a planned contact: its points are dropped and
          // its voxels cleared. Any frame works (odom on the shelf,
          // link_grasp_center while carried). Marker orientation is ignored.
          if (m->action != visualization_msgs::msg::Marker::ADD ||
              m->header.frame_id.empty()) {
            target_box_.reset();
            RCLCPP_INFO(get_logger(), "Target box cleared");
            return;
          }
          SelfFilterBox box;
          box.frame = m->header.frame_id;
          box.center = Eigen::Vector3d(m->pose.position.x, m->pose.position.y,
                                       m->pose.position.z);
          box.size = Eigen::Vector3d(m->scale.x, m->scale.y, m->scale.z);
          if (!box.center.allFinite() || !box.size.allFinite() ||
              (box.size.array() <= 0.0).any() || (box.size.array() > 1.0).any()) {
            RCLCPP_WARN(get_logger(), "Rejected invalid target box");
            return;
          }
          target_box_ = box;
          RCLCPP_INFO(get_logger(), "Target box set in %s", box.frame.c_str());
        });

    resetTree();

    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        filtered_topic_, rclcpp::SensorDataQoS());
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
        octomap_topic_, rclcpp::QoS(1).reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(1).reliable().transient_local());

    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reset",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          capture_pending_ = false;
          frozen_ = false;
          resetTree();
          last_map_update_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          const auto stamp = now();
          const builtin_interfaces::msg::Time stamp_msg = stamp;
          publishOctomap(stamp_msg);
          publishMarkers(stamp_msg);
          publishStatus();
          response->success = true;
          response->message = "OctoMap cleared; waiting for a fresh point cloud";
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        });

    capture_service_ = create_service<std_srvs::srv::Trigger>(
        "~/capture_once",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          resetTree();
          last_map_update_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          frozen_ = false;
          capture_pending_ = true;
          const builtin_interfaces::msg::Time stamp = now();
          publishOctomap(stamp);
          publishMarkers(stamp);
          publishStatus();
          response->success = true;
          response->message =
              "OctoMap cleared; the next valid cloud will be captured and frozen";
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        });

    // The Stretch driver resets odometry to zero on every switch to trajectory
    // mode; voxels stored in odom would then sit in the wrong place around the
    // robot. Clear the map on such a jump; the fixed head refills it quickly.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const Eigen::Vector2d p(m->pose.pose.position.x, m->pose.pose.position.y);
          const double t = rclcpp::Time(m->header.stamp).seconds();
          if (have_odom_ && map_frame_ == m->header.frame_id &&
              (p - last_odom_).norm() > 0.05 + 1.0 * std::abs(t - last_odom_time_)) {
            RCLCPP_WARN(get_logger(), "Odometry jumped %.3f m (driver reset): clearing the OctoMap",
                        (p - last_odom_).norm());
            resetTree();
          }
          last_odom_ = p;
          last_odom_time_ = t;
          have_odom_ = true;
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          handleCloud(std::move(msg));
        });
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(),
                "OctoMap builder listening to %s, publishing %s and %s in %s",
                input_topic_.c_str(), filtered_topic_.c_str(),
                octomap_topic_.c_str(), map_frame_.c_str());
  }

 private:
  void resetTree() {
    tree_ = std::make_unique<octomap::OcTree>(resolution_);
    tree_->setOccupancyThres(occupancy_threshold_);
    tree_->setProbHit(prob_hit_);
    tree_->setProbMiss(prob_miss_);
    tree_->setClampingThresMin(clamping_min_);
    tree_->setClampingThresMax(clamping_max_);
  }

  void configureSelfFilter(const std::vector<std::string>& frames,
                           const std::vector<double>& boxes) {
    self_filter_boxes_.clear();
    if (boxes.size() != frames.size() * 6) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring self-filter boxes: expected 6 values per frame");
      return;
    }
    for (std::size_t i = 0; i < frames.size(); ++i) {
      SelfFilterBox box;
      box.frame = frames[i];
      box.center =
          Eigen::Vector3d(boxes[6 * i], boxes[6 * i + 1], boxes[6 * i + 2]);
      box.size = Eigen::Vector3d(boxes[6 * i + 3], boxes[6 * i + 4],
                                 boxes[6 * i + 5]);
      if (!box.frame.empty() && box.size.allFinite() &&
          (box.size.array() > 0.0).all()) {
        self_filter_boxes_.push_back(box);
      }
    }
  }

  bool refreshSelfFilterTransforms(std::vector<SelfFilterBox>& boxes) {
    bool all_ok = true;
    for (auto& box : boxes) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
            box.frame, map_frame_, tf2::TimePointZero,
            tf2::durationFromSec(0.05));
        box.map_to_box = transformToEigen(transform);
        box.transform_valid = true;
      } catch (const tf2::TransformException& e) {
        box.transform_valid = false;
        all_ok = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Self-filter TF unavailable %s <- %s: %s",
                             box.frame.c_str(), map_frame_.c_str(), e.what());
      }
    }
    return all_ok;
  }

  // Capsules at the cloud's capture time: with a fast arm the latest TF is
  // centimeters away from where the camera saw it.
  std::vector<CollisionCapsule> robotCapsules(const rclcpp::Time& stamp) {
    if (!self_filter_enabled_ || !self_filter_capsules_) return {};
    static const auto model =
        PrimitiveRobotCollisionModel::makeConservativeStretchArm();
    // Filter-only envelope: the SG3 gripper with its D405 and open fingers is
    // far wider than the planner's thin gripper capsule.
    static const PrimitiveRobotCollisionModel envelope(
        {{"wrist_gripper_envelope", "link_wrist_yaw", "link_grasp_center",
          gripper_envelope_radius_}});
    static const std::vector<std::string> links = {
        "link_lift",      "link_arm_l4",     "link_arm_l3",
        "link_arm_l2",    "link_arm_l1",     "link_arm_l0",
        "link_wrist_yaw", "link_wrist_pitch", "link_wrist_roll",
        "link_gripper_s3_body", "link_grasp_center"};
    LinkTransforms transforms;
    try {
      for (const auto& link : links) {
        geometry_msgs::msg::TransformStamped t;
        try {
          t = tf_buffer_.lookupTransform(map_frame_, link, stamp,
                                         tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException&) {
          t = tf_buffer_.lookupTransform(map_frame_, link, tf2::TimePointZero);
        }
        transforms[link] = transformToEigen(t);
      }
      auto capsules = model.capsules(transforms);
      for (const auto& c : envelope.capsules(transforms)) capsules.push_back(c);
      for (auto& c : capsules) c.radius += capsule_margin_;
      return capsules;
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Capsule self-filter TF unavailable: %s", e.what());
      return {};
    }
  }

  static bool pointInsideCapsules(const Eigen::Vector3d& point,
                                  const std::vector<CollisionCapsule>& capsules) {
    for (const auto& c : capsules)
      if (segmentSegmentDistanceSquared(point, point, c.a, c.b) <=
          c.radius * c.radius)
        return true;
    return false;
  }

  void clearCapsuleVolumes(const std::vector<CollisionCapsule>& capsules) {
    if (!clear_self_filter_volumes_) return;
    for (const auto& c : capsules) {
      const Eigen::Vector3d lo =
          c.a.cwiseMin(c.b) - Eigen::Vector3d::Constant(c.radius);
      const Eigen::Vector3d hi =
          c.a.cwiseMax(c.b) + Eigen::Vector3d::Constant(c.radius);
      auto first = [this](double v) {
        return std::floor(v / resolution_) * resolution_ + 0.5 * resolution_;
      };
      for (double x = first(lo.x()); x <= hi.x() + 1e-12; x += resolution_)
        for (double y = first(lo.y()); y <= hi.y() + 1e-12; y += resolution_)
          for (double z = first(lo.z()); z <= hi.z() + 1e-12; z += resolution_) {
            const Eigen::Vector3d p(x, y, z);
            if (segmentSegmentDistanceSquared(p, p, c.a, c.b) <=
                c.radius * c.radius)
              tree_->updateNode(octomap::point3d(x, y, z), false, true);
          }
    }
    tree_->updateInnerOccupancy();
  }

  bool pointInsideSelfFilter(const Eigen::Vector3d& point,
                             const std::vector<SelfFilterBox>& boxes) const {
    if (!self_filter_enabled_) return false;
    for (const auto& box : boxes) {
      if (!box.transform_valid) continue;
      const Eigen::Vector3d local = box.map_to_box * point;
      const Eigen::Array3d delta =
          (local - box.center).array().abs() - (0.5 * box.size).array();
      if ((delta <= 0.0).all()) return true;
    }
    return false;
  }

  void clearSelfFilterVolumes(const std::vector<SelfFilterBox>& boxes) {
    if (!self_filter_enabled_ || !clear_self_filter_volumes_) return;
    for (const auto& box : boxes) {
      if (!box.transform_valid) continue;
      const Eigen::Isometry3d box_to_map = box.map_to_box.inverse();
      const Eigen::Vector3d half = 0.5 * box.size;
      Eigen::Vector3d map_min = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::infinity());
      Eigen::Vector3d map_max = Eigen::Vector3d::Constant(
          -std::numeric_limits<double>::infinity());
      for (double x : {-half.x(), half.x()}) {
        for (double y : {-half.y(), half.y()}) {
          for (double z : {-half.z(), half.z()}) {
            const Eigen::Vector3d corner =
                box_to_map * (box.center + Eigen::Vector3d(x, y, z));
            map_min = map_min.cwiseMin(corner);
            map_max = map_max.cwiseMax(corner);
          }
        }
      }
      auto first_center = [this](double value) {
        return std::floor(value / resolution_) * resolution_ +
               0.5 * resolution_;
      };
      for (double x = first_center(map_min.x());
           x <= map_max.x() + 1e-12; x += resolution_) {
        for (double y = first_center(map_min.y());
             y <= map_max.y() + 1e-12; y += resolution_) {
          for (double z = first_center(map_min.z());
               z <= map_max.z() + 1e-12; z += resolution_) {
            const Eigen::Vector3d map_point(x, y, z);
            const Eigen::Vector3d local = box.map_to_box * map_point;
            if (((local - box.center).array().abs() <= half.array()).all()) {
              tree_->updateNode(octomap::point3d(x, y, z), false, true);
            }
          }
        }
      }
    }
    tree_->updateInnerOccupancy();
  }

  bool floorFiltered(const Eigen::Vector3d& point) const {
    return floor_filter_enabled_ && point.z() >= floor_filter_min_z_ &&
           point.z() <= floor_filter_max_z_;
  }

  std::vector<Point3> filterCloud(
      const sensor_msgs::msg::PointCloud2& cloud,
      const Eigen::Vector3d& sensor_origin,
      const std::vector<SelfFilterBox>& self_boxes,
      const std::vector<CollisionCapsule>& capsules) const {
    std::vector<Point3> points;
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    if (voxel_leaf_size_ > 0.0) {
      seen.reserve(points.capacity());
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
      if (!point.allFinite()) continue;
      const double range = (point - sensor_origin).norm();
      if (!std::isfinite(range) || range < min_range_ ||
          range > max_sensor_range_) {
        continue;
      }
      if (floorFiltered(point)) continue;
      if (pointInsideSelfFilter(point, self_boxes)) continue;
      if (pointInsideCapsules(point, capsules)) continue;

      if (voxel_leaf_size_ > 0.0) {
        const VoxelKey key{
            static_cast<std::int64_t>(std::floor(point.x() / voxel_leaf_size_)),
            static_cast<std::int64_t>(std::floor(point.y() / voxel_leaf_size_)),
            static_cast<std::int64_t>(std::floor(point.z() / voxel_leaf_size_))};
        if (!seen.insert(key).second) continue;
      }
      points.push_back(
          {static_cast<float>(point.x()), static_cast<float>(point.y()),
           static_cast<float>(point.z())});
    }
    return points;
  }

  void publishFilteredCloud(const std::vector<Point3>& points,
                            const builtin_interfaces::msg::Time& stamp) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = map_frame_;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto& point : points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    filtered_pub_->publish(cloud);
  }

  void updateMap(const std::vector<Point3>& points,
                 const Eigen::Vector3d& sensor_origin) {
    if (reset_map_each_cloud_) resetTree();
    octomap::Pointcloud scan;
    scan.reserve(points.size());
    for (const auto& point : points) scan.push_back(point.x, point.y, point.z);
    tree_->insertPointCloud(scan,
                            octomap::point3d(sensor_origin.x(), sensor_origin.y(),
                                             sensor_origin.z()),
                            max_sensor_range_, true, true);
    tree_->updateInnerOccupancy();
    last_map_update_ = now();
  }

  void publishOctomap(const builtin_interfaces::msg::Time& stamp) {
    octomap_msgs::msg::Octomap msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Failed to serialize binary OctoMap");
      return;
    }
    msg.header.frame_id = map_frame_;
    msg.header.stamp = stamp;
    octomap_pub_->publish(msg);
  }

  void publishMarkers(const builtin_interfaces::msg::Time& stamp) {
    if (!publish_markers_) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = stamp;
    marker.ns = "octomap_occupied_voxels";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = resolution_;
    marker.color.r = 0.1f;
    marker.color.g = 0.55f;
    marker.color.b = 1.0f;
    marker.color.a = 0.55f;

    int count = 0;
    for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end;
         ++it) {
      if (!tree_->isNodeOccupied(*it)) continue;
      if (max_marker_voxels_ > 0 && count >= max_marker_voxels_) break;
      const auto c = it.getCoordinate();
      geometry_msgs::msg::Point p;
      p.x = c.x();
      p.y = c.y();
      p.z = c.z();
      marker.points.push_back(p);
      ++count;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    markers.markers.push_back(marker);
    marker_pub_->publish(markers);
  }

  void publishStatus() {
    std_msgs::msg::String msg;
    if (last_map_update_.nanoseconds() == 0) {
      msg.data = sceneFreshnessName(SceneFreshness::Unavailable);
    } else {
      const double age = (now() - last_map_update_).seconds();
      const auto freshness =
          age <= freshness_timeout_ ? SceneFreshness::Available
                                    : SceneFreshness::Stale;
      msg.data = std::string(sceneFreshnessName(freshness)) + " age_sec=" +
                 std::to_string(age) + " timeout_sec=" +
                 std::to_string(freshness_timeout_) + " frozen=" +
                 (frozen_ ? "1" : "0");
    }
    status_pub_->publish(msg);
  }

  void handleCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (frozen_) return;
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring cloud with empty frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped map_from_cloud;
    try {
      map_from_cloud = tf_buffer_.lookupTransform(
          map_frame_, msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(0.10));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "PointCloud TF unavailable %s <- %s: %s",
                           map_frame_.c_str(), msg->header.frame_id.c_str(),
                           e.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      tf2::doTransform(*msg, transformed, map_from_cloud);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "PointCloud transform failed: %s", e.what());
      return;
    }

    auto boxes = self_filter_boxes_;
    if (target_box_) boxes.push_back(*target_box_);
    if (self_filter_enabled_) refreshSelfFilterTransforms(boxes);
    const auto capsules = robotCapsules(rclcpp::Time(msg->header.stamp));

    const auto& t = map_from_cloud.transform.translation;
    const Eigen::Vector3d sensor_origin(t.x, t.y, t.z);
    std::vector<Point3> points;
    try {
      points = filterCloud(transformed, sensor_origin, boxes, capsules);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "PointCloud filter failed: %s", e.what());
      return;
    }

    publishFilteredCloud(points, msg->header.stamp);
    if (points.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Filtered cloud is empty");
      return;
    }

    updateMap(points, sensor_origin);
    clearSelfFilterVolumes(boxes);
    clearCapsuleVolumes(capsules);
    if (capture_pending_) {
      capture_pending_ = false;
      frozen_ = true;
    } else if (publish_period_ > 0.0 &&
               (now() - last_publish_).seconds() < publish_period_) {
      return;
    }
    last_publish_ = now();
    publishOctomap(msg->header.stamp);
    publishMarkers(msg->header.stamp);
    publishStatus();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "OctoMap updated: raw=%u filtered=%zu resolution=%.3f",
                         msg->width * msg->height, points.size(), resolution_);
  }

  std::string input_topic_;
  std::string filtered_topic_;
  std::string octomap_topic_;
  std::string marker_topic_;
  std::string status_topic_;
  std::string map_frame_;

  double resolution_ = 0.04;
  double min_range_ = 0.15;
  double max_sensor_range_ = 2.5;
  double voxel_leaf_size_ = 0.02;
  double freshness_timeout_ = 1.0;
  double occupancy_threshold_ = 0.50;
  double prob_hit_ = 0.70;
  double prob_miss_ = 0.40;
  double clamping_min_ = 0.12;
  double clamping_max_ = 0.97;
  bool reset_map_each_cloud_ = false;
  bool publish_markers_ = true;
  bool clear_self_filter_volumes_ = true;
  bool capture_pending_ = false;
  bool frozen_ = false;
  int max_marker_voxels_ = 30000;

  bool floor_filter_enabled_ = false;
  double floor_filter_min_z_ = -0.05;
  double floor_filter_max_z_ = 0.04;

  bool self_filter_enabled_ = true;
  bool self_filter_capsules_ = true;
  double capsule_margin_ = 0.03;
  double gripper_envelope_radius_ = 0.07;
  std::vector<SelfFilterBox> self_filter_boxes_;
  std::optional<SelfFilterBox> target_box_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr target_box_sub_;

  std::unique_ptr<octomap::OcTree> tree_;
  rclcpp::Time last_map_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  double publish_period_ = 0.0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  Eigen::Vector2d last_odom_ = Eigen::Vector2d::Zero();
  double last_odom_time_ = 0.0;
  bool have_odom_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace stretch_collision_planning

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stretch_collision_planning::OctomapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
