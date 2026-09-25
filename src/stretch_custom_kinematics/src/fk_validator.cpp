#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <fstream>
#include <iomanip>
#include <set>

#include "stretch_custom_kinematics/forward_kinematics.hpp"
#include "stretch_robot_interface/interface_node.hpp"
class FkValidator : public stretch_robot_interface::InterfaceNode {
 public:
  FkValidator()
      : InterfaceNode("fk_validator"),
        buffer_(get_clock()),
        listener_(buffer_, this, false) {
    require_exact_ = declare_parameter("require_exact_tf_sample", false);
    tf_samples_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(100).best_effort(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
          for (const auto& transform : message->transforms) {
            const auto stamp =
                rclcpp::Time(transform.header.stamp).nanoseconds();
            exact_samples_[stamp].insert(transform.child_frame_id);
          }
          while (exact_samples_.size() > 400)
            exact_samples_.erase(exact_samples_.begin());
        });
    position_tolerance_ = declare_parameter("position_tolerance_m", 0.001);
    angle_tolerance_ = declare_parameter("orientation_tolerance_rad", 0.001);
    if (position_tolerance_ <= 0 || angle_tolerance_ <= 0 ||
        !std::isfinite(position_tolerance_) || !std::isfinite(angle_tolerance_))
      throw std::invalid_argument("Invalid FK tolerance");
    const auto csv = declare_parameter("report_csv", std::string(""));
    if (!csv.empty()) {
      csv_.open(csv);
      if (!csv_) throw std::runtime_error("Cannot open report_csv");
      csv_ << "stamp,position_m,position_mm,orientation_rad,orientation_deg,"
              "pass\n";
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(250),
                               [this]() { compare(); });
  }

  ~FkValidator() override {
    RCLCPP_INFO(
        get_logger(),
        "FINAL compared=%zu failures=%zu skipped_no_exact_tf=%zu "
        "skipped_old=%zu tf_lookup_failures=%zu maxima=(%.12g m, %.12g rad)",
        count_, failures_, skipped_exact_, skipped_old_, lookup_failures_,
        max_position_, max_angle_);
  }

 private:
  void onState(const stretch_robot_interface::RobotState& q) override {
    // Keep the URDF that produced this state, so model updates cannot mix
    // snapshots.
    pending_.push_back({q, model_});
    while (pending_.size() > 200) pending_.pop_front();
  }
  void compare() {
    if (!ready()) {
      pending_.clear();
      return;
    }
    while (!pending_.empty()) {
      auto item = pending_.front();
      const auto& q = item.first;
      const double age =
          (now() - rclcpp::Time(q.timestamp, get_clock()->get_clock_type()))
              .seconds();
      if (age < 0.20)
        return;  // Allow the TF producer and network to deliver the matching
                 // time.
      pending_.pop_front();
      if (age > 1.0) {
        ++skipped_old_;
        continue;
      }
      if (require_exact_) {
        const auto found =
            exact_samples_.find(rclcpp::Time(q.timestamp).nanoseconds());
        bool exact = found != exact_samples_.end();
        if (exact) {
          for (const auto& joint : item.second->chain()) {
            if (joint->type != urdf::Joint::FIXED &&
                !found->second.count(joint->child_link_name))
              exact = false;
          }
        }
        if (!exact) {
          ++skipped_exact_;
          continue;
        }
      }
      try {
        auto tf = buffer_.lookupTransform(
            item.second->base(), item.second->tcp(),
            tf2::TimePoint(std::chrono::nanoseconds(
                rclcpp::Time(q.timestamp).nanoseconds())));
        Eigen::Isometry3d live = Eigen::Isometry3d::Identity();
        auto& p = tf.transform.translation;
        auto& r = tf.transform.rotation;
        live.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        live.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z)
                            .normalized()
                            .toRotationMatrix();
        stretch_custom_kinematics::ForwardKinematics fk(item.second);
        const auto e = stretch_custom_kinematics::poseError(
            fk.computeTcpTransform(q), live);
        bool pass = e.position_m <= position_tolerance_ &&
                    e.orientation_rad <= angle_tolerance_;
        ++count_;
        if (!pass) ++failures_;
        max_position_ = std::max(max_position_, e.position_m);
        max_angle_ = std::max(max_angle_, e.orientation_rad);
        if (csv_) {
          csv_ << std::setprecision(17) << rclcpp::Time(q.timestamp).seconds()
               << "," << e.position_m << "," << e.position_m * 1000 << ","
               << e.orientation_rad << "," << e.orientation_rad * 180 / M_PI
               << "," << pass << "\n";
          csv_.flush();
        }
        if (count_ % 25 == 1 || !pass)
          RCLCPP_INFO(get_logger(),
                      "FK %s [%s -> %s, timestamp matched] position=%.9g m "
                      "(%.9g mm), orientation=%.9g rad (%.9g deg); samples=%zu "
                      "failures=%zu maxima=(%.9g m, %.9g rad)",
                      pass ? "PASS" : "FAIL", item.second->base().c_str(),
                      item.second->tcp().c_str(), e.position_m,
                      e.position_m * 1000, e.orientation_rad,
                      e.orientation_rad * 180 / M_PI, count_, failures_,
                      max_position_, max_angle_);
      } catch (const tf2::TransformException& e) {
        ++lookup_failures_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF unavailable at state timestamp: %s", e.what());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "FK rejected: %s", e.what());
      }
    }
  }
  bool require_exact_ = false;
  size_t skipped_exact_ = 0, skipped_old_ = 0, lookup_failures_ = 0;
  std::map<int64_t, std::set<std::string>> exact_samples_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_samples_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::deque<std::pair<stretch_robot_interface::RobotState,
                       std::shared_ptr<stretch_robot_interface::RobotModel>>>
      pending_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream csv_;
  double position_tolerance_, angle_tolerance_, max_position_ = 0,
                                                max_angle_ = 0;
  size_t count_ = 0, failures_ = 0;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FkValidator>());
  rclcpp::shutdown();
  return 0;
}
