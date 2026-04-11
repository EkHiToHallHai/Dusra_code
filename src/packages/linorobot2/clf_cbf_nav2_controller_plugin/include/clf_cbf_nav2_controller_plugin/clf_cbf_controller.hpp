#ifndef CLF_CBF_NAV2_CONTROLLER_PLUGIN__CLF_CBF_CONTROLLER_HPP_
#define CLF_CBF_NAV2_CONTROLLER_PLUGIN__CLF_CBF_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "dr_clf_cbf_interfaces/srv/compute_twist.hpp"

namespace clf_cbf_nav2_controller_plugin
{

class ClfCbfController : public nav2_core::Controller
{
public:
  ClfCbfController() = default;
  ~ClfCbfController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  
  rclcpp::Client<dr_clf_cbf_interfaces::srv::ComputeTwist>::SharedPtr compute_twist_client_;
  std::string service_name_;
  
  nav_msgs::msg::Path current_path_;
  std::mutex path_mutex_;
  std::mutex speed_limit_mutex_;
  
  bool goal_reached_{false};
  bool use_sim_time_{false};
  int service_timeout_ms_{100};
  double goal_tolerance_{0.25};
  double speed_limit_{1.0};  // 1.0 means 100%, no limit
};

}  // namespace clf_cbf_nav2_controller_plugin

#endif  // CLF_CBF_NAV2_CONTROLLER_PLUGIN__CLF_CBF_CONTROLLER_HPP_