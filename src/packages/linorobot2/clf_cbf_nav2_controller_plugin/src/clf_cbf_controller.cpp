#include "clf_cbf_nav2_controller_plugin/clf_cbf_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace std::chrono_literals;
using namespace clf_cbf_nav2_controller_plugin;

void ClfCbfController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = node;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  
  // Declare and get use_sim_time parameter
  node_->declare_parameter(name_ + ".use_sim_time", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".use_sim_time", use_sim_time_);
  
  // Declare other parameters
  node_->declare_parameter(name_ + ".service_timeout_ms", rclcpp::ParameterValue(100));
  node_->get_parameter(name_ + ".service_timeout_ms", service_timeout_ms_);
  
  node_->declare_parameter(name_ + ".goal_tolerance", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
  
  service_name_ = "/dr_clf_cbf_controller/compute_twist";
  compute_twist_client_ = node_->create_client<dr_clf_cbf_interfaces::srv::ComputeTwist>(service_name_);
  
  RCLCPP_INFO(
    node_->get_logger(), 
    "CLF-CBF Controller configured with name: %s, use_sim_time: %s", 
    name.c_str(),
    use_sim_time_ ? "true" : "false"
  );
}

void ClfCbfController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating CLF-CBF Controller");
  
  // Wait for service to be available
  if (!compute_twist_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Service %s not available after waiting 5s", 
      service_name_.c_str()
    );
  } else {
    RCLCPP_INFO(node_->get_logger(), "Service %s is available", service_name_.c_str());
  }
}

void ClfCbfController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating CLF-CBF Controller");
  goal_reached_ = false;
}

void ClfCbfController::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up CLF-CBF Controller");
  compute_twist_client_.reset();
  node_.reset();
}

void ClfCbfController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  current_path_ = path;
  goal_reached_ = false;
  RCLCPP_INFO(node_->get_logger(), "New path set with %zu poses", path.poses.size());
}

void ClfCbfController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(speed_limit_mutex_);
  
  if (percentage) {
    speed_limit_ = speed_limit;
  } else {
    // If absolute value, convert to percentage (assuming max speed of 1.0 m/s)
    speed_limit_ = speed_limit / 1.0;
  }
  
  RCLCPP_INFO(
    node_->get_logger(),
    "Speed limit set to: %.2f %s",
    speed_limit,
    percentage ? "%" : "m/s"
  );
}

geometry_msgs::msg::TwistStamped ClfCbfController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto request = std::make_shared<dr_clf_cbf_interfaces::srv::ComputeTwist::Request>();
  
  request->current_pose = pose;
  
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    request->path = current_path_;
  }
  
  geometry_msgs::msg::TwistStamped cmd;
  
  // Use sim time or wall time based on parameter
  if (use_sim_time_) {
    cmd.header.stamp = node_->now();
  } else {
    cmd.header.stamp = node_->get_clock()->now();
  }
  cmd.header.frame_id = "base_link";

  if (!compute_twist_client_->wait_for_service(std::chrono::milliseconds(service_timeout_ms_))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available");
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  auto future = compute_twist_client_->async_send_request(request);
  auto status = future.wait_for(std::chrono::milliseconds(service_timeout_ms_));

  if (status == std::future_status::ready) {
    auto response = future.get();
    if (response->success) {
      cmd.twist = response->cmd_vel;
      goal_reached_ = response->goal_reached;
      
      // Apply speed limit
      {
        std::lock_guard<std::mutex> lock(speed_limit_mutex_);
        if (speed_limit_ < 1.0) {
          cmd.twist.linear.x *= speed_limit_;
          cmd.twist.linear.y *= speed_limit_;
          cmd.twist.angular.z *= speed_limit_;
        }
      }
      
    } else {
      RCLCPP_WARN(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;
      cmd.twist.linear.z = 0.0;
      cmd.twist.angular.x = 0.0;
      cmd.twist.angular.y = 0.0;
      cmd.twist.angular.z = 0.0;
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Service call timed out");
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;
  }

  return cmd;
}

PLUGINLIB_EXPORT_CLASS(clf_cbf_nav2_controller_plugin::ClfCbfController, nav2_core::Controller)