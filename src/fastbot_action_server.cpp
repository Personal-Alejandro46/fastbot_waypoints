#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "fastbot_msgs/action/waypoint_action.hpp"

class WaypointActionServer : public rclcpp::Node {
public:
  using WaypointAction = fastbot_msgs::action::WaypointAction;
  using GoalHandle = rclcpp_action::ServerGoalHandle<WaypointAction>;

  explicit WaypointActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fastbot_as", options) {
    _cb_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = _cb_group;

    _pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot/cmd_vel", 1);
    _sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&WaypointActionServer::_clbk_odom, this,
                  std::placeholders::_1),
        sub_opts);

    _action_server = rclcpp_action::create_server<WaypointAction>(
        this, "fastbot_waypoint_as",
        std::bind(&WaypointActionServer::_handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&WaypointActionServer::_handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&WaypointActionServer::_handle_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), _cb_group);

    RCLCPP_INFO(this->get_logger(), "Action server started");
  }

private:
  rclcpp::CallbackGroup::SharedPtr _cb_group;
  rclcpp_action::Server<WaypointAction>::SharedPtr _action_server;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_cmd_vel;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;

  geometry_msgs::msg::Point _position;
  double _yaw = 0.0;
  std::string _state = "idle";

  static constexpr double _yaw_precision = M_PI / 90.0;
  static constexpr double _dist_precision = 0.05;

  void _clbk_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    _position = msg->pose.pose.position;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    _yaw = yaw;
  }

  rclcpp_action::GoalResponse
  _handle_goal(const rclcpp_action::GoalUUID &,
               std::shared_ptr<const WaypointAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Goal received → x=%.3f  y=%.3f",
                goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  _handle_cancel(const std::shared_ptr<GoalHandle>) {
    RCLCPP_INFO(this->get_logger(), "Goal cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void _handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&WaypointActionServer::_execute, this, goal_handle)}
        .detach();
  }

  static double _normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  void _stop_robot() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    _pub_cmd_vel->publish(stop);
  }

  void _execute(const std::shared_ptr<GoalHandle> goal_handle) {
    rclcpp::Rate rate(25);
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();

    const geometry_msgs::msg::Point des_pos = goal->position;

    double desired_yaw = std::atan2(des_pos.y - _position.y, des_pos.x - _position.x);
    double err_pos = std::hypot(des_pos.x - _position.x, des_pos.y - _position.y);
    double err_yaw = _normalize_angle(desired_yaw - _yaw);

    RCLCPP_INFO(this->get_logger(),
                "Position X: %.4f | Position Y: %.4f | Desired Yaw: %.4f rad "
                "(%.2f deg)",
                _position.x, _position.y, desired_yaw,
                desired_yaw * 180.0 / M_PI);

    while (err_pos > _dist_precision && rclcpp::ok()) {
      desired_yaw = std::atan2(des_pos.y - _position.y, des_pos.x - _position.x);
      err_yaw = _normalize_angle(desired_yaw - _yaw);
      err_pos = std::hypot(des_pos.x - _position.x, des_pos.y - _position.y);

      RCLCPP_INFO(this->get_logger(),
                  "[%s] | pos=(%.3f, %.3f) | err_pos=%.4f m | "
                  "yaw=%.3f rad (%.1f deg) | err_yaw=%.4f rad (%.1f deg)",
                  _state.c_str(), _position.x, _position.y, err_pos, _yaw,
                  _yaw * 180.0 / M_PI, err_yaw, err_yaw * 180.0 / M_PI);

      if (goal_handle->is_canceling()) {
        _stop_robot();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        return;
      }

      geometry_msgs::msg::Twist twist;
      if (std::fabs(err_yaw) > _yaw_precision) {
        _state = "fix yaw";
        twist.angular.z = (err_yaw > 0.0) ? 0.25 : -0.25;
      } else {
        _state = "go to point";
        twist.linear.x = 0.26;
        twist.angular.z = 0.0;
      }
      _pub_cmd_vel->publish(twist);

      feedback->position = _position;
      feedback->state = _state;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    _stop_robot();

    RCLCPP_INFO(this->get_logger(), "=== GOAL CONCLUIDO ===");
    RCLCPP_INFO(this->get_logger(), "Final Position : X=%.4f m | Y=%.4f m",
                _position.x, _position.y);
    RCLCPP_INFO(this->get_logger(), "Error Yaw final: %.4f rad (%.2f deg)",
                err_yaw, err_yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Err pos final  : %.4f m", err_pos);
    RCLCPP_INFO(this->get_logger(), "=====================");

    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}