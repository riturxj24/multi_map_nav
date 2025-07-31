#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "multi_map_nav/action/multi_map_navigate.hpp"

using namespace std::chrono_literals;
using MultiMapNavigate = multi_map_nav::action::MultiMapNavigate;
using GoalHandle = rclcpp_action::ClientGoalHandle<MultiMapNavigate>;

class MultiMapNavigationClient : public rclcpp::Node
{
public:
  MultiMapNavigationClient()
  : Node("multi_map_navigation_client")
  {
    declare_goal_parameters();

    client_ = rclcpp_action::create_client<MultiMapNavigate>(this, "multi_map_navigate");

    timer_ = this->create_wall_timer(
      2s, std::bind(&MultiMapNavigationClient::send_goal, this));
  }

private:
  rclcpp_action::Client<MultiMapNavigate>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;


  void declare_goal_parameters() {
    this->declare_parameter("map_name", "room2");
    this->declare_parameter("x", 2.0);
    this->declare_parameter("y", 3.0);
    this->declare_parameter("z", 0.0);
    this->declare_parameter("qx", 0.0);
    this->declare_parameter("qy", 0.0);
    this->declare_parameter("qz", 0.0);
    this->declare_parameter("qw", 1.0);
  }

 
  void send_goal()
  {
    timer_->cancel();  

    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = build_goal_message();

    RCLCPP_INFO(this->get_logger(),
      "Sending goal to map '%s' at (%.2f, %.2f, %.2f)",
      goal_msg.map_name.c_str(),
      goal_msg.pose.pose.position.x,
      goal_msg.pose.pose.position.y,
      goal_msg.pose.pose.position.z);

    auto options = rclcpp_action::Client<MultiMapNavigate>::SendGoalOptions();
    options.goal_response_callback = std::bind(&MultiMapNavigationClient::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback = std::bind(&MultiMapNavigationClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(&MultiMapNavigationClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, options);
  }

  
  MultiMapNavigate::Goal build_goal_message() {
    MultiMapNavigate::Goal goal;
    goal.map_name = this->get_parameter("map_name").as_string();

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = this->get_parameter("x").as_double();
    goal.pose.pose.position.y = this->get_parameter("y").as_double();
    goal.pose.pose.position.z = this->get_parameter("z").as_double();
    goal.pose.pose.orientation.x = this->get_parameter("qx").as_double();
    goal.pose.pose.orientation.y = this->get_parameter("qy").as_double();
    goal.pose.pose.orientation.z = this->get_parameter("qz").as_double();
    goal.pose.pose.orientation.w = this->get_parameter("qw").as_double();

    return goal;
  }

  
  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server. Awaiting result...");
    }
  }

  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const MultiMapNavigate::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Feedback: %.1f%% complete | Current map: %s | Status: %s",
      feedback->percent_complete,
      feedback->current_map.c_str(),
      feedback->status.c_str());
  }

  void result_callback(const GoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        break;
    }
    rclcpp::shutdown();
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMapNavigationClient>());
  rclcpp::shutdown();
  return 0;
}

