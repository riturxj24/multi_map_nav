#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include "multi_map_nav/action/multi_map_navigate.hpp"
#include "multi_map_nav/wormhole_db.hpp"

using namespace std::chrono_literals;
using MultiMapNavigate = multi_map_nav::action::MultiMapNavigate;
using GoalHandle = rclcpp_action::ServerGoalHandle<MultiMapNavigate>;

class MultiMapNavigationServer : public rclcpp::Node
{
public:
  MultiMapNavigationServer()
  : Node("multi_map_navigation_server")
  {

    this->declare_parameter("db_path", "/tmp/wormhole.db");
    this->declare_parameter("current_map", "room1");
    current_map_ = this->get_parameter("current_map").as_string();
    std::string db_path = this->get_parameter("db_path").as_string();
    db_.initialize(db_path);


    current_map_pub_ = this->create_publisher<std_msgs::msg::String>("/multi_map_nav/current_map", 10);
    map_change_client_ = this->create_client<std_srvs::srv::Empty>("/map_server/map_change");


    publish_current_map();


    action_server_ = rclcpp_action::create_server<MultiMapNavigate>(
      this,
      "multi_map_navigate",
      std::bind(&MultiMapNavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MultiMapNavigationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MultiMapNavigationServer::handle_accepted, this, std::placeholders::_1)
    );
  }

private:

  std::string current_map_;
  multi_map_nav::WormholeDB db_;

  rclcpp_action::Server<MultiMapNavigate>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_map_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr map_change_client_;


  void publish_current_map() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = current_map_;
    current_map_pub_->publish(std::move(msg));
  }

  void change_map(const std::string &new_map) {
    if (current_map_ == new_map) return;
    current_map_ = new_map;
    publish_current_map();

    if (map_change_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      map_change_client_->async_send_request(request);
    }
  }


  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiMapNavigate::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal to map '%s'", goal->map_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> /*goal_handle*/) {
    RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() {
      execute(goal_handle);
    }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MultiMapNavigate::Feedback>();
    auto result = std::make_shared<MultiMapNavigate::Result>();

    bool need_wormhole = (current_map_ != goal->map_name);
    std::vector<multi_map_nav::Wormhole> wormhole_path;

    if (need_wormhole) {
      bool found = db_.find_path(
        current_map_, geometry_msgs::msg::Pose(),
        goal->map_name, goal->pose.pose,
        wormhole_path);

      if (!found) {
        result->success = false;
        result->message = "No wormhole path found";
        goal_handle->succeed(result);
        return;
      }
    }


    float total_steps = 10.0 + wormhole_path.size() * 5.0;
    float completed_steps = 0.0;

    for (size_t i = 0; i <= wormhole_path.size(); ++i) {
      for (int j = 0; j < 5; ++j) {
        if (goal_handle->is_canceling()) {
          result->success = false;
          result->message = "Navigation canceled";
          goal_handle->canceled(result);
          return;
        }

        completed_steps += 1.0;
        feedback->current_map = current_map_;
        feedback->percent_complete = (completed_steps / total_steps) * 100.0f;
        feedback->status = "Navigating...";
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(300ms);
      }

      if (i < wormhole_path.size())
        change_map(wormhole_path[i].target_map);
    }

 
    if (current_map_ != goal->map_name)
      change_map(goal->map_name);

    result->success = true;
    result->message = "Navigation complete";
    goal_handle->succeed(result);
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMapNavigationServer>());
  rclcpp::shutdown();
  return 0;
}

