#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;  
class TFPublisherNode : public rclcpp::Node
{
public:
  TFPublisherNode() : Node("tf_publisher_node")
  {

    map_name_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/multi_map_nav/current_map", 10,
      std::bind(&TFPublisherNode::handle_map_change, this, std::placeholders::_1));


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);


    timer_ = this->create_wall_timer(
      100ms, std::bind(&TFPublisherNode::broadcast_dynamic_tf, this));


    publish_static_transform();

    current_map_ = "room1";  
    RCLCPP_INFO(this->get_logger(), "TF Publisher initialized for map: %s", current_map_.c_str());
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string current_map_;

  void handle_map_change(const std_msgs::msg::String::SharedPtr msg)
  {
    current_map_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Current map changed to: %s", current_map_.c_str());
  }

  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.x = 0.0;
    map_to_odom.transform.rotation.y = 0.0;
    map_to_odom.transform.rotation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;

    static_broadcaster_->sendTransform(map_to_odom);
    RCLCPP_INFO(this->get_logger(), "Published static transform: map -> odom");
  }

  void broadcast_dynamic_tf()
  {
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = this->now();
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link";

    if (current_map_ == "room1") {
      odom_to_base.transform.translation.x = 2.0;
      odom_to_base.transform.translation.y = 2.0;
    } else if (current_map_ == "room2") {
      odom_to_base.transform.translation.x = 0.0;
      odom_to_base.transform.translation.y = 0.0;
    } else if (current_map_ == "room3") {
      odom_to_base.transform.translation.x = -2.0;
      odom_to_base.transform.translation.y = -2.0;
    } else {
      odom_to_base.transform.translation.x = 1.0;
      odom_to_base.transform.translation.y = 1.0;
    }

    odom_to_base.transform.translation.z = 0.0;
    odom_to_base.transform.rotation.x = 0.0;
    odom_to_base.transform.rotation.y = 0.0;
    odom_to_base.transform.rotation.z = 0.0;
    odom_to_base.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(odom_to_base);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

