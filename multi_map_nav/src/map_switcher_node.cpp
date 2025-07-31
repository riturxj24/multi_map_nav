#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <unordered_map>

class MapSwitcherNode : public rclcpp::Node {
public:
  MapSwitcherNode() : Node("map_switcher") {
    active_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    map_name_sub_ = create_subscription<std_msgs::msg::String>(
      "/multi_map_nav/current_map", 10,
      std::bind(&MapSwitcherNode::on_map_name_change, this, std::placeholders::_1));

    for (const auto &room : {"room1", "room2", "room3"}) {
      std::string topic_name = "map_" + std::string(room);
      map_subs_[room] = create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_name, 1,
        [this, room](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Received map for %s", room);
          maps_[room] = *msg;
          if (current_map_ == room) publish_current_map();
        });
    }

    current_map_ = "room1";
    timer_ = create_wall_timer(std::chrono::seconds(1),
              std::bind(&MapSwitcherNode::publish_current_map, this));
    RCLCPP_INFO(get_logger(), "Initialized map_switcher with default map: %s", current_map_.c_str());
  }

private:
  std::string current_map_;
  nav_msgs::msg::OccupancyGrid last_valid_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr active_map_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;
  std::unordered_map<std::string, nav_msgs::msg::OccupancyGrid> maps_;
  std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  void on_map_name_change(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &new_map = msg->data;
    if (new_map != current_map_) {
      RCLCPP_INFO(get_logger(), "Switching from %s to %s", current_map_.c_str(), new_map.c_str());
      current_map_ = new_map;
      publish_current_map();
    }
  }

  void publish_current_map() {
    if (maps_.count(current_map_) && !maps_[current_map_].data.empty()) {
      auto map = maps_[current_map_];
      map.header.frame_id = "map";
      map.header.stamp = now();
      active_map_pub_->publish(map);
      last_valid_map_ = map;
      RCLCPP_INFO(get_logger(), "Published map: %s", current_map_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No valid data for %s, using fallback", current_map_.c_str());
      if (last_valid_map_.data.empty()) generate_placeholder();
      last_valid_map_.header.stamp = now();
      active_map_pub_->publish(last_valid_map_);
    }
  }

  void generate_placeholder() {
    nav_msgs::msg::OccupancyGrid placeholder;
    placeholder.header.frame_id = "map";
    placeholder.info.resolution = 0.05;
    placeholder.info.width = 200;
    placeholder.info.height = 200;
    placeholder.info.origin.position.x = -5.0;
    placeholder.info.origin.position.y = -5.0;
    placeholder.info.origin.orientation.w = 1.0;

    std::vector<int8_t> data(200 * 200, 0);
    for (int i = 0; i < 200; ++i) {
      data[i] = 100;                             // top
      data[199 * 200 + i] = 100;                 // bottom
      data[i * 200] = 100;                       // left
      data[i * 200 + 199] = 100;                 // right
    }
    placeholder.data = data;
    last_valid_map_ = placeholder;
    RCLCPP_INFO(get_logger(), "Generated placeholder map");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSwitcherNode>());
  rclcpp::shutdown();
  return 0;
}

