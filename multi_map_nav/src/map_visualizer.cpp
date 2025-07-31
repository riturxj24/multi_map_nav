#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "multi_map_nav/wormhole_db.hpp"

class MapVisualizer : public rclcpp::Node {
public:
  MapVisualizer() : Node("map_visualizer") {
    this->declare_parameter("db_path", "/tmp/wormhole.db");
    std::string db_path = this->get_parameter("db_path").as_string();

    db_.initialize(db_path);

    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/multi_map_nav/visualization", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapVisualizer::publish_visualization, this));

    RCLCPP_INFO(this->get_logger(), "Map visualizer started");
  }

private:
  multi_map_nav::WormholeDB db_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int marker_id_ = 0;

  void publish_visualization() {
    visualization_msgs::msg::MarkerArray marker_array;
    std::vector<std::string> maps = {"map1", "map2", "map3"};

    marker_id_ = 0;
    for (const auto &map_name : maps) {
      std::vector<multi_map_nav::Wormhole> wormholes;
      if (db_.get_wormholes(map_name, wormholes)) {
        for (const auto &wormhole : wormholes) {
          add_wormhole_markers(wormhole, marker_array);
        }
      }
    }

    markers_pub_->publish(marker_array);
  }

  void add_wormhole_markers(const multi_map_nav::Wormhole &wormhole,
                             visualization_msgs::msg::MarkerArray &marker_array) {
    auto now_stamp = this->now();

    marker_array.markers.push_back(create_arrow_marker(wormhole.source_pose, "wormhole_source", {1, 0, 0, 1}, now_stamp));
    marker_array.markers.push_back(create_arrow_marker(wormhole.target_pose, "wormhole_target", {0, 1, 0, 1}, now_stamp));
    marker_array.markers.push_back(create_connection_line(wormhole.source_pose, wormhole.target_pose, now_stamp));
    marker_array.markers.push_back(create_text_marker(wormhole.source_pose, wormhole.source_map, now_stamp));
    marker_array.markers.push_back(create_text_marker(wormhole.target_pose, wormhole.target_map, now_stamp));
  }

  visualization_msgs::msg::Marker create_arrow_marker(const geometry_msgs::msg::Pose &pose,
                                                       const std::string &ns,
                                                       const std::array<float, 4> &color,
                                                       const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.8;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    return marker;
  }

  visualization_msgs::msg::Marker create_connection_line(const geometry_msgs::msg::Pose &src,
                                                          const geometry_msgs::msg::Pose &tgt,
                                                          const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = stamp;
    line.ns = "wormhole_connection";
    line.id = marker_id_++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p1, p2;
    p1.x = src.position.x;
    p1.y = src.position.y;
    p1.z = src.position.z;
    p2.x = tgt.position.x;
    p2.y = tgt.position.y;
    p2.z = tgt.position.z;

    line.points = {p1, p2};
    line.scale.x = 0.1;
    line.color.r = 1.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 0.8;
    return line;
  }

  visualization_msgs::msg::Marker create_text_marker(const geometry_msgs::msg::Pose &pose,
                                                      const std::string &text,
                                                      const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = stamp;
    text_marker.ns = "wormhole_text";
    text_marker.id = marker_id_++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose = pose;
    text_marker.pose.position.z += 0.5;
    text_marker.text = text;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    return text_marker;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapVisualizer>());
  rclcpp::shutdown();
  return 0;
}

