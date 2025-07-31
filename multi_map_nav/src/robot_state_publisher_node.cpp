#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RobotStatePublisherNode : public rclcpp::Node
{
public:
  RobotStatePublisherNode() : Node("robot_state_publisher_node")
  {
    rclcpp::QoS qos(10);
    qos.transient_local();  // Acts like latched topic
    robot_description_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "robot_description", qos);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotStatePublisherNode::publish_robot_description, this));

    RCLCPP_INFO(this->get_logger(), "Robot state publisher initialized");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_robot_description()
  {
    timer_->cancel();

    std::string robot_description = R"(
<?xml version="1.0"?>
<robot name="custom_diff_robot">
  <link name="base_footprint"/>

  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
        <box size="0.300 0.200 0.100"/>
      </geometry>
      <material name="dark_blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
        <box size="0.300 0.200 0.100"/>
      </geometry>
    </collision>
  </link>

  <link name="wheel_left_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
    </collision>
  </link>

  <link name="wheel_right_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
    </collision>
  </link>

  <!-- LIDAR sensor link -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.01"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.01"/>
      </geometry>
    </collision>
  </link>

  <!-- Joints -->
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.050" rpy="0 0 0"/>
  </joint>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0 0.12 -0.03" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <origin xyz="0 -0.12 -0.03" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Materials -->
  <material name="dark">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>

  <material name="dark_blue">
    <color rgba="0.0 0.0 0.5 1.0"/>
  </material>

  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
</robot>
    )";

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = robot_description;
    robot_description_publisher_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Published custom robot description with LIDAR");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStatePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

