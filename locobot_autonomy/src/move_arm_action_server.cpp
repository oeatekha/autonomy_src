#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>

class MoveArmNode : public rclcpp::Node
{
public:

  using MoveitGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using GoalHandle = rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>;

  MoveArmNode() : Node("MoveArmNode")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
        "goal", 10, std::bind(&MoveArmNode::goal_callback, this, std::placeholders::_1));

    
  auto const move_group_node = std::make_shared<rclcpp::Node>(
        "locobot_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
      ); 
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_; // Declare the subscriber
  std::shared_ptr<rclcpp::Node> move_group_node;

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  }

  // Make goal_callback a member function of MoveArmNode
  void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);
    goal_point_ = *msg;
    
  }

  geometry_msgs::msg::Point goal_point_;

  void execute()
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    auto result = std::make_shared<moveit_msgs::action::MoveGroup::Result>();
    auto feedback = std::make_shared<moveit_msgs::action::MoveGroup::Feedback>();
    
    // create ROS Logger
    auto const logger = rclcpp::get_logger("MoveArmNode");

    // Set a Target Pose
    geometry_msgs::msg::Pose target_pose;
    auto move_group_interface = MoveitGroupInterface(this->move_group_node, "interbotix_arm");
 
    // position
    target_pose.position.x = goal_point_.x;
    target_pose.position.y = goal_point_.y;
    target_pose.position.z = goal_point_.z;
    //orientation
    tf2::Quaternion q;
    // float roll = (3.14159/180)*goal->pose[3];
    // float pitch = (3.14159/180)*goal->pose[4];
    // float yaw = (3.14159/180)*goal->pose[5];
    // q.setRPY(roll, pitch, yaw);
    // target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;
    target_pose.orientation = tf2::toMsg(q);
    move_group_interface.setPoseTarget(target_pose);


    // Create Target Plan for Target Pose
    auto const [plan_success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(plan_success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }

  }


}; // class MoveArmNode

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveArmNode>());
  rclcpp::shutdown();
  return 0;
}
