#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
// #include <moveit_msgs/Grasp.h>

auto const logger = rclcpp::get_logger("hello_moveit");

using moveit::planning_interface::MoveGroupInterface;
using namespace moveit::task_constructor;

bool planAndMoveToPose(MoveGroupInterface& move_group_interface, const geometry_msgs::msg::Pose& target_pose); // func 1: plan and move to the target pose
bool planAndMoveToNamedTarget(MoveGroupInterface& move_group_interface, const std::string& name);

geometry_msgs::msg::Pose block_position;
geometry_msgs::msg::Pose received_msg;
bool block_position_received = false;


struct GraspPoses {
    // arm poses
    geometry_msgs::msg::Pose pre_grasp;
    geometry_msgs::msg::Pose grasp;
    geometry_msgs::msg::Pose post_grasp;
};
GraspPoses calculateGraspPoses(const geometry_msgs::msg::Pose& block_position); // func2: calculate grasp pose based on block position
void pick(moveit::planning_interface::MoveGroupInterface& move_group_interface_arm, moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, const geometry_msgs::msg::Pose& block_position); //func: pick up block

// Callback function for the block position subscriber
void blockPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    received_msg = *msg;
    block_position = [] {
        geometry_msgs::msg::Pose msg;
        msg.position.x = received_msg.position.x;
        msg.position.y = received_msg.position.y;
        msg.position.z = received_msg.position.z;
        return msg;
    }();
    if (block_position.position.x == 0.0 && block_position.position.y == 0.0 && block_position.position.z == 0.0) {
        block_position_received = false;
    } else {
        block_position_received = true;
    }
    block_position_received = true;
    RCLCPP_INFO(logger, "New block position received: [%f, %f, %f]", block_position.position.x, block_position.position.y, block_position.position.z);
}

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create move_group_interface for the desired group (arm + gripper in our case)
    MoveGroupInterface move_group_interface_arm(node, "interbotix_arm");
    MoveGroupInterface move_group_interface_gripper(node, "interbotix_gripper");


    auto const ee_name = move_group_interface_arm.getEndEffectorLink();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "End effector link: %s", ee_name.c_str());


    // TODO1: Get block position (from subscribing ROS topic?)
    // Callback function for the block position subscriber
    auto block_position_subscription = node->create_subscription<geometry_msgs::msg::Pose>(
        "/block_position",
        10, // QoS settings
        [&move_group_interface_arm, &move_group_interface_gripper](const geometry_msgs::msg::Pose::SharedPtr msg) {
            RCLCPP_INFO(logger, "Received block position: x = %f, y = %f, z = %f", msg->position.x, msg->position.y, msg->position.z);
            // Call pick function with received pose
            pick(move_group_interface_arm, move_group_interface_gripper, *msg);
        });

    // block_position = [] {
    //     geometry_msgs::msg::Pose msg;
    //     msg.position.x = 0.45;
    //     msg.position.y = 0.0;
    //     msg.position.z = 0.01;
    //     return msg;
    // }();

    // pick(move_group_interface_arm, move_group_interface_gripper, block_position);

    // spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


bool planAndMoveToPose(MoveGroupInterface& move_group_interface, const geometry_msgs::msg::Pose& target_pose) {
        // Set the target pose
        move_group_interface.setPoseTarget(target_pose);

        // Plan to the target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface.plan(plan));

        if(success) {
            // Execute the plan
            RCLCPP_INFO(logger, "Planning successed!");
            success = static_cast<bool>(move_group_interface.execute(plan));
            RCLCPP_INFO(logger, "Motion executed successfully!");
        } else {
            RCLCPP_ERROR(logger, "Planning failed! No execution allowed!");
        }

        return success;
    }

bool planAndMoveToNamedTarget(MoveGroupInterface& move_group_interface, const std::string& name) {
        // Set the named target
        move_group_interface.setNamedTarget(name);

        // Plan to the target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface.plan(plan));

        if(success) {
            // Execute the plan
            RCLCPP_INFO(logger, "Planning successed!");
            success = static_cast<bool>(move_group_interface.execute(plan));
            RCLCPP_INFO(logger, "Motion executed successfully!");
        } else {
            RCLCPP_ERROR(logger, "Planning failed! No execution allowed!");
        }

        return success;
    }


GraspPoses calculateGraspPoses(const geometry_msgs::msg::Pose& block_position) {
    GraspPoses poses;

    // Assuming a fixed orientation for all poses
    auto grasp_orientation = [] {
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0; // Example orientation: facing downwards
        orientation.y = 0.7071067811865476;
        orientation.z = 0.0;
        orientation.w = 0.7071067811865476;
        // orientation.x = 0.02489;
        // orientation.y = 0.01431;
        // orientation.z = 0.02490;
        // orientation.w = 0.99928;

        return orientation;
    }();

    // Set the same orientation for all poses
    poses.pre_grasp.orientation = grasp_orientation;
    poses.grasp.orientation = grasp_orientation;
    poses.post_grasp.orientation = grasp_orientation;

    // Adjust positions based on the block position
    // Pre-grasp pose: slightly above the block
    // print out block position to see if it is correct
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasp Position: [%f, %f, %f]", block_position.position.x, block_position.position.y, block_position.position.z);

    poses.pre_grasp.position.x = block_position.position.x;
    poses.pre_grasp.position.y = block_position.position.y;
    poses.pre_grasp.position.z = block_position.position.z + 0.1; // 10 cm above

    // Grasp pose: at the block's position (adjust z if needed to match your gripper's characteristics)
    poses.grasp.position.x = block_position.position.x;
    poses.grasp.position.y = block_position.position.y;
    poses.grasp.position.z = block_position.position.z + 0.012; // Assuming grasp at the block level

    // Post-grasp pose: slightly above the grasp pose
    poses.post_grasp.position.x = block_position.position.x;
    poses.post_grasp.position.y = block_position.position.y;
    poses.post_grasp.position.z = block_position.position.z + 0.1; // 10 cm above, similar to pre-grasp

    return poses;
}



void pick(moveit::planning_interface::MoveGroupInterface& move_group_interface_arm, moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, const geometry_msgs::msg::Pose& block_position) {
    
    // Calculate the grasp poses based on the block position
    GraspPoses poses = calculateGraspPoses(block_position);


    // Open gripper 
    // if (!planAndMoveToNamedTarget(move_group_interface_gripper, "Released")) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open gripper.");
    //     return;
    // }

    // Move to pre-grasp pose
    if (!planAndMoveToPose(move_group_interface_arm, poses.pre_grasp)) {
        // Print Pre-grasp pose
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pre-grasp pose: [%f, %f, %f]", poses.pre_grasp.position.x, poses.pre_grasp.position.y, poses.pre_grasp.position.z);
        // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move to pre-grasp pose.");
        return;
    }
}

