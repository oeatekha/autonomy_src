#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <cmath>
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

struct GraspPoses {
    // arm poses
    geometry_msgs::msg::Pose pre_grasp;
    geometry_msgs::msg::Pose grasp;
    geometry_msgs::msg::Pose post_grasp;
};
GraspPoses calculateGraspPoses(const geometry_msgs::msg::Pose& block_position); // func2: calculate grasp pose based on block position
void pick(moveit::planning_interface::MoveGroupInterface& move_group_interface_arm, moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, const geometry_msgs::msg::Pose& block_position); //func: pick up block
// void pick(moveit::planning_interface::MoveGroupInterface& move_group_interface_arm, 
//           moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, 
//           const geometry_msgs::msg::Pose& block_position);

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

    // Set block pose
    // TODO2: block might contains a rotation around z-axis (orientation)
    auto const block_position = [] {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.45;
        msg.position.y = 0.0;
        msg.position.z = 0.01;
        return msg;
    }();

    // TODO: when to grip? (get any comments from outside?)
    pick(move_group_interface_arm, move_group_interface_gripper, block_position);

    // TODO: Place
    

    // shutdown ros node
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
    if (!planAndMoveToNamedTarget(move_group_interface_gripper, "Released")) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open gripper.");
        return;
    }

    // Move to pre-grasp pose
    if (!planAndMoveToPose(move_group_interface_arm, poses.pre_grasp)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move to pre-grasp pose.");
        return;
    }

    // Move to grasp pose
    if (!planAndMoveToPose(move_group_interface_arm, poses.grasp)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move to grasp pose.");
        return;
    }
    
    // Close gripper
    if (!planAndMoveToNamedTarget(move_group_interface_gripper, "Grasping")) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open gripper.");
        return;
    }

    // TODO: check if successful 

    // Move to post-grasp pose
    if (!planAndMoveToPose(move_group_interface_arm, poses.post_grasp)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move to post-grasp pose.");
        return;
    }

    // Additional logic to handle after grasping the block
    // ...
}
    
    // STEP 1: Get the target pose for the block
    // STEP 2: Define pre-grasp pose and move to it (Note: the end-effector should be open and facing downwards)
    // STEP 3: Define grasp pose and grasp (Note: move down to block and close gripper)
    // STEP 4: Define post-grasp pose and move to it

    // -----------------------------------------------------------------------------------------------------//
    // // Set a target Pose with updated values !!!
    // auto const target_pose = [] {
    //     geometry_msgs::msg::Pose msg;
    //     msg.orientation.x = 1.0; // Rotation of 180 degrees around the X-axis
    //     msg.orientation.y = 0.0;
    //     msg.orientation.z = 0.0;
    //     msg.orientation.w = 0.0; // Cosine part of the quaternion for 180-degree rotation
    //     msg.position.x = 0.4;
    //     msg.position.y = 0.0;
    //     msg.position.z = 0.2;
    //     return msg;
    // }();

    // -----------------------------------------------------------------------------------------------------//
    // Create a ROS logger
    // auto const logger = rclcpp::get_logger("hello_moveit");

  
    // // Set the target pose
    // move_group_interface.setPoseTarget(random_pose); 
    // move_group_interface.setRandomTarget();	
    // move_group_interface.setNamedTarget("Sleep");
    // move_group_interface.setNamedTarget("Released");

    // -----------------------------------------------------------------------------------------------------//
    // //  set a target pose
    // auto const target_pose = [] {
    //     geometry_msgs::msg::Pose msg;
    //     msg.orientation.w = 1;
    //     msg.position.x = 0.4;
    //     msg.position.y = 0.0;
    //     msg.position.z = 0.2;
    //     return msg;
    // }();
    // move_group_interface.setPoseTarget(target_pose);

    // -----------------------------------------------------------------------------------------------------//
    // // Create the MoveIt MoveGroup Interface
    // using moveit::planning_interface::MoveGroupInterface;
    // // name for the arm need to match the group name define in locobot urdf (checked in rviz)
    // auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");
    // // auto move_group_interface = MoveGroupInterface(node, "interbotix_gripper");

    // -----------------------------------------------------------------------------------------------------//
    // // Create a plan to that target pose
    // auto const [success, plan] = [&move_group_interface]{
    //     moveit::planning_interface::MoveGroupInterface::Plan msg;
    //     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    //     return std::make_pair(ok, msg);
    // }();

    // -----------------------------------------------------------------------------------------------------//
    // // Execute the plan
    // if(success) {
    //     move_group_interface.execute(plan);
    //     // move_group_interface.pick("box1");
    // } else {
    //     // Ensure logger is correctly initialized earlier in your code
    //     RCLCPP_ERROR(logger, "Planning failed!");
    // }

    // Shutdown ROS


    // -----------------------------------------------------------------------------------------------------//

    // Create collision object for the robot to avoid
    // auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    //     moveit_msgs::msg::CollisionObject collision_object;
    //     collision_object.header.frame_id = frame_id;
    //     collision_object.id = "box1";
    //     shape_msgs::msg::SolidPrimitive primitive;

    //     // Define the size of the box in meters
    //     primitive.type = primitive.BOX;
    //     primitive.dimensions.resize(3);
    //     primitive.dimensions[primitive.BOX_X] = 0.02;
    //     primitive.dimensions[primitive.BOX_Y] = 0.02;
    //     primitive.dimensions[primitive.BOX_Z] = 0.02;

    //     // Define the pose of the box (relative to the frame_id)
    //     geometry_msgs::msg::Pose box_pose;
    //     box_pose.orientation.w = 1.0;
    //     box_pose.position.x = 0.3;
    //     box_pose.position.y = 0.0;
    //     box_pose.position.z = 0.0;

    //     collision_object.primitives.push_back(primitive);
    //     collision_object.primitive_poses.push_back(box_pose);
    //     collision_object.operation = collision_object.ADD;

    //     return collision_object;
    // }();

    // // Add the collision object to the scene
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // planning_scene_interface.applyCollisionObject(collision_object);

    // -----------------------------------------------------------------------------------------------------//
    // // Execute sequential movements
    // if (planAndMoveToPose(move_group_interface, target_pose)) {
    //     RCLCPP_INFO(logger, "Successfully moved to target_pose.");
    //     if (planAndMoveToPose(move_group_interface, pre_grasp_pose)) {
    //         RCLCPP_INFO(logger, "Successfully moved to pre_grasp_pose.");
    //     } else {
    //         RCLCPP_ERROR(logger, "Failed to move to pre_grasp_pose.");
    //     }
    // } else {
    //     RCLCPP_ERROR(logger, "Failed to move to target_pose.");
    // }