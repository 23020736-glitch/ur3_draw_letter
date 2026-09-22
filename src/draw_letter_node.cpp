#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("draw_letter_node", node_options);

    auto marker_pub = move_group_node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    static const std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    std::string fixed_frame = "world";
    move_group.setPoseReferenceFrame(fixed_frame);
    move_group.setMaxVelocityScalingFactor(0.15); 
    move_group.setMaxAccelerationScalingFactor(0.15);

    std::this_thread::sleep_for(3s);

    // BUOC 1: DUNG TAY MAY DUNG DUNG VE TU THE CHUAN (READY POSE)
    RCLCPP_INFO(move_group_node->get_logger(), "Dung tay may len tu the dung sang...");
    move_group.setNamedTarget("ready");
    move_group.move();

    auto publish_marker = [&](int id, const std::vector<geometry_msgs::msg::Point>& points) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = fixed_frame;
        marker.header.stamp = move_group_node->now();
        marker.ns = "letter_D";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.012;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.points = points;
        marker_pub->publish(marker);
    };

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    // BUOC 2: DI CHUYEN DEN DIEM VE O TAM CAO (Z = 0.40m)
    RCLCPP_INFO(move_group_node->get_logger(), "Di chuyen den tam cao z=0.40m...");
    target_pose.position.x = 0.35;  
    target_pose.position.y = -0.06; 
    target_pose.position.z = 0.40;  
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (static_cast<bool>(move_group.plan(my_plan))) {
        move_group.execute(my_plan);
    }

    // BUOC 3: VE NET 1 (THAN VA BUNG CHU D O TAM CAO Z = 0.30m -> 0.46m)
    RCLCPP_INFO(move_group_node->get_logger(), "Dang ve Net 1: Chu D...");
    std::vector<geometry_msgs::msg::Pose> waypoints_D;
    std::vector<geometry_msgs::msg::Point> pts_D;

    auto add_pt_D = [&](double x, double y, double z) {
        target_pose.position.x = x; target_pose.position.y = y; target_pose.position.z = z;
        waypoints_D.push_back(target_pose);
        geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = z;
        pts_D.push_back(p);
    };

    // Toa do nang cao giup khuoyu va vai tay may vuan dung dung
    add_pt_D(0.35, -0.06, 0.30); // Ha but
    add_pt_D(0.35, -0.06, 0.46); // Than so dung
    add_pt_D(0.35,  0.00, 0.46); // Bung tren
    add_pt_D(0.35,  0.05, 0.38); // Bung giua
    add_pt_D(0.35,  0.00, 0.30); // Bung duoi
    add_pt_D(0.35, -0.06, 0.30); // Khep chan D

    moveit_msgs::msg::RobotTrajectory trajectory_D;
    double fraction = move_group.computeCartesianPath(waypoints_D, 0.005, 0.0, trajectory_D);
    if (fraction > 0.5) {
        move_group.execute(trajectory_D);
        publish_marker(1, pts_D);
    }

    // BUOC 4: NHAC BUT SANG NET 2
    RCLCPP_INFO(move_group_node->get_logger(), "Nhac BUT sang Net 2...");
    std::vector<geometry_msgs::msg::Pose> waypoints_lift;
    target_pose.position.x = 0.30; target_pose.position.y = -0.06; target_pose.position.z = 0.38;
    waypoints_lift.push_back(target_pose);
    target_pose.position.x = 0.35; target_pose.position.y = -0.10; target_pose.position.z = 0.38;
    waypoints_lift.push_back(target_pose);
    
    moveit_msgs::msg::RobotTrajectory traj_lift;
    if (move_group.computeCartesianPath(waypoints_lift, 0.005, 0.0, traj_lift) > 0.5) {
        move_group.execute(traj_lift);
    }

    // BUOC 5: VE NET 2 (DAU GACH NGANG CHU Đ)
    RCLCPP_INFO(move_group_node->get_logger(), "Dang ve Net 2: Dau gach ngang...");
    std::vector<geometry_msgs::msg::Pose> waypoints_Dash;
    std::vector<geometry_msgs::msg::Point> pts_Dash;

    target_pose.position.x = 0.35; target_pose.position.y = -0.10; target_pose.position.z = 0.38;
    waypoints_Dash.push_back(target_pose);
    geometry_msgs::msg::Point p1; p1.x = 0.35; p1.y = -0.10; p1.z = 0.38; pts_Dash.push_back(p1);

    target_pose.position.x = 0.35; target_pose.position.y = -0.02; target_pose.position.z = 0.38;
    waypoints_Dash.push_back(target_pose);
    geometry_msgs::msg::Point p2; p2.x = 0.35; p2.y = -0.02; p2.z = 0.38; pts_Dash.push_back(p2);

    moveit_msgs::msg::RobotTrajectory trajectory_Dash;
    fraction = move_group.computeCartesianPath(waypoints_Dash, 0.005, 0.0, trajectory_Dash);
    if (fraction > 0.5) {
        move_group.execute(trajectory_Dash);
        publish_marker(2, pts_Dash);
    }

    // Dua tay may ve tu the dung nghi cao ranh
    move_group.setNamedTarget("ready");
    move_group.move();

    RCLCPP_INFO(move_group_node->get_logger(), "==> HOAN THANH VE CHU Đ (TAY MAY DUNG DUNG)! <==");

    while (rclcpp::ok()) {
        publish_marker(1, pts_D);
        publish_marker(2, pts_Dash);
        std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
