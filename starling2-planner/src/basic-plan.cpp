#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


moveit_msgs::msg::Constraints setRotationConstraints(std::vector<std::string> jointNames) {

  moveit_msgs::msg::Constraints c;

  // Lock roll & pitch ~= 0 (allow any yaw encoded in quat)
  for (auto name : jointNames) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = name;
    jc.position = 0.0;
    jc.tolerance_above = 1e-3;
    jc.tolerance_below = 1e-3;
    jc.weight = 1.0;
    c.joint_constraints.push_back(jc);
  }

  return c;

}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "basic_plan",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("basic_plan");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto move_group_interface = MoveGroupInterface(node, "quad-base");
  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setWorkspace(0.00, -1.0, -0.01, 2.0, 1.0, 0.01);

  // Set Constraints
  moveit_msgs::msg::Constraints constraints = setRotationConstraints({"virtual/rot_x","virtual/rot_y"});
  move_group_interface.setPathConstraints(constraints);

  // --- joint-space goal (floating base) ---
  std::map<std::string, double> joint_target{
    {"virtual/trans_x", 2.0},
    {"virtual/trans_y", 0.0},
    {"virtual/trans_z", 0.0},
    {"virtual/rot_x",   0.0},
    {"virtual/rot_y",   0.0},
    {"virtual/rot_z",   0.0},
    {"virtual/rot_w",   1.0},
  };
  move_group_interface.setJointValueTarget(joint_target);

  // Create collision object for the robot to avoid
  auto const collision_objects = [frame_id =
                                  move_group_interface.getPlanningFrame()] () {

    std::vector<moveit_msgs::msg::CollisionObject> obstacles;
    
    // First cube
    moveit_msgs::msg::CollisionObject collision_object1;
    collision_object1.header.frame_id = frame_id;
    collision_object1.id = "box1";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.2;
    primitive.dimensions[primitive.BOX_Z] = 0.2;
    
    geometry_msgs::msg::Pose box_pose1;
    box_pose1.orientation.w = 1.0;  // Identity orientation
    box_pose1.position.x = 1.35;
    box_pose1.position.y = -0.65;
    box_pose1.position.z = 0.0;
    
    collision_object1.primitives.push_back(primitive);
    collision_object1.primitive_poses.push_back(box_pose1);
    collision_object1.operation = collision_object1.ADD;
    
    // Second cube (same size, different position)
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = frame_id;
    collision_object2.id = "box2";
    
    geometry_msgs::msg::Pose box_pose2;
    box_pose2.orientation.w = 1.0;  // Identity orientation
    box_pose2.position.x = 0.85;     // Different position
    box_pose2.position.y = 0.0;
    box_pose2.position.z = 0.0;
    
    collision_object2.primitives.push_back(primitive);  // Same primitive (same size)
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object2.ADD;
    
    obstacles.push_back(collision_object1);
    obstacles.push_back(collision_object2);
    
    return obstacles;

  };

  auto obstacles = collision_objects();

  // Add the collision object to the scene
  for (auto& obs : obstacles ) {
    planning_scene_interface.applyCollisionObject(obs);
  }


  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();


  if (success) {

    RCLCPP_INFO(logger, "Planning succeeded in %.3f s", plan.planning_time_);

    const auto& jt = plan.trajectory_.joint_trajectory;
    const auto& md = plan.trajectory_.multi_dof_joint_trajectory;

    // Helper to get seconds as double
    auto to_sec = [](const builtin_interfaces::msg::Duration& d) {
      return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
    };

    // --- Multi-DOF waypoints (typical for floating-base drones) ---
    if (!md.points.empty()) {
      RCLCPP_INFO(logger, "Multi-DOF trajectory: %zu points, %zu joints",
                  md.points.size(), md.joint_names.size());
      for (size_t i = 0; i < md.points.size(); ++i) {

        const auto& p = md.points[i];
        double t = to_sec(p.time_from_start);

        for (size_t j = 0; j < p.transforms.size(); ++j) {

          const auto& name = (j < md.joint_names.size()) ? md.joint_names[j] : std::string("?");
          const auto& tf = p.transforms[j];

          RCLCPP_INFO(
            logger,
            "[%03zu] t=%.3f  %s  xyz=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
            i, t, name.c_str(),
            tf.translation.x, tf.translation.y, tf.translation.z,
            tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
          );
        }

      }
    }

    // --- Standard joint waypoints (if any) ---
    // if (!jt.points.empty()) {
    //   RCLCPP_INFO(logger, "Joint trajectory: %zu points, %zu joints",
    //               jt.points.size(), jt.joint_names.size());
    //   for (size_t i = 0; i < jt.points.size(); ++i) {
    //     const auto& p = jt.points[i];
    //     double t = to_sec(p.time_from_start);

    //     // positions/velocities/accelerations are aligned with jt.joint_names
    //     std::string pos_line = "pos=[";
    //     for (size_t k = 0; k < p.positions.size(); ++k) {
    //       pos_line += (k ? ", " : "") + std::to_string(p.positions[k]);
    //     }
    //     pos_line += "]";

    //     RCLCPP_INFO(logger, "[%03zu] t=%.3f  %s", i, t, pos_line.c_str());

    //     if (!p.velocities.empty()) {
    //       std::string vel_line = "vel=[";
    //       for (size_t k = 0; k < p.velocities.size(); ++k) {
    //         vel_line += (k ? ", " : "") + std::to_string(p.velocities[k]);
    //       }
    //       vel_line += "]";
    //       RCLCPP_INFO(logger, "         %s", vel_line.c_str());
    //     }
    //     if (!p.accelerations.empty()) {
    //       std::string acc_line = "acc=[";
    //       for (size_t k = 0; k < p.accelerations.size(); ++k) {
    //         acc_line += (k ? ", " : "") + std::to_string(p.accelerations[k]);
    //       }
    //       acc_line += "]";
    //       RCLCPP_INFO(logger, "         %s", acc_line.c_str());
    //     }
    //   }
    // }

  } else {
    RCLCPP_ERROR(logger, "Planning failed or timed out");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}