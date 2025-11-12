#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <random>
#include <fstream>
#include <filesystem>

#include <nlohmann/json.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

// Global dataset variables
nlohmann::json dataset;
int trajectory_id = 0;
const std::string DATASET_FILE = "trajectory_dataset.json";

void initializeDataset() {
    // Try to load existing dataset
    std::ifstream infile(DATASET_FILE);
    if (infile.good()) {
        infile >> dataset;
        trajectory_id = dataset["data"].size();
        infile.close();
    } else {
        // Create new dataset
        dataset["dataset_info"]["version"] = "1.0";
        dataset["dataset_info"]["robot"] = "quadrotor";
        dataset["dataset_info"]["planner"] = "moveit_ompl";
        dataset["data"] = nlohmann::json::array();
    }
}

void saveTrajectory(const std::map<std::string, double>& joint_target,
                    const std::vector<moveit_msgs::msg::CollisionObject>& obstacles,
                    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                    const builtin_interfaces::msg::Duration& d) {
    
    nlohmann::json entry;
    entry["id"] = trajectory_id++;
    
    // Helper function to round to 3 decimal places
    auto round3 = [](double value) {
        return std::round(value * 1000.0) / 1000.0;
    };
    
    // Save Goal Configuration
    entry["config"]["goal"]["x"] = round3(joint_target.at("virtual/trans_x"));
    entry["config"]["goal"]["y"] = round3(joint_target.at("virtual/trans_y"));
    entry["config"]["goal"]["z"] = round3(joint_target.at("virtual/trans_z"));
    
    // Save obstacles
    // for (size_t i = 0; i < obstacles.size(); ++i) {
    //     entry["config"]["obstacles"][i]["x"] = round3(obstacles[i].primitive_poses[0].position.x);
    //     entry["config"]["obstacles"][i]["y"] = round3(obstacles[i].primitive_poses[0].position.y);
    //     entry["config"]["obstacles"][i]["z"] = round3(obstacles[i].primitive_poses[0].position.z);
    // }
    
    // Save trajectory
    entry["trajectory"]["planning_time"] = round3(plan.planning_time_);
    
    // Helper to get seconds as double
    auto to_sec = [](const builtin_interfaces::msg::Duration& d) {
        return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
    };
    
    const auto& md = plan.trajectory_.multi_dof_joint_trajectory;
    for (size_t i = 0; i < md.points.size(); ++i) {
        const auto& p = md.points[i];
        const auto& tf = p.transforms[0];
        
        nlohmann::json waypoint;
        waypoint["time"] = round3(to_sec(p.time_from_start));
        waypoint["x"] = round3(tf.translation.x);
        waypoint["y"] = round3(tf.translation.y);
        waypoint["z"] = round3(tf.translation.z);
        waypoint["qx"] = round3(tf.rotation.x);
        waypoint["qy"] = round3(tf.rotation.y);
        waypoint["qz"] = round3(tf.rotation.z);
        waypoint["qw"] = round3(tf.rotation.w);
        
        entry["trajectory"]["waypoints"].push_back(waypoint);
    }
    
    dataset["data"].push_back(entry);
}

std::map<std::string, double> setJointTarget(double xmin, double xmax, double ymin, double ymax) {

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> x_dist(xmin, xmax);
  std::uniform_real_distribution<double> y_dist(ymin, ymax);

  // Generate random y and z values
  double random_x = x_dist(gen);
  double random_y = y_dist(gen);

  std::map<std::string, double> joint_target{
    {"virtual/trans_x", random_x},
    {"virtual/trans_y", random_y},
    {"virtual/trans_z", 0.0},
    {"virtual/rot_x",   0.0},
    {"virtual/rot_y",   0.0},
    {"virtual/rot_z",   0.0},
    {"virtual/rot_w",   1.0},
  };

  return joint_target;

}

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
    "multi_obs_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("multi_obs_planner");

  // Initialize dataset
  initializeDataset();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "quad-base");

  // Initialize Planning Scene Interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Set Workspace Bounds and Planning Time
  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setWorkspace(0.00, -2.0, -0.01, 2.0, 2.0, 0.01);

  // Set Constraints
  moveit_msgs::msg::Constraints constraints = setRotationConstraints({"virtual/rot_x","virtual/rot_y"});
  move_group_interface.setPathConstraints(constraints);

  int iterations = 1000;
  int successes = 0;

  while (successes < iterations) {
    /* -------------------------------------------------------------------------------------------------------- */
    
    // Set Joint Target
    std::map<std::string, double> joint_target = setJointTarget(1.75, 1.9, -0.5, 0.5);
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
        

    // Generate the random obstacles
    auto obstacles = collision_objects();

    // Add all collision objects to the scene
    for (const auto& obstacle : obstacles) {
      planning_scene_interface.applyCollisionObject(obstacle);

      // RCLCPP_INFO(logger, "Added obstacle '%s' at position (%.3f, %.3f, %.3f)", 
      //             obstacle.id.c_str(),
      //             obstacle.primitive_poses[0].position.x,
      //             obstacle.primitive_poses[0].position.y,
      //             obstacle.primitive_poses[0].position.z);

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

      // Save trajectory to dataset
      saveTrajectory(joint_target, obstacles, plan, plan.trajectory_.multi_dof_joint_trajectory.points[0].time_from_start);
      RCLCPP_INFO(logger, "Saved trajectory %d to dataset", trajectory_id - 1);

      successes++;

    } else {

      RCLCPP_ERROR(logger, "Planning failed or timed out");
      
    }


    // Collect obstacle IDs during creation
    std::vector<std::string> obstacle_ids;
    for (const auto& obstacle : obstacles) {
        obstacle_ids.push_back(obstacle.id);
    }

    // Remove all obstacles
    planning_scene_interface.removeCollisionObjects(obstacle_ids);

    /* -------------------------------------------------------------------------------------------------------- */

  }

  // Save to file
  std::ofstream outfile(DATASET_FILE);
  outfile << dataset.dump(2);
  outfile.close();
  RCLCPP_INFO(logger, "Dataset collection complete. Total trajectories: %d", 
              successes);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;

}