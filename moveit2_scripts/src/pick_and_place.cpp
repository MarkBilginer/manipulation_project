// pick_and_place.cpp — STEP 1: minimal MoveIt wiring with debug prints
// ROS 2 Humble + MoveIt 2 + Gazebo Classic

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <algorithm>
#include <string>
#include <thread>
#include <vector>
#include <limits>

using moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

// Adjust only if SRDF uses different names
static const std::string PLANNING_GROUP_ROBOT   = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
// Robotiq 85 driven joint (with mimics). Change if different.
static const std::string GRIPPER_DRIVE_JOINT    = "robotiq_85_left_knuckle_joint";

class PickAndPlaceTrajectory {
public:
  // parameter name without underscore; assign into member base_node_
  explicit PickAndPlaceTrajectory(const rclcpp::Node::SharedPtr& base_node)
  : base_node_(base_node) {
    RCLCPP_INFO(LOGGER, "STEP 1: booting MoveIt…");

    // do NOT create a second node; reuse base_node_ and spin it
    executor_.add_node(base_node_);
    std::thread([this]() {
      RCLCPP_INFO(LOGGER, "Executor thread started.");
      executor_.spin();
      RCLCPP_INFO(LOGGER, "Executor thread stopped.");
    }).detach();

    // Create MoveIt interfaces ON THE SAME NODE
    try {
      RCLCPP_INFO(LOGGER, "Creating MoveGroupInterface for '%s'…", PLANNING_GROUP_ROBOT.c_str());
      mg_robot_   = std::make_shared<MoveGroupInterface>(base_node_, PLANNING_GROUP_ROBOT);

      RCLCPP_INFO(LOGGER, "Creating MoveGroupInterface for '%s'…", PLANNING_GROUP_GRIPPER.c_str());
      mg_gripper_ = std::make_shared<MoveGroupInterface>(base_node_, PLANNING_GROUP_GRIPPER);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "MoveGroupInterface construction failed: %s", e.what());
      throw;
    }

    // Basic info
    RCLCPP_INFO(LOGGER, "Planning frame : %s", mg_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "EEF link       : %s", mg_robot_->getEndEffectorLink().c_str());

    auto groups = mg_robot_->getJointModelGroupNames();
    RCLCPP_INFO(LOGGER, "Available planning groups (%zu):", groups.size());
    for (const auto& g : groups) RCLCPP_INFO(LOGGER, "  - %s", g.c_str());

    // Fetch current state (wait up to 5s)
    RCLCPP_INFO(LOGGER, "Waiting for current state (up to 5.0s)...");
    auto state = mg_robot_->getCurrentState(5.0);
    if (!state) {
      RCLCPP_WARN(LOGGER, "No current state received. Is the sim running and /joint_states publishing?");
      return;
    }
    RCLCPP_INFO(LOGGER, "Current state received.");

    // Print a few variables for sanity
    const auto& var_names = state->getVariableNames();
    RCLCPP_INFO(LOGGER, "Robot has %zu variables. First up to 10:", var_names.size());
    for (size_t i = 0; i < std::min<size_t>(10, var_names.size()); ++i) {
      RCLCPP_INFO(LOGGER, "  [%02zu] %s = %.6f", i, var_names[i].c_str(),
                  state->getVariablePosition(var_names[i]));
    }

    // Print the gripper drive joint position
    double gpos;
    if (readJoint(*state, GRIPPER_DRIVE_JOINT, gpos)) {
      RCLCPP_INFO(LOGGER, "[Gripper] %s = %.6f rad", GRIPPER_DRIVE_JOINT.c_str(), gpos);
    } else {
      RCLCPP_WARN(LOGGER, "[Gripper] Joint '%s' not found in RobotState — check joint name/SRDF.",
                  GRIPPER_DRIVE_JOINT.c_str());
    }

    RCLCPP_INFO(LOGGER, "STEP 1 complete. (No motion yet.)");
  }

  ~PickAndPlaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Shutting down PickAndPlaceTrajectory.");
  }

private:
  bool readJoint(const moveit::core::RobotState& st, const std::string& name, double& out) {
    const auto& names = st.getVariableNames();
    auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end()) return false;
    out = st.getVariablePosition(name);
    return true;
  }

  // keep a handle to the single node we’re reusing
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<MoveGroupInterface> mg_robot_;
  std::shared_ptr<MoveGroupInterface> mg_gripper_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto base_node = std::make_shared<rclcpp::Node>("pick_and_place_trajectory");
  PickAndPlaceTrajectory app(base_node);

  // Keep process alive while background executor spins
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) rate.sleep();

  rclcpp::shutdown();
  return 0;
}
