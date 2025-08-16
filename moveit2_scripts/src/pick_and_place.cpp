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

    // STEP 2: one safe motion to a known joint configuration
    bool goHome() {
        RCLCPP_INFO(LOGGER, "[Step] Go Home (joint goal)");
        // UR-style joint order: shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3
        // This is a neutral pose you can adjust later.
        setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708, +0.0000);

        // Conservative speed and goal tolerances for safety/robustness.
        mg_robot_->setMaxVelocityScalingFactor(0.30);
        mg_robot_->setMaxAccelerationScalingFactor(0.30);
        mg_robot_->setGoalJointTolerance(0.005);      // ≈ 0.29°
        mg_robot_->setGoalPositionTolerance(0.005);   // 5 mm (mainly for pose goals)
        mg_robot_->setGoalOrientationTolerance(0.02); // ~1.15°

        return planAndExecKinematics();
    }

    // One-shot runner for STEP 2
    void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "=== STEP 2: executing single GoHome motion ===");
    if (!goHome()) {
        RCLCPP_ERROR(LOGGER, "GoHome FAILED");
        return;
    }
    RCLCPP_INFO(LOGGER, "GoHome DONE");
    }


private:
    bool readJoint(const moveit::core::RobotState& st, const std::string& name, double& out) {
    const auto& names = st.getVariableNames();
    auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end()) return false;
    out = st.getVariablePosition(name);
    return true;
    }

    // Make sure we always start planning from a clean state.
    // - stop any residual motion
    // - clear stale pose targets / constraints
    // - snapshot the CURRENT robot state as planner start
    void prepareForNewGoal(bool clear_pose_targets = true) {
        mg_robot_->stop();
        if (clear_pose_targets) mg_robot_->clearPoseTargets();
        mg_robot_->clearPathConstraints();
        mg_robot_->setStartStateToCurrentState();
    }

    // Set a 6-DoF joint target (UR-style order).
    // Adds an info print so you can verify exactly what was sent.
    void setup_joint_value_target(float a0, float a1, float a2, float a3, float a4, float a5) {
    prepareForNewGoal(true);
    std::vector<double> q = {a0, a1, a2, a3, a4, a5};
    mg_robot_->setJointValueTarget(q);
    RCLCPP_INFO(LOGGER, "[Target] Joint goal set: [%.4f %.4f %.4f %.4f %.4f %.4f]",
                a0, a1, a2, a3, a4, a5);
    }

    // Plan a kinematics trajectory and execute it.
    // We log both the planning result and the execution result.
    bool planAndExecKinematics() {
    // Refresh start state right before planning (in case joints changed)
    mg_robot_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (mg_robot_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
        RCLCPP_WARN(LOGGER, "[Plan] Kinematics plan FAILED");
        return false;
    }
    RCLCPP_INFO(LOGGER, "[Plan] Kinematics plan OK. Executing…");

    auto ret = mg_robot_->execute(plan);
    bool exec_ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, exec_ok ? "[Exec] Robot kinematics EXECUTION SUCCESS"
                                : "[Exec] Robot kinematics EXECUTION FAILED");
    return exec_ok;
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

    //run once, then exit
    app.execute_trajectory_plan();

    rclcpp::shutdown();
    return 0;
}
