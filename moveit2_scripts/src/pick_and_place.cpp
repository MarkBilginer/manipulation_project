// pick_and_place.cpp
// ROS 2 Humble + MoveIt 2 + Gazebo Classic

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <algorithm>
#include <chrono> // for sleep_for durations
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

// Adjust only if SRDF uses different names
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
// Robotiq 85 driven joint (with mimics). Change if different.
static const std::string GRIPPER_DRIVE_JOINT = "robotiq_85_left_knuckle_joint";

class PickAndPlaceTrajectory {
public:
  // parameter name without underscore; assign into member base_node_
  explicit PickAndPlaceTrajectory(const rclcpp::Node::SharedPtr &base_node)
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
      RCLCPP_INFO(LOGGER, "Creating MoveGroupInterface for '%s'…",
                  PLANNING_GROUP_ROBOT.c_str());
      mg_robot_ = std::make_shared<MoveGroupInterface>(base_node_,
                                                       PLANNING_GROUP_ROBOT);

      RCLCPP_INFO(LOGGER, "Creating MoveGroupInterface for '%s'…",
                  PLANNING_GROUP_GRIPPER.c_str());
      mg_gripper_ = std::make_shared<MoveGroupInterface>(
          base_node_, PLANNING_GROUP_GRIPPER);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "MoveGroupInterface construction failed: %s",
                   e.what());
      throw;
    }

    // Always plan in a fixed world frame for repeatability
    mg_robot_->setPoseReferenceFrame("world");

    // Determinism knobs
    mg_robot_->setNumPlanningAttempts(1); // avoid plan-to-plan variation
    mg_robot_->setPlannerId(
        "RRTConnectkConfigDefault"); // pick a specific OMPL planner
    mg_robot_->setPlanningTime(5.0); // enough time but not huge

    // Conservative speed and goal tolerances for safety/robustness.
    mg_robot_->setMaxVelocityScalingFactor(0.30);
    mg_robot_->setMaxAccelerationScalingFactor(0.30);
    mg_robot_->setGoalPositionTolerance(0.01);    // 2 mm
    mg_robot_->setGoalOrientationTolerance(0.01); // ~0.057°
    mg_robot_->setGoalJointTolerance(0.01);       // ~0.17°

    mg_gripper_->setMaxVelocityScalingFactor(0.3);
    mg_gripper_->setMaxAccelerationScalingFactor(0.3);
    mg_gripper_->setGoalJointTolerance(0.005); // tighter tolerance for reliable gripping


    RCLCPP_INFO(
        LOGGER,
        "[Cfg] Using frame='world', planner='RRTConnectkConfigDefault', "
        "attempts=1, planning_time=5.0s");

    // Basic info
    RCLCPP_INFO(LOGGER, "Planning frame : %s",
                mg_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "EEF link       : %s",
                mg_robot_->getEndEffectorLink().c_str());

    auto groups = mg_robot_->getJointModelGroupNames();
    RCLCPP_INFO(LOGGER, "Available planning groups (%zu):", groups.size());
    for (const auto &g : groups)
      RCLCPP_INFO(LOGGER, "  - %s", g.c_str());

    // Fetch current state (wait up to 5s)
    RCLCPP_INFO(LOGGER, "Waiting for current state (up to 5.0s)...");
    auto state = mg_robot_->getCurrentState(5.0);
    if (!state) {
      RCLCPP_WARN(LOGGER, "No current state received. Is the sim running and "
                          "/joint_states publishing?");
      return;
    }
    RCLCPP_INFO(LOGGER, "Current state received.");

    // Print a few variables for sanity
    const auto &var_names = state->getVariableNames();
    RCLCPP_INFO(LOGGER,
                "Robot has %zu variables. First up to 10:", var_names.size());
    for (size_t i = 0; i < std::min<size_t>(10, var_names.size()); ++i) {
      RCLCPP_INFO(LOGGER, "  [%02zu] %s = %.6f", i, var_names[i].c_str(),
                  state->getVariablePosition(var_names[i]));
    }

    // Print the gripper drive joint position
    double gpos;
    if (readJoint(*state, GRIPPER_DRIVE_JOINT, gpos)) {
      RCLCPP_INFO(LOGGER, "[Gripper] %s = %.6f rad",
                  GRIPPER_DRIVE_JOINT.c_str(), gpos);
    } else {
      RCLCPP_WARN(LOGGER,
                  "[Gripper] Joint '%s' not found in RobotState — check joint "
                  "name/SRDF.",
                  GRIPPER_DRIVE_JOINT.c_str());
    }

    RCLCPP_INFO(LOGGER, "STEP 1 complete. (No motion yet.)");
  }

  ~PickAndPlaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Shutting down PickAndPlaceTrajectory.");
  }

  // one safe motion to a known joint configuration
  bool goHome() {
    RCLCPP_INFO(LOGGER, "[Step] Go Home (joint goal)");
    // UR-style joint order: shoulder_pan, shoulder_lift, elbow, wrist1, wrist2,
    // wrist3 This is a neutral pose you can adjust later.
    setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708,
                             +0.0000);

    return planAndExecKinematics();
  }

  // Absolute pre-grasp pose
  // Example: above a cube with tool Z pointing down (180° about X: q = [-1, 0,
  // 0, 0]).
  bool goToPregrasp(float x, float y, float z, float qx, float qy, float qz,
                    float qw) {
    RCLCPP_INFO(LOGGER, "[Step] Go to Pregrasp (pose goal)");
    setup_goal_pose_target(x, y, z, qx, qy, qz, qw);
    return planAndExecKinematics();
  }

  // Straight-line approach relative to current EEF pose
  bool approachDelta(float dx, float dy, float dz) {
    RCLCPP_INFO(LOGGER, "[Step] Approach (Cartesian)");
    setup_waypoints_target(dx, dy, dz);
    return planAndExecCartesian();
  }

  bool retreatDelta(float dx, float dy, float dz) {
    RCLCPP_INFO(LOGGER, "[Step] Retreat (Cartesian)");
    setup_waypoints_target(dx, dy, dz);
    return planAndExecCartesian();
  }

  bool openGripper() {
    RCLCPP_INFO(LOGGER, "[Step] Open Gripper");
    setup_named_pose_gripper("gripper_open");
    // keep gripper slow and gentle
    mg_gripper_->setMaxVelocityScalingFactor(0.03);
    mg_gripper_->setMaxAccelerationScalingFactor(0.02);
    return planAndExecGripper();
  }

  bool closeGripperNamed() {
    RCLCPP_INFO(LOGGER, "[Step] Close Gripper (named)");
    setup_named_pose_gripper("gripper_close");
    mg_gripper_->setMaxVelocityScalingFactor(0.03);
    mg_gripper_->setMaxAccelerationScalingFactor(0.02);
    return planAndExecGripper();
  }

  bool turnShoulder180() {
    RCLCPP_INFO(LOGGER, "[Step] Turn shoulder/base by 180°");

    // Get current state
    auto st = mg_robot_->getCurrentState(2.0);
    if (!st) {
      RCLCPP_ERROR(LOGGER, "[Turn180] No current state available.");
      return false;
    }

    const moveit::core::JointModelGroup *jmg =
        st->getJointModelGroup(PLANNING_GROUP_ROBOT);
    std::vector<double> q;
    st->copyJointGroupPositions(jmg, q);

    // assume base/shoulder is index 0
    q[0] += M_PI; // add 180 degrees

    mg_robot_->setJointValueTarget(q);

    mg_robot_->setMaxVelocityScalingFactor(0.2);
    mg_robot_->setMaxAccelerationScalingFactor(0.1);

    return planAndExecKinematics();
  }

  void execute_trajectory_plan() {
    using namespace std::chrono_literals;
    RCLCPP_INFO(LOGGER, "=== STEP 5: Close → GoHome → Pregrasp → Open → "
                        "Approach → Incremental Close → Slow Lift ===");

    // Close first (spec requirement; safe start)
    if (!closeGripperNamed()) {
     RCLCPP_ERROR(LOGGER, "Initial Close FAILED");
    return;
    }
    //if (!openGripper()) {
      //RCLCPP_ERROR(LOGGER, "OpenGripper FAILED");
      //return;
    //}

    if (!goHome()) {
      RCLCPP_ERROR(LOGGER, "GoHome FAILED");
      return;
    }

    // Define your desired orientation in RPY (radians)
    double roll = M_PI; // rotation about X
    double pitch = 0.0; // rotation about Y (90°)
    double yaw = 0.0;   // rotation about Z

    // Build quaternion from RPY
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize(); // make sure it's a valid unit quaternion

    // Extract values
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();
    double qw = q.w();

    // Now call your goToPregrasp with these values
    if (!goToPregrasp(+0.343f, 0.132f, +0.260f, (float)qx, (float)qy, (float)qz,
                      (float)qw)) {
      RCLCPP_ERROR(LOGGER, "Pregrasp FAILED");
      return;
    }

    if (!openGripper()) {
      RCLCPP_ERROR(LOGGER, "OpenGripper FAILED");
      return;
    }

    if (!approachDelta(0.0f, 0.0f, -0.050f)) {
      RCLCPP_ERROR(LOGGER, "Approach FAILED");
      return;
    }

    if (!closeGripperRamp(0.55, 0.780, 0.12, 0.08, 0.0001, 0.0003, 20)) {
      RCLCPP_ERROR(LOGGER, "Incremental close FAILED");
      return;
    }

    std::this_thread::sleep_for(1s); // small settle before lift

    // Very slow first lift to avoid kick
    mg_robot_->setMaxVelocityScalingFactor(0.10);
    mg_robot_->setMaxAccelerationScalingFactor(0.05);
    if (!retreatDelta(0.0f, 0.0f, +0.085f)) {
      RCLCPP_ERROR(LOGGER, "Retreat FAILED");
      return;
    }

    if (!turnShoulder180())
      return;

    std::this_thread::sleep_for(200ms); // small settle

    if (!openGripper())
      return;

    // Restore conservative speed
    mg_robot_->setMaxVelocityScalingFactor(0.30);
    mg_robot_->setMaxAccelerationScalingFactor(0.30);

    RCLCPP_INFO(LOGGER, "DONE");
  }

private:
  bool readJoint(const moveit::core::RobotState &st, const std::string &name,
                 double &out) {
    const auto &names = st.getVariableNames();
    auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end())
      return false;
    out = st.getVariablePosition(name);
    return true;
  }

  // Make sure we always start planning from a clean state.
  // - stop any residual motion
  // - clear stale pose targets / constraints
  // - snapshot the CURRENT robot state as planner start
  void prepareForNewGoal(bool clear_pose_targets = true) {
    mg_robot_->stop();
    if (clear_pose_targets)
      mg_robot_->clearPoseTargets();
    mg_robot_->clearPathConstraints();
    mg_robot_->setStartStateToCurrentState();
  }

  // Set a 6-DoF joint target (UR-style order).
  // Adds an info print so you can verify exactly what was sent.
  void setup_joint_value_target(float a0, float a1, float a2, float a3,
                                float a4, float a5) {
    prepareForNewGoal(true);
    std::vector<double> q = {a0, a1, a2, a3, a4, a5};
    mg_robot_->setJointValueTarget(q);
    RCLCPP_INFO(LOGGER,
                "[Target] Joint goal set: [%.4f %.4f %.4f %.4f %.4f %.4f]", a0,
                a1, a2, a3, a4, a5);
  }

  // Plan a kinematics trajectory and execute it.
  // We log both the planning result and the execution result.
  bool planAndExecKinematics() {
    using namespace std::chrono_literals;
    // Refresh start state right before planning (in case joints changed)
    mg_robot_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned =
        (mg_robot_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_WARN(LOGGER, "[Plan] Kinematics plan FAILED");
      return false;
    }
    RCLCPP_INFO(LOGGER, "[Plan] Kinematics plan OK. Executing…");

    auto ret = mg_robot_->execute(plan);
    bool exec_ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, exec_ok ? "[Exec] Robot kinematics EXECUTION SUCCESS"
                                : "[Exec] Robot kinematics EXECUTION FAILED");
    // to settle after execution of motion
    if (exec_ok)
      std::this_thread::sleep_for(std::chrono::milliseconds(500ms));

    return exec_ok;
  }

  // Set an absolute pose target for the EEF (MoveIt will IK to joints)
  void setup_goal_pose_target(float x, float y, float z, float qx, float qy,
                              float qz, float qw) {
    // Clear pose targets & constraints, stop motion, set fresh start state
    prepareForNewGoal(/*clear_pose_targets=*/true);

    Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.x = qx;
    p.orientation.y = qy;
    p.orientation.z = qz;
    p.orientation.w = qw;

    mg_robot_->setPoseTarget(p);
    RCLCPP_INFO(
        LOGGER,
        "[Target] Pose set: pos(%.3f, %.3f, %.3f) quat(%.3f, %.3f, %.3f, %.3f)",
        x, y, z, qx, qy, qz, qw);
  }

  // Build a straight-line waypoint segment relative to current EEF pose
  void setup_waypoints_target(float dx, float dy, float dz) {
    // Same hygiene before sampling current pose
    prepareForNewGoal(/*clear_pose_targets=*/true);

    Pose start = mg_robot_->getCurrentPose().pose;
    waypoints_.clear();
    waypoints_.push_back(start);

    Pose goal = start;
    goal.position.x += dx;
    goal.position.y += dy;
    goal.position.z += dz;
    waypoints_.push_back(goal);

    RCLCPP_INFO(LOGGER, "[Waypoints] Δ = (%.3f, %.3f, %.3f)", dx, dy, dz);
  }

  // Compute and execute a Cartesian path; print the achieved fraction
  bool planAndExecCartesian() {
    mg_robot_->setStartStateToCurrentState();
    cart_fraction_ = mg_robot_->computeCartesianPath(
        waypoints_, eef_step_, jump_threshold_, cart_traj_);

    RCLCPP_INFO(LOGGER, "[Cartesian] path fraction = %.3f (need ≥ 0.98)",
                cart_fraction_);
    waypoints_.clear();

    if (cart_fraction_ < 0.98) {
      RCLCPP_WARN(LOGGER, "[Cartesian] fraction too low; aborting execute.");
      return false;
    }

    // Time-parameterize with TOTG so the controller gets a proper, smooth
    // trajectory. Pick conservative scalings that match how you’re moving
    // during approach/retreat.
    robot_trajectory::RobotTrajectory rt(mg_robot_->getRobotModel(),
                                         PLANNING_GROUP_ROBOT);
    rt.setRobotTrajectoryMsg(*mg_robot_->getCurrentState(), cart_traj_);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    const double vel_scale =
        0.20; // set to 0.15 for your approach, 0.10–0.20 for retreat
    const double acc_scale = 0.10; // keep low to avoid slip on pickup
    if (!totg.computeTimeStamps(rt, vel_scale, acc_scale)) {
      RCLCPP_WARN(
          LOGGER,
          "[Cartesian] TOTG failed; executing un-timed path may be jerky.");
    } else {
      rt.getRobotTrajectoryMsg(cart_traj_);
    }

    auto ret = mg_robot_->execute(cart_traj_);
    bool ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, ok ? "[Exec] Cartesian EXECUTION SUCCESS"
                           : "[Exec] Cartesian EXECUTION FAILED");
    return ok;
  }

  // --- Gripper planning hygiene (mirrors the arm helpers) ---
  void prepareForNewGoalGripper(bool clear_pose_targets = true) {
    mg_gripper_->stop();
    if (clear_pose_targets)
      mg_gripper_->clearPoseTargets();
    mg_gripper_->clearPathConstraints();
    mg_gripper_->setStartStateToCurrentState();
  }

  // Set a named gripper target (e.g., "gripper_open", "gripper_close")
  void setup_named_pose_gripper(const std::string &pose_name) {
    prepareForNewGoalGripper(true);
    mg_gripper_->setNamedTarget(pose_name);
    RCLCPP_INFO(LOGGER, "[Target] Gripper named target: %s", pose_name.c_str());
  }

  // Plan+execute for the gripper group with clear prints
  bool planAndExecGripper() {
    using namespace std::chrono_literals;
    mg_gripper_->setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned =
        (mg_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_WARN(LOGGER, "[Plan] Gripper plan FAILED");
      return false;
    }
    RCLCPP_INFO(LOGGER, "[Plan] Gripper plan OK. Executing…");
    auto ret = mg_gripper_->execute(plan);
    bool ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, ok ? "[Exec] Gripper EXECUTION SUCCESS"
                           : "[Exec] Gripper EXECUTION FAILED");
    if (ok)
      std::this_thread::sleep_for(std::chrono::milliseconds(500ms));

    return ok;
  }

  // Command the driven gripper joint
  void setup_joint_value_gripper(double pos) {
    prepareForNewGoalGripper(true);
    std::map<std::string, double> targets;
    targets[GRIPPER_DRIVE_JOINT] = pos;
    mg_gripper_->setJointValueTarget(targets);
    RCLCPP_INFO(LOGGER, "[Target] Gripper drive joint %.5f rad", pos);
  }

  double getGripperPosition() {
    auto st = mg_gripper_->getCurrentState(1.0);
    if (!st)
      return std::numeric_limits<double>::quiet_NaN();
    return st->getVariablePosition(GRIPPER_DRIVE_JOINT);
  }

  bool planExecGripperTo(double pos, int settle_ms, double vel_scale,
                         double acc_scale, bool log_progress = false,
                         int poll_ms = 50) {
    prepareForNewGoalGripper(true);
    mg_gripper_->setStartStateToCurrentState(); // ensure fresh start
    mg_gripper_->setMaxVelocityScalingFactor(vel_scale);
    mg_gripper_->setMaxAccelerationScalingFactor(acc_scale);
    mg_gripper_->setJointValueTarget({{GRIPPER_DRIVE_JOINT, pos}});

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg_gripper_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(LOGGER, "[GripperRamp] plan failed pos=%.5f", pos);
      return false;
    }

    // Async execution so we can optionally poll/log
    auto exec_future = std::async(std::launch::async,
                                  [&]() { return mg_gripper_->execute(plan); });

    if (log_progress) {
      while (exec_future.wait_for(std::chrono::milliseconds(poll_ms)) ==
             std::future_status::timeout) {
        double now = getGripperPosition();
        RCLCPP_INFO(LOGGER, "[GripperProgress] now=%.5f", now);
      }
    }

    if (exec_future.get() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(LOGGER, "[Gripper] exec failed pos=%.5f", pos);
      return false;
    }

    if (settle_ms > 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));

    return true;
  }

  bool closeGripperRamp(double start_pos, double goal_pos,
                        double fast_vel = 0.10, double fast_acc = 0.06,
                        double slow_vel = 0.03, double slow_acc = 0.02,
                        int settle_ms = 20) {
    start_pos = std::min(start_pos, goal_pos);

    // --- Step1: one fast motion to start_pos
    if (!planExecGripperTo(start_pos, settle_ms, fast_vel, fast_acc))
      return false;

    // --- Step2:slow creep
    if (!planExecGripperTo(goal_pos, settle_ms, slow_vel, slow_acc, true))
      return false;

    double now = getGripperPosition();
    RCLCPP_INFO(LOGGER, "[CloseSlow] reached gripper position=%.5f", now);

    return true;
  }

  // keep a handle to the single node we’re reusing
  rclcpp::Node::SharedPtr base_node_;

  rclcpp::executors::SingleThreadedExecutor executor_;

  std::shared_ptr<MoveGroupInterface> mg_robot_;
  std::shared_ptr<MoveGroupInterface> mg_gripper_;

  using Pose = geometry_msgs::msg::Pose;
  std::vector<Pose> waypoints_;                 // for Cartesian paths
  moveit_msgs::msg::RobotTrajectory cart_traj_; // computed Cartesian trajectory

  double cart_fraction_ = 0.0;        // [0..1]
  const double jump_threshold_ = 0.0; // disable jump detection
  const double eef_step_ = 0.001;     // 1 mm step for interpolation
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto base_node = std::make_shared<rclcpp::Node>("pick_and_place_trajectory");
  PickAndPlaceTrajectory app(base_node);

  // run once, then exit
  app.execute_trajectory_plan();

  rclcpp::shutdown();
  return 0;
}
