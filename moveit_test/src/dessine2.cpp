#include <memory>
#include <thread>
#include <chrono>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Retiming
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

struct UVPen {
  double u{0.0};
  double v{0.0};
  bool pen{false};
};

static std::vector<UVPen> load_uvpen_csv(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Impossible d'ouvrir le CSV: " + path);
  }

  std::vector<UVPen> pts;
  std::string line;
  bool first_line = true;

  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    if (first_line) {
      first_line = false;
      if (line.find("u") != std::string::npos && line.find("pen") != std::string::npos) {
        continue;
      }
    }

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> cols;
    while (std::getline(ss, cell, ',')) {
      auto start = cell.find_first_not_of(" \t\r");
      auto end   = cell.find_last_not_of(" \t\r");
      cols.push_back((start == std::string::npos) ? "" : cell.substr(start, end - start + 1));
    }
    if (cols.size() < 3) continue;

    UVPen p;
    p.u = std::stod(cols[0]);
    p.v = std::stod(cols[1]);
    p.pen = (std::stoi(cols[2]) != 0);
    pts.push_back(p);
  }

  if (pts.empty()) {
    throw std::runtime_error("CSV vide ou illisible: " + path);
  }
  return pts;
}

// Retiming systématique avant execute()
static bool retimeTrajectory(
  const moveit::planning_interface::MoveGroupInterface& mgi,
  const rclcpp::Logger& logger,
  moveit_msgs::msg::RobotTrajectory& traj_msg,
  double vel_scale,
  double accel_scale)
{
  auto current_state_ptr = mgi.getCurrentState(1.0);
  if (!current_state_ptr) {
    RCLCPP_ERROR(logger, "Retiming: impossible d'obtenir l'état courant.");
    return false;
  }

  robot_trajectory::RobotTrajectory rt(mgi.getRobotModel(), mgi.getName());
  rt.setRobotTrajectoryMsg(*current_state_ptr, traj_msg);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, vel_scale, accel_scale)) {
    RCLCPP_ERROR(logger, "Retiming: computeTimeStamps() a échoué.");
    return false;
  }

  rt.getRobotTrajectoryMsg(traj_msg);
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  // Executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // MoveIt
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "ur_manipulator");

  move_group_interface.setPoseReferenceFrame("base_link");
  move_group_interface.setPlanningTime(5.0);

  // Facteurs MoveIt (conservateurs)
  move_group_interface.setMaxVelocityScalingFactor(0.02);
  move_group_interface.setMaxAccelerationScalingFactor(0.02);

  // Attendre l'état courant
  RCLCPP_INFO(logger, "Attente de l'état courant du robot...");
  auto current_state = move_group_interface.getCurrentState(10.0);
  if (!current_state) {
    RCLCPP_ERROR(logger, "Impossible de récupérer l'état courant du robot (timeout).");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "État courant du robot reçu.");

  auto pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger,
    "EE pose: x=%.3f y=%.3f z=%.3f qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z,
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w
  );

  // Charger CSV
  auto share = ament_index_cpp::get_package_share_directory("moveit_test");
  auto csv_path = share + "/data/output.csv";
  if (csv_path.empty()) {
    RCLCPP_ERROR(logger, "Chemin CSV vide.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  std::vector<UVPen> points2d;
  try {
    points2d = load_uvpen_csv(csv_path);
    RCLCPP_INFO(logger, "Chargé %zu points depuis %s", points2d.size(), csv_path.c_str());
    for (size_t i = 0; i < std::min<size_t>(points2d.size(), 3); ++i) {
      RCLCPP_INFO(logger, "[%zu] u=%.4f v=%.4f pen=%d",
                  i, points2d[i].u, points2d[i].v, points2d[i].pen ? 1 : 0);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Erreur lecture CSV: %s", e.what());
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // Pause
  RCLCPP_INFO(logger, "Pause avant démarrage du dessin...");
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  move_group_interface.setStartStateToCurrentState();

  // -----------------------------
  //  Réglages "CB3-safe" + séparation pen-up/pen-down
  // -----------------------------
  const double pen_lift = -0.01;             // demandé (ne pas changer)
  const std::size_t chunk_size = 8;          // goals plus petits
  const double eef_step = 0.03;              // moins de points -> moins de charge
  const double jump_thresh = 0.0;

  const double retime_vel_scale   = 0.02;
  const double retime_accel_scale = 0.01;

  const auto goal_pause = std::chrono::milliseconds(400); // baisse forte des drops

  // Pose d'ancrage (origine du dessin)
  const geometry_msgs::msg::Pose anchor_pose = move_group_interface.getCurrentPose().pose;

  auto near = [](double a, double b, double eps){ return std::fabs(a-b) < eps; };
  const double eps_pos = 2e-4; // 0.2 mm

  auto samePosition = [&](const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b){
    return near(a.position.x, b.position.x, eps_pos) &&
           near(a.position.y, b.position.y, eps_pos) &&
           near(a.position.z, b.position.z, eps_pos);
  };

  auto pose_from_uvpen = [&](const UVPen& pt) {
    geometry_msgs::msg::Pose p = anchor_pose;
    p.position.x += pt.u;
    p.position.z += pt.v;
    p.orientation = anchor_pose.orientation;

    if (!pt.pen) {
      p.position.y += pen_lift;
    }
    return p;
  };

  // Exécuter un petit move cartésien (2 poses) + retiming
  auto execute_cartesian_to = [&](const geometry_msgs::msg::Pose& target) -> bool {
    move_group_interface.setStartStateToCurrentState();

    geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;
    if (samePosition(start_pose, target)) {
      return true;
    }

    std::vector<geometry_msgs::msg::Pose> wps;
    wps.reserve(2);
    wps.push_back(start_pose);
    wps.push_back(target);

    moveit_msgs::msg::RobotTrajectory traj;
    double frac = move_group_interface.computeCartesianPath(wps, eef_step, jump_thresh, traj);
    if (frac < 0.95) {
      RCLCPP_ERROR(logger, "Move court: computeCartesianPath fraction=%.1f%%", frac * 100.0);
      return false;
    }

    if (!retimeTrajectory(move_group_interface, logger, traj, retime_vel_scale, retime_accel_scale)) {
      RCLCPP_ERROR(logger, "Move court: retiming échoué");
      return false;
    }

    auto res = move_group_interface.execute(traj);
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Move court: execute échoué");
      return false;
    }

    std::this_thread::sleep_for(goal_pause);
    return true;
  };

  // -----------------------------
  //  Boucle : traits pen-down séparés, pen-up en petits moves
  // -----------------------------
  std::size_t idx = 0;
  while (idx < points2d.size())
  {
    // Avancer jusqu'au prochain pen==1
    while (idx < points2d.size() && !points2d[idx].pen) {
      ++idx;
    }
    if (idx >= points2d.size()) break;

    const std::size_t stroke_begin = idx;

    // Trouver fin du trait pen-down
    while (idx < points2d.size() && points2d[idx].pen) {
      ++idx;
    }
    const std::size_t stroke_end = idx; // [stroke_begin, stroke_end)

    RCLCPP_INFO(logger, "Trait pen-down: [%zu..%zu) (%zu pts)",
                stroke_begin, stroke_end, stroke_end - stroke_begin);

    // Aller au début du trait en pen-up, puis descendre en pen-down
    {
      geometry_msgs::msg::Pose start_down = pose_from_uvpen(points2d[stroke_begin]);
      geometry_msgs::msg::Pose start_up = start_down;
      start_up.position.y += pen_lift; // forcer stylo levé

      if (!execute_cartesian_to(start_up)) {
        RCLCPP_ERROR(logger, "Arrêt: impossible d'aller au début du trait (pen up).");
        break;
      }
      if (!execute_cartesian_to(start_down)) {
        RCLCPP_ERROR(logger, "Arrêt: impossible de descendre au début du trait (pen down).");
        break;
      }
    }

    // Exécuter le trait en chunks pen-down
    for (std::size_t chunk_begin = stroke_begin; chunk_begin < stroke_end; chunk_begin += chunk_size)
    {
      const std::size_t chunk_end = std::min(chunk_begin + chunk_size, stroke_end);

      move_group_interface.setStartStateToCurrentState();

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.reserve((chunk_end - chunk_begin) + 1);

      geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;
      waypoints.push_back(start_pose);

      for (std::size_t k = chunk_begin; k < chunk_end; ++k)
      {
        geometry_msgs::msg::Pose p = pose_from_uvpen(points2d[k]); // ici pen==1
        waypoints.push_back(p);
      }

      moveit_msgs::msg::RobotTrajectory trajectory;
      const double fraction =
        move_group_interface.computeCartesianPath(waypoints, eef_step, jump_thresh, trajectory);

      RCLCPP_INFO(logger, "  Segment trait [%zu..%zu) fraction: %.1f%%",
                  chunk_begin, chunk_end, fraction * 100.0);

      if (fraction < 0.95) {
        RCLCPP_ERROR(logger, "Arrêt: échec planif segment trait [%zu..%zu).", chunk_begin, chunk_end);
        goto END_DRAW;
      }

      if (!retimeTrajectory(move_group_interface, logger, trajectory,
                            retime_vel_scale, retime_accel_scale))
      {
        RCLCPP_ERROR(logger, "Arrêt: retiming échoué segment trait [%zu..%zu).", chunk_begin, chunk_end);
        goto END_DRAW;
      }

      auto exec_result = move_group_interface.execute(trajectory);
      if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(logger, "Arrêt: execute échoué segment trait [%zu..%zu).", chunk_begin, chunk_end);
        goto END_DRAW;
      }

      std::this_thread::sleep_for(goal_pause);
    }

    // Lever le stylo en fin de trait
    {
      geometry_msgs::msg::Pose end_down = pose_from_uvpen(points2d[stroke_end - 1]);
      geometry_msgs::msg::Pose end_up = end_down;
      end_up.position.y += pen_lift;
      if (!execute_cartesian_to(end_up)) {
        RCLCPP_ERROR(logger, "Arrêt: impossible de lever le stylo en fin de trait.");
        break;
      }
    }
  }

END_DRAW:
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
