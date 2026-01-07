#include <memory>
#include <thread>
#include <chrono>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>   // std::fabs
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Retiming
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_parabolic_time_parameterization.h>

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

    // Ignore l'en-tête "u,v,pen"
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

  // Petite pause (laisser tout se stabiliser)
  RCLCPP_INFO(logger, "Pause avant démarrage du dessin...");
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  move_group_interface.setStartStateToCurrentState();

  // -----------------------------
  // Réduire la charge de communication + retiming
  // -----------------------------
  // Chunks petits => goals plus petits (souvent mieux sur contrôleurs fragiles)
  const std::size_t chunk_size = 12;

  // eef_step plus grand => moins de points interpolés => trajectoires plus légères
  const double eef_step = 0.02;    // 2 cm
  const double jump_thresh = 0.0;

  // Pen lift conservé comme demandé
  const double pen_lift = -0.01;

  // Retiming (très conservateur)
  const double retime_vel_scale = 0.02;
  const double retime_accel_scale = 0.01;

  // Désactive le spam de logs point par point
  const bool verbose_points = false;

  // Pose d'ancrage (origine du dessin)
  const geometry_msgs::msg::Pose anchor_pose = move_group_interface.getCurrentPose().pose;

  auto near = [](double a, double b, double eps){ return std::fabs(a-b) < eps; };

  for (std::size_t chunk_begin = 0; chunk_begin < points2d.size(); chunk_begin += chunk_size)
  {
    const std::size_t chunk_end = std::min(chunk_begin + chunk_size, points2d.size());

    move_group_interface.setStartStateToCurrentState();

    // Waypoints du chunk
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve((chunk_end - chunk_begin) + 1);

    // Pose courante
    geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

    // Construire les waypoints
    for (std::size_t i = chunk_begin; i < chunk_end; ++i)
    {
      geometry_msgs::msg::Pose p = anchor_pose;

      // Coordonnées absolues du dessin
      p.position.x += points2d[i].u;
      p.position.z += points2d[i].v;

      // Orientation fixe
      p.orientation = anchor_pose.orientation;

      // Stylo levé si pen == 0
      if (!points2d[i].pen) {
        p.position.y += pen_lift;
      }

      if (verbose_points) {
        RCLCPP_INFO(logger, "[%zu] u=%.4f v=%.4f pen=%d",
                    i, points2d[i].u, points2d[i].v, points2d[i].pen ? 1 : 0);
      }

      waypoints.push_back(p);
    }

    if (waypoints.empty()) {
      continue;
    }

    // Éviter d’ajouter start_pose si le 1er waypoint est déjà quasi identique
    const auto& first = waypoints.front();
    const double eps_pos = 1e-4; // 0.1 mm
    const bool same_start =
      near(first.position.x, start_pose.position.x, eps_pos) &&
      near(first.position.y, start_pose.position.y, eps_pos) &&
      near(first.position.z, start_pose.position.z, eps_pos);

    if (!same_start) {
      waypoints.insert(waypoints.begin(), start_pose);
    }

    // Planification cartésienne
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_interface.computeCartesianPath(
      waypoints, eef_step, jump_thresh, trajectory);

    RCLCPP_INFO(logger, "Segment [%zu..%zu) fraction : %.1f%%",
                chunk_begin, chunk_end, fraction * 100.0);

    if (fraction <= 0.95) {
      RCLCPP_ERROR(logger, "Échec planif segment [%zu..%zu) : %.1f%%. Arrêt.",
                   chunk_begin, chunk_end, fraction * 100.0);
      break;
    }

    // Retiming systématique
    if (!retimeTrajectory(move_group_interface, logger, trajectory,
                          retime_vel_scale, retime_accel_scale))
    {
      RCLCPP_ERROR(logger, "Retiming échoué sur segment [%zu..%zu). Arrêt.", chunk_begin, chunk_end);
      break;
    }

    // Exécution
    auto exec_result = move_group_interface.execute(trajectory);
    if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(logger, "Échec exécution segment [%zu..%zu). Arrêt.", chunk_begin, chunk_end);
      break;
    }

    // Petite pause pour réduire le stress de la chaîne de contrôle
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
