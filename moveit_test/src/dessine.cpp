#include <memory>
#include <thread>
#include <chrono>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


/*
============================================================
Description :
Ce programme contrôle un UR5 à l’aide de ROS 2 et de MoveIt afin d’exécuter 
une trajectoire cartésienne définie par une série de poses. Il charge et traite des données de trajectoire, 
segmente le mouvement en portions successives, planifie chaque segment et les exécute. 

Auteur :
Binder Aurore, Schmitt Théo, Hatton Axel
INSA de Strasbourg - 5ème année

Dernière modification :
2024-09-10

============================================================
*/



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
    // Ignore lignes vides / commentaires
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
      // trim simple (espaces)
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

int main(int argc, char * argv[])
{
  // Initialisation de ROS2
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  // -----------------------------
  //   Executor pour faire tourner le node
  // -----------------------------
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // Interface MoveIt
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "ur_manipulator");

  move_group_interface.setPoseReferenceFrame("base_link");
  move_group_interface.setPlanningTime(5.0);
  move_group_interface.setMaxVelocityScalingFactor(0.02);
  move_group_interface.setMaxAccelerationScalingFactor(0.02);

  // Laisser un peu de temps pour que le state monitor reçoive les premiers joint_states
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
  "EE pose: x=%.3f y=%.3f z=%.3f x=%.3f y=%.3f z=%.3f w=%.3f",
  pose.pose.position.x,
  pose.pose.position.y,
  pose.pose.position.z,
  pose.pose.orientation.x,
  pose.pose.orientation.y,
  pose.pose.orientation.z,
  pose.pose.orientation.w);
  
  // Import des points du csv :

auto share = ament_index_cpp::get_package_share_directory("moveit_test");
auto csv_path = share + "/data/output.csv";
if (csv_path.empty()) {
RCLCPP_ERROR(node->get_logger(), "Paramètre 'csv_path' vide. Exemple: --ros-args -p csv_path:=/chemin/triangle.csv");
return 1;
}

std::vector<UVPen> points2d;

try {
  points2d = load_uvpen_csv(csv_path);
  RCLCPP_INFO(node->get_logger(), "Chargé %zu points depuis %s", points2d.size(), csv_path.c_str());

  // Exemple: afficher les 3 premiers
  for (size_t i = 0; i < std::min<size_t>(points2d.size(), 3); ++i) {
    RCLCPP_INFO(node->get_logger(), "[%zu] u=%.4f v=%.4f pen=%d",
                i, points2d[i].u, points2d[i].v, points2d[i].pen ? 1 : 0);
  }
} catch (const std::exception& e) {
  RCLCPP_ERROR(node->get_logger(), "Erreur lecture CSV: %s", e.what());
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 1;
}

RCLCPP_INFO(node->get_logger(), "Dodo");
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  // definir le départ du prochain plan = position actuel
  move_group_interface.setStartStateToCurrentState();

  // -----------------------------
  //  Waypoints à partir de la pose courante
  // -----------------------------
const std::size_t chunk_size = 25;
const double eef_step = 0.01;
const double jump_thresh = 0.0;

const double pen_lift = -0.01; // 1 cm en mètres

// Pose d'ancrage : origine du dessin (u=0, v=0)
const geometry_msgs::msg::Pose anchor_pose = move_group_interface.getCurrentPose().pose;

for (std::size_t chunk_begin = 0; chunk_begin < points2d.size(); chunk_begin += chunk_size)
{
  const std::size_t chunk_end = std::min(chunk_begin + chunk_size, points2d.size());

  move_group_interface.setStartStateToCurrentState();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve((chunk_end - chunk_begin) + 1);

  // Continuité : départ = pose courante
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;
  waypoints.push_back(start_pose);

  for (std::size_t i = chunk_begin; i < chunk_end; ++i)
  {
    geometry_msgs::msg::Pose p = anchor_pose;

    // Coordonnées absolues du dessin
    p.position.x += points2d[i].u;
    p.position.z += points2d[i].v;

    // Orientation fixe (souvent mieux pour dessiner)
    p.orientation = anchor_pose.orientation;

    // Stylo levé si pen == 0
    if (!points2d[i].pen) {
      p.position.y += pen_lift;   // <-- AXE À ADAPTER si besoin
    }

    RCLCPP_INFO(logger, "[%zu] u=%.4f v=%.4f pen=%d",
                i, points2d[i].u, points2d[i].v, points2d[i].pen ? 1 : 0);

    waypoints.push_back(p);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(
      waypoints, eef_step, jump_thresh, trajectory);

  RCLCPP_INFO(logger, "Segment [%zu..%zu) fraction : %.1f%%",
              chunk_begin, chunk_end, fraction * 100.0);

  if (fraction > 0.95)
  {
    auto exec_result = move_group_interface.execute(trajectory);
    if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(logger, "Échec exécution segment [%zu..%zu). Arrêt.", chunk_begin, chunk_end);
      break;
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Échec planif segment [%zu..%zu) : %.1f%%. Arrêt.",
                 chunk_begin, chunk_end, fraction * 100.0);
    break;
  }
}


  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;

  }