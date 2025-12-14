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
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

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
auto csv_path = share + "/data/waypoint_triangle.csv";
std::string csv_path = node->declare_parameter<std::string>("csv_path", "");
if (csv_path.empty()) {
RCLCPP_ERROR(node->get_logger(), "Paramètre 'csv_path' vide. Exemple: --ros-args -p csv_path:=/chemin/triangle.csv");
return 1;
}

try {
  auto points2d = load_uvpen_csv(csv_path);
  RCLCPP_INFO(node->get_logger(), "Chargé %zu points depuis %s", points2d.size(), csv_path.c_str());

  // Exemple: afficher les 3 premiers
  for (size_t i = 0; i < std::min<size_t>(points2d.size(), 3); ++i) {
    RCLCPP_INFO(node->get_logger(), "[%zu] u=%.4f v=%.4f pen=%d",
                i, points2d[i].u, points2d[i].v, points2d[i].pen ? 1 : 0);
  }
} catch (const std::exception& e) {
  RCLCPP_ERROR(node->get_logger(), "Erreur lecture CSV: %s", e.what());
  return 1;
}

  }