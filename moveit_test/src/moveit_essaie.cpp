#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char * argv[])
{
  // Initialisation de ROS2
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Logger ROS
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Interface MoveIt
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "ur_manipulator");

  // -----------------------------
  //    Définition des waypoints
  // -----------------------------
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Point 1
  geometry_msgs::msg::Pose p1;
  p1.orientation.w = 1.0;
  p1.position.x = 0.28;
  p1.position.y = -0.20;
  p1.position.z = 0.50;
  waypoints.push_back(p1);

  // Point 2 (décalé en Y)
  geometry_msgs::msg::Pose p2 = p1;
  p2.position.y -= 0.10;  // -10 cm supplémentaires en Y
  waypoints.push_back(p2);

  // Point 3 (décalé en Z)
  geometry_msgs::msg::Pose p3 = p2;
  p3.position.z += 0.10;  // +10 cm en Z
  waypoints.push_back(p3);

  // -----------------------------------
  //   Calcul de la trajectoire cartésienne
  // -----------------------------------
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;    // résolution (m)
  const double jump_thresh = 0.0;  // désactiver la détection de "jumps"

  double fraction = move_group_interface.computeCartesianPath(
    waypoints,
    eef_step,
    jump_thresh,
    trajectory
  );

  // -----------------------------------
  //   Exécution de la trajectoire
  // -----------------------------------
  if (fraction > 0.95) {
    RCLCPP_INFO(logger, "Trajectoire cartésienne planifiée avec succès (%.1f%%).",
                fraction * 100.0);
    move_group_interface.execute(trajectory);
  } else {
    RCLCPP_ERROR(logger,
                 "Échec de la planification cartésienne, seulement %.1f%% du chemin trouvé.",
                 fraction * 100.0);
  }

  // Arrêt de ROS2
  rclcpp::shutdown();
  return 0;
}
