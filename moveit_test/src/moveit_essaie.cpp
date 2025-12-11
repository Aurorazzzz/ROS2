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

  // On fixe le repère de référence des poses de l'EEF
  move_group_interface.setPoseReferenceFrame("base_link");

  // Optionnel : on peut réduire un peu la vitesse max
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

  // -----------------------------
  //         Point 1 (p1)
  // -----------------------------
  geometry_msgs::msg::Pose p1;
  p1.orientation.x = 0.0;
  p1.orientation.y = 0.0;
  p1.orientation.z = 0.0;
  p1.orientation.w = 1.0;   // orientation identité
  p1.position.x = 0.28;
  p1.position.y = -0.20;
  p1.position.z = 0.50;

  // -----------------------------
  //   Aller à p1 avec un plan normal
  // -----------------------------
  move_group_interface.setPoseTarget(p1);

  MoveGroupInterface::Plan plan_to_p1;
  auto plan_result = move_group_interface.plan(plan_to_p1);

  if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Impossible de planifier un mouvement vers p1, arrêt du programme.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Plan vers p1 trouvé, exécution...");
  auto exec_result = move_group_interface.execute(plan_to_p1);
  if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Échec de l'exécution du plan vers p1, arrêt du programme.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Robot positionné en p1 (approximativement).");

  // -----------------------------
  //     Définition des waypoints
  //     à partir de la pose courante
  // -----------------------------
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Pose de départ = pose courante de l'EEF après le mouvement vers p1
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

  // Point 2 : décalage en Y
  geometry_msgs::msg::Pose p2 = start_pose;
  p2.position.y -= 0.10;   // -10 cm en Y
  waypoints.push_back(p2);

  // Point 3 : décalage en Z
  geometry_msgs::msg::Pose p3 = p2;
  p3.position.z += 0.10;   // +10 cm en Z
  waypoints.push_back(p3);

  // -----------------------------------
  //   Calcul de la trajectoire cartésienne
  // -----------------------------------
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;    // 1 cm de résolution
  const double jump_thresh = 0.0;  // pas de détection de "jumps"

  double fraction = move_group_interface.computeCartesianPath(
    waypoints,
    eef_step,
    jump_thresh,
    trajectory
  );

  RCLCPP_INFO(logger, "Fraction cartésienne trouvée : %.1f%%", fraction * 100.0);

  // -----------------------------------
  //   Exécution de la trajectoire
  // -----------------------------------
  if (fraction > 0.95) {
    RCLCPP_INFO(logger,
      "Trajectoire cartésienne planifiée avec succès (%.1f%% du chemin). Exécution...",
      fraction * 100.0);
    auto exec_result_cart = move_group_interface.execute(trajectory);
    if (exec_result_cart != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Échec de l'exécution de la trajectoire cartésienne.");
    }
  } else {
    RCLCPP_ERROR(
      logger,
      "Échec de la planification cartésienne, seulement %.1f%% du chemin trouvé.",
      fraction * 100.0);
  }

  // Arrêt de ROS2
  rclcpp::shutdown();
  return 0;
}
