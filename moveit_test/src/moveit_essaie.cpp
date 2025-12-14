#include <memory>
#include <thread>
#include <chrono>

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

  // -----------------------------
  //          Point 1 (p1)
  // -----------------------------
  geometry_msgs::msg::Pose p1;
  // p1.orientation.x = -0.675;
  // p1.orientation.y = 0.147;
  // p1.orientation.z = -0.140;
  // p1.orientation.w = 0.709; 
  p1.orientation.w = 1;   // orientation identité
  p1.position.x = -0.07;
  p1.position.y = 0.759;
  p1.position.z = 0.433;

  // -----------------------------
  //   Aller à p1 avec un plan normal
  // -----------------------------
  move_group_interface.setPoseTarget(p1);

  MoveGroupInterface::Plan plan_to_p1;
  auto plan_result = move_group_interface.plan(plan_to_p1);

  if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Impossible de planifier un mouvement vers p1, arrêt du programme.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Plan vers p1 trouvé, exécution...");
  auto exec_result = move_group_interface.execute(plan_to_p1);
  if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Échec de l'exécution du plan vers p1, arrêt du programme.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Robot positionné en p1 (approximativement).");

  // On laisse un petit délai pour que le state monitor se mette à jour après le mouvement
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  move_group_interface.setStartStateToCurrentState();

  // -----------------------------
  //  Waypoints à partir de la pose courante
  // -----------------------------
  std::vector<geometry_msgs::msg::Pose> waypoints;

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
  const double eef_step = 0.01;    // 1 cm
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

  // Arrêt propre
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
