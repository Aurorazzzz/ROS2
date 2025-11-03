#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import rclpy
from rclpy.node import Node

# Service et message OpenMANIPULATOR-X
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.msg import KinematicsPose

# Pose ROS standard
from geometry_msgs.msg import Pose

class TaskSpacePositionClient(Node):
    """
    Client ROS2 pour le service:
      /goal_task_space_path_position_only  (open_manipulator_msgs/srv/SetKinematicsPose)

    Utilisation (exemples) :
      ros2 run <your_package> set_task_space_position_client.py -- 0.15 0.0 0.10 2.0
      ros2 run <your_package> set_task_space_position_client.py -- 0.20 -0.05 0.08 3.5 --group arm --ee tool_link
    """

    def __init__(self,
                 x: float,
                 y: float,
                 z: float,
                 path_time: float,
                 planning_group: str = 'arm',
                 end_effector_name: str = 'tool_link'):
        super().__init__('task_space_position_client')

        # Paramètres de la requête
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.path_time = float(path_time)
        self.planning_group = planning_group
        self.end_effector_name = end_effector_name

        # Création du client vers le service correspondant
        self.cli = self.create_client(SetKinematicsPose, '/goal_task_space_path_position_only')

        self.get_logger().info("En attente du service '/goal_task_space_path_position_only'...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service indisponible, nouvelle tentative...')

        # Préparer et envoyer la requête
        self.req = SetKinematicsPose.Request()
        self.req.planning_group = self.planning_group
        self.req.end_effector_name = self.end_effector_name

        # KinematicsPose (position uniquement) : on fixe orientation à l'identité (w=1)
        kp = KinematicsPose()
        kp.pose = Pose()
        kp.pose.position.x = self.x
        kp.pose.position.y = self.y
        kp.pose.position.z = self.z
        kp.pose.orientation.x = 0.0
        kp.pose.orientation.y = 0.0
        kp.pose.orientation.z = 0.0
        kp.pose.orientation.w = 1.0

        self.req.kinematics_pose = kp
        self.req.path_time = self.path_time

        # Appel asynchrone
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self._response_cb)

    def _response_cb(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Échec de l'appel au service: {e}")
        else:
            if res.is_planned:
                self.get_logger().info(
                    f"✓ Trajectoire planifiée: position cible=({self.x:.3f}, {self.y:.3f}, {self.z:.3f}) "
                    f"en {self.path_time:.3f}s (group='{self.planning_group}', ee='{self.end_effector_name}')."
                )
            else:
                self.get_logger().warn("✗ Le planificateur a renvoyé is_planned=False (échec de planification).")
        finally:
            # On ferme proprement le nœud après la réponse
            rclpy.shutdown()


def main(argv=None):
    # Parsing minimaliste des arguments CLI
    import argparse
    parser = argparse.ArgumentParser(
        description="Client ROS2 pour /goal_task_space_path_position_only (OpenMANIPULATOR-X).")
    parser.add_argument('x', type=float, help='Position X cible (m)')
    parser.add_argument('y', type=float, help='Position Y cible (m)')
    parser.add_argument('z', type=float, help='Position Z cible (m)')
    parser.add_argument('path_time', type=float, help="Durée totale de la trajectoire (s)")
    parser.add_argument('--group', default='arm', help="Nom du planning_group (défaut: 'arm')")
    parser.add_argument('--ee', default='tool_link', help="Nom de l’end-effector (défaut: 'tool_link')")

    args = parser.parse_args(argv)

    rclpy.init()
    TaskSpacePositionClient(
        x=args.x,
        y=args.y,
        z=args.z,
        path_time=args.path_time,
        planning_group=args.group,
        end_effector_name=args.ee
    )
    # spin_until_future_complete non nécessaire car on ferme dans le callback
    rclpy.spin(rclpy.get_global_executor().context)  # garde le process vivant jusqu’à shutdown()

if __name__ == '__main__':
    main()
