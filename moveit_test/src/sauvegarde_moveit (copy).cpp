insa@insa-desktop:~/ros_ws$ ros2 run moveit_test moveit_essaie
Error:   Semantic description is not specified for the same robot as the URDF
         at line 681 in ./src/model.cpp
[INFO] [1765467182.078755293] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 1.26056 seconds
[INFO] [1765467182.079744096] [moveit_robot_model.robot_model]: Loading robot model 'ur5'...
[INFO] [1765467182.080299960] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[WARN] [1765467182.167750904] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1765467182.372335712] [move_group_interface]: Ready to take commands for planning group ur_manipulator.
[INFO] [1765467182.374725738] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1765467182.406567243] [move_group_interface]: Planning request accepted
[INFO] [1765467182.634733081] [move_group_interface]: Planning request complete!
[INFO] [1765467182.636150545] [move_group_interface]: time taken to generate plan: 0.109931 seconds
[INFO] [1765467182.637048293] [hello_moveit]: Plan vers p1 trouvé, exécution...
[INFO] [1765467182.649601994] [move_group_interface]: Execute request accepted
[INFO] [1765467189.049155612] [move_group_interface]: Execute request success!
[INFO] [1765467189.049788641] [hello_moveit]: Robot positionné en p1 (approximativement).
[INFO] [1765467189.066118335] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1765467190.066891490] [moveit_ros.current_state_monitor]: Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds. Requested time 1765467189.066477, but latest received state has time 0.000000.
Check clock synchronization if your are running ROS across multiple machines!
[ERROR] [1765467190.068021106] [move_group_interface]: Failed to fetch current robot state
[INFO] [1765467190.132493336] [hello_moveit]: Fraction cartésienne trouvée : 26.3%
[ERROR] [1765467190.133036626] [hello_moveit]: Échec de la planification cartésienne, seulement 26.3% du chemin trouvé.
insa@insa-desktop:~/ros_ws$ 


