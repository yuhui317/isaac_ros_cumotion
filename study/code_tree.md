# isaac_ros_cumotion · 完整代码树

[初步分析](analysis.md) · [源码定位](code_symbols.md) · [范围](annotation_scope.json) · [验证](annotation_validation.json)

固定基线 `4744ab9e29e4df73e11e0eb5c75cdbe07bcd99fe`，来源 `release-5.0`，学习分支 `codex/study/isaac-ros-code-analysis`。

138 个基线文件、0 个 gitlink；本轮注释 8 个原始文件。树不包含 `.git` 或缓存，固定基线自带的资源仍完整列出。

```text
isaac_ros_cumotion/
├── .gitattributes
├── .gitignore  [只读参考；未注释]
├── LICENSE
├── README.md  [只读参考；未注释]
├── SECURITY.md
├── isaac_ros_cumotion/
│   ├── CMakeLists.txt  [片段阅读；未注释]
│   ├── include/
│   │   └── isaac_ros_cumotion/
│   │       ├── cumotion_planner.hpp  [本轮中文注释]
│   │       ├── impl/
│   │       │   ├── cumotion_logger_bridge.hpp  [原样保留；未展开]
│   │       │   ├── ik_solver_impl.hpp  [原样保留；未展开]
│   │       │   ├── robot_manager_impl.hpp  [片段阅读；未注释]
│   │       │   ├── trajectory_optimizer_impl.hpp  [原样保留；未展开]
│   │       │   ├── utils.hpp  [原样保留；未展开]
│   │       │   └── world_manager_impl.hpp  [原样保留；未展开]
│   │       └── static_planning_scene.hpp  [原样保留；未展开]
│   ├── launch/
│   │   └── isaac_ros_cumotion.launch.py  [本轮中文注释]
│   ├── package.xml  [片段阅读；未注释]
│   ├── params/
│   │   └── isaac_ros_cumotion_params.yaml
│   ├── scripts/
│   │   └── verify_esdf_self_consistent.py  [原样保留；未展开]
│   ├── src/
│   │   ├── cumotion_planner.cpp  [片段阅读；未注释]
│   │   ├── impl/
│   │   │   ├── cumotion_logger_bridge.cpp  [原样保留；未展开]
│   │   │   ├── ik_solver_impl.cpp  [原样保留；未展开]
│   │   │   ├── robot_manager_impl.cpp  [本轮中文注释]
│   │   │   ├── trajectory_optimizer_impl.cpp  [本轮中文注释]
│   │   │   ├── utils.cpp  [原样保留；未展开]
│   │   │   └── world_manager_impl.cpp  [本轮中文注释]
│   │   └── static_planning_scene.cpp  [原样保留；未展开]
│   └── test/
│       ├── download_rosbag.sh  [原样保留；未展开]
│       ├── test_cumotion_logger_bridge.cpp  [原样保留；未展开]
│       ├── test_cumotion_logger_bridge.hpp  [原样保留；未展开]
│       ├── test_moveit_scene_file_parser.cpp  [原样保留；未展开]
│       ├── test_robot_manager_impl.cpp  [原样保留；未展开]
│       ├── test_robot_manager_impl.hpp  [原样保留；未展开]
│       ├── test_trajectory_optimizer_impl.cpp  [原样保留；未展开]
│       ├── test_trajectory_optimizer_impl.hpp  [原样保留；未展开]
│       ├── test_utils_compose_pose.cpp  [原样保留；未展开]
│       ├── test_utils_compose_pose.hpp  [原样保留；未展开]
│       └── test_world_manager_impl.cpp  [原样保留；未展开]
├── isaac_ros_cumotion_controllers/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── isaac_ros_cumotion_controllers/
│   │       ├── bimanual_ik_controller.hpp  [原样保留；未展开]
│   │       ├── controller_utils.hpp  [原样保留；未展开]
│   │       ├── ik_controller.hpp  [原样保留；未展开]
│   │       └── ik_controller_base.hpp  [原样保留；未展开]
│   ├── isaac_ros_cumotion_controllers.xml
│   ├── package.xml
│   ├── scripts/
│   │   └── ik_controller_marker_node.py  [原样保留；未展开]
│   └── src/
│       ├── bimanual_ik_controller.cpp  [原样保留；未展开]
│       ├── ik_controller.cpp  [本轮中文注释]
│       └── ik_controller_base.cpp  [原样保留；未展开]
├── isaac_ros_cumotion_examples/
│   ├── launch/
│   │   ├── franka.launch.py  [只读参考；未注释]
│   │   ├── franka_isaac_sim.launch.py  [原样保留；未展开]
│   │   ├── ur.launch.py  [原样保留；未展开]
│   │   └── ur_isaac_sim.launch.py  [原样保留；未展开]
│   ├── package.xml
│   ├── resource/
│   │   └── isaac_ros_cumotion_examples
│   ├── rviz/
│   │   ├── franka_moveit_config.rviz
│   │   └── ur_moveit_config.rviz
│   ├── setup.py  [原样保留；未展开]
│   ├── test/
│   │   ├── test_copyright.py  [原样保留；未展开]
│   │   ├── test_flake8.py  [原样保留；未展开]
│   │   ├── test_franka_launch.py  [原样保留；未展开]
│   │   └── test_pep257.py  [原样保留；未展开]
│   └── ur_config/
│       ├── initial_positions.yaml
│       ├── ros2_controllers.yaml
│       ├── ur.ros2_control.xacro
│       └── ur.urdf.xacro
├── isaac_ros_cumotion_interfaces/
│   ├── CMakeLists.txt
│   ├── action/
│   │   ├── AttachObject.action
│   │   ├── IKSolution.action
│   │   ├── MotionPlan.action
│   │   └── UpdateLinkSpheres.action
│   ├── package.xml
│   └── srv/
│       ├── GetRobotDescription.srv
│       ├── PublishStaticPlanningScene.srv
│       └── SetRobotDescription.srv
├── isaac_ros_cumotion_moveit/
│   ├── CMakeLists.txt
│   ├── config/
│   │   └── isaac_ros_cumotion_planning.yaml
│   ├── cumotion_planner_plugin_description.xml
│   ├── include/
│   │   └── isaac_ros_cumotion_moveit/
│   │       ├── cumotion_interface.hpp  [原样保留；未展开]
│   │       ├── cumotion_move_group_client.hpp  [原样保留；未展开]
│   │       ├── cumotion_planner_manager.hpp  [原样保留；未展开]
│   │       └── cumotion_planning_context.hpp  [原样保留；未展开]
│   ├── package.xml
│   └── src/
│       ├── cumotion_interface.cpp  [本轮中文注释]
│       ├── cumotion_move_group_client.cpp  [只读参考；未注释]
│       ├── cumotion_planner_manager.cpp  [原样保留；未展开]
│       └── cumotion_planning_context.cpp  [本轮中文注释]
├── isaac_ros_cumotion_object_attachment/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── isaac_ros_cumotion_object_attachment/
│   │       └── object_attachment.hpp  [原样保留；未展开]
│   ├── launch/
│   │   └── object_attachment.launch.py  [原样保留；未展开]
│   ├── package.xml
│   ├── params/
│   │   └── object_attachment_params.yaml
│   ├── src/
│   │   └── object_attachment.cpp  [原样保留；未展开]
│   └── test/
│       ├── isaac_ros_object_attachment_esdf_test.py  [原样保留；未展开]
│       ├── isaac_ros_object_attachment_pol_test.py  [原样保留；未展开]
│       ├── object_attachment_test_base.py  [原样保留；未展开]
│       └── test_cuboid_geometry.cpp  [原样保留；未展开]
├── isaac_ros_cumotion_robot_description/
│   ├── package.xml
│   ├── resource/
│   │   └── isaac_ros_cumotion_robot_description
│   ├── setup.py  [原样保留；未展开]
│   ├── test/
│   │   ├── test_copyright.py  [原样保留；未展开]
│   │   ├── test_flake8.py  [原样保留；未展开]
│   │   └── test_pep257.py  [原样保留；未展开]
│   ├── urdf/
│   │   ├── flexiv_rizon4s_grav.urdf
│   │   ├── ur10e_robotiq_2f_140.urdf
│   │   ├── ur10e_robotiq_2f_85.urdf
│   │   ├── ur5e_robotiq_2f_140.urdf
│   │   └── ur5e_robotiq_2f_85.urdf
│   └── xrdf/
│       ├── flexiv_rizon4s_grav.xrdf
│       ├── flexiv_rizon4s_grav_no_gripper_spheres.xrdf
│       ├── franka.xrdf
│       ├── ur10e.xrdf
│       ├── ur10e_robotiq_2f_140.xrdf
│       ├── ur10e_robotiq_2f_85.xrdf
│       ├── ur5e.xrdf
│       ├── ur5e_robotiq_2f_140.xrdf
│       └── ur5e_robotiq_2f_85.xrdf
├── isaac_ros_cumotion_robot_segmenter/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── isaac_ros_cumotion_robot_segmenter/
│   │       └── robot_segmenter.hpp  [原样保留；未展开]
│   ├── launch/
│   │   └── robot_segmenter.launch.py  [原样保留；未展开]
│   ├── package.xml
│   ├── params/
│   │   └── robot_segmentation_params.yaml
│   ├── src/
│   │   └── robot_segmenter.cpp  [原样保留；未展开]
│   └── test/
│       ├── .gitignore
│       ├── download_rosbag.sh  [原样保留；未展开]
│       ├── test_data/
│       │   ├── camera_pose.json
│       │   ├── depth_camera_info.json
│       │   ├── golden_robot_segmenter_depth_float16.npy
│       │   ├── golden_robot_segmenter_depth_uint16.npy
│       │   ├── golden_robot_segmenter_mask_float16.npy
│       │   ├── golden_robot_segmenter_mask_uint16.npy
│       │   ├── input_depth_image.npy
│       │   ├── robot.urdf
│       │   ├── robot.xrdf
│       │   └── robot_mask_from_curobo.npy
│       ├── test_robot_segmenter_float16.py  [原样保留；未展开]
│       ├── test_robot_segmenter_pol.py  [原样保留；未展开]
│       ├── test_robot_segmenter_reload.py  [原样保留；未展开]
│       └── test_robot_segmenter_uint16.py  [原样保留；未展开]
└── study/
    ├── analysis.md  [新增学习材料]
    ├── annotation_scope.json  [新增学习材料]
    ├── annotation_validation.json  [新增学习材料]
    ├── code_symbols.md  [新增学习材料]
    ├── code_tree.md  [新增学习材料]
    ├── generate_study_index.py  [新增学习材料]
    ├── repository.json  [新增学习材料]
    └── verify_annotations.py  [新增学习材料]
```

## 本轮阅读入口

| 文件 | 范围 | 阅读深度 |
|---|---|---|
| [.gitignore](../.gitignore) | 只读参考，未注释 | full |
| [README.md](../README.md) | 只读参考，未注释 | full |
| [isaac_ros_cumotion/CMakeLists.txt](../isaac_ros_cumotion/CMakeLists.txt) | 只读参考，未注释 | partial: vendor/link/dependency structural searches |
| [isaac_ros_cumotion/include/isaac_ros_cumotion/cumotion_planner.hpp](../isaac_ros_cumotion/include/isaac_ros_cumotion/cumotion_planner.hpp) | 中文注释 | full |
| [isaac_ros_cumotion/include/isaac_ros_cumotion/impl/robot_manager_impl.hpp](../isaac_ros_cumotion/include/isaac_ros_cumotion/impl/robot_manager_impl.hpp) | 只读参考，未注释 | partial: original baseline lines 80-145 |
| [isaac_ros_cumotion/launch/isaac_ros_cumotion.launch.py](../isaac_ros_cumotion/launch/isaac_ros_cumotion.launch.py) | 中文注释 | full |
| [isaac_ros_cumotion/package.xml](../isaac_ros_cumotion/package.xml) | 只读参考，未注释 | partial: package version and dependency searches |
| [isaac_ros_cumotion/src/cumotion_planner.cpp](../isaac_ros_cumotion/src/cumotion_planner.cpp) | 只读参考，未注释 | partial: original baseline lines 650-989, plus symbol/call searches |
| [isaac_ros_cumotion/src/impl/robot_manager_impl.cpp](../isaac_ros_cumotion/src/impl/robot_manager_impl.cpp) | 中文注释 | full |
| [isaac_ros_cumotion/src/impl/trajectory_optimizer_impl.cpp](../isaac_ros_cumotion/src/impl/trajectory_optimizer_impl.cpp) | 中文注释 | full |
| [isaac_ros_cumotion/src/impl/world_manager_impl.cpp](../isaac_ros_cumotion/src/impl/world_manager_impl.cpp) | 中文注释 | full |
| [isaac_ros_cumotion_controllers/src/ik_controller.cpp](../isaac_ros_cumotion_controllers/src/ik_controller.cpp) | 中文注释 | full |
| [isaac_ros_cumotion_examples/launch/franka.launch.py](../isaac_ros_cumotion_examples/launch/franka.launch.py) | 只读参考，未注释 | full |
| [isaac_ros_cumotion_moveit/src/cumotion_interface.cpp](../isaac_ros_cumotion_moveit/src/cumotion_interface.cpp) | 中文注释 | full |
| [isaac_ros_cumotion_moveit/src/cumotion_move_group_client.cpp](../isaac_ros_cumotion_moveit/src/cumotion_move_group_client.cpp) | 只读参考，未注释 | full |
| [isaac_ros_cumotion_moveit/src/cumotion_planning_context.cpp](../isaac_ros_cumotion_moveit/src/cumotion_planning_context.cpp) | 中文注释 | full |

目录树和行号是定位材料，不代表所有代码已理解或运行。子模块只保留固定引用，内部文件不在本轮范围。

重新生成：`python study/generate_study_index.py`。再运行 `python study/verify_annotations.py --report study/annotation_validation.json` 审查源码保留情况；两个工具均不运行上游项目。
