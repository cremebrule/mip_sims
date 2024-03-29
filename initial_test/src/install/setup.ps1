# generated from colcon_powershell/shell/template/prefix_chain.ps1.em

# This script extends the environment with the environment of other prefix
# paths which were sourced when this file was generated as well as all packages
# contained in this prefix path.

# function to source another script with conditional trace output
# first argument: the path of the script
function _colcon_prefix_chain_powershell_source_script {
  param (
    $_colcon_prefix_chain_powershell_source_script_param
  )
  # source script with conditional trace output
  if (Test-Path $_colcon_prefix_chain_powershell_source_script_param) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_prefix_chain_powershell_source_script_param'"
    }
    . "$_colcon_prefix_chain_powershell_source_script_param"
  } else {
    Write-Error "not found: '$_colcon_prefix_chain_powershell_source_script_param'"
  }
}

# source chained prefixes
_colcon_prefix_chain_powershell_source_script "/opt/ros/crystal\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_clang_format\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_copyright\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cppcheck\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cpplint\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_flake8\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_index_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_index_python\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_lint_cmake\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_package\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_core\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_export_definitions\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_export_include_directories\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_export_libraries\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_export_interfaces\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_export_link_flags\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_include_directories\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_libraries\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_export_dependencies\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_python\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_target_dependencies\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_test\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_auto\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_clang_format\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_copyright\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_cppcheck\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_cpplint\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_flake8\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_lint_cmake\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_nose\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_pytest\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_ros\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_lint_auto\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_pclint\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_pclint\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_pep257\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_pep257\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_pep8\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_pep8\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_pyflakes\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_pyflakes\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_xmllint\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_xmllint\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/connext_cmake_module\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/console_bridge_vendor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/fastrtps_cmake_module\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_gtest\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_gmock\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/libcurl_vendor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/libyaml_vendor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/message_filters\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/opensplice_cmake_module\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/osrf_pycommon\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/launch\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/launch_testing\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/class_loader\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/python_cmake_module\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/python_qt_binding\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/qt_dotgraph\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/qt_gui\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/qt_gui_app\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/qt_gui_py_common\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcl_logging_log4cxx\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcl_logging_noop\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcl_yaml_param_parser\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcutils\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/resource_retriever\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_connext_shared_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_implementation_cmake\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros_environment\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_adapter\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_parser\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_actions\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_cmake\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_generator_dds_idl\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_interface\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_generator_c\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_fastrtps_shared_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_generator_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_connext_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_connext_c\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_connext_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_fastrtps_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_fastrtps_c\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_fastrtps_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_introspection_c\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_introspection_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_fastrtps_dynamic_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_opensplice_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_opensplice_c\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_opensplice_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rmw_implementation\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_c\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_generator_py\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_typesupport_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_default_generators\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosidl_default_runtime\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/builtin_interfaces\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/lifecycle_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/pendulum_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcl_interfaces\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcl\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcl_lifecycle\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rosgraph_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rclcpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_timer\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rttest\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz_assimp_vendor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz_ogre_vendor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz_rendering\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz_rendering_tests\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/std_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/actionlib_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/demo_nodes_cpp_native\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_composition\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_publisher\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_subscriber\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/geometry_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/diagnostic_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/initial_test\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/logging_demo\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/nav_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/dummy_map_server\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/sensor_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/dummy_sensors\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/image_tools\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/intra_process_demo\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/map_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/shape_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/std_srvs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/stereo_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/test_cli\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/test_cli_remapping\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/test_communication\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/test_launch_ros\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/test_rclcpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/test_security\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tf2\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/laser_geometry\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tf2_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tf2_ros\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tf2_eigen\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tf2_geometry_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tf2_sensor_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/pluginlib\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/qt_gui_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/qt_gui_core\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_gui_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tlsf\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/tlsf_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/pendulum_control\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/trajectory_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/uncrustify_vendor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_uncrustify\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_cmake_uncrustify\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ament_lint_common\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/unique_identifier_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/action_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/example_interfaces\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/composition\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_client\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_service\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rcl_action\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rclcpp_action\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_action_client\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclcpp_minimal_action_server\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rclpy\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/demo_nodes_py\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclpy_executors\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclpy_minimal_action_client\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclpy_minimal_action_server\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclpy_minimal_client\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclpy_minimal_publisher\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclpy_minimal_service\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/examples_rclpy_minimal_subscriber\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/launch_ros\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/demo_nodes_cpp\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rclcpp_lifecycle\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2cli\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2msg\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2multicast\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2node\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2param\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2pkg\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2launch\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2run\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/lifecycle\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2srv\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2topic\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2service\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/ros2lifecycle\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_gui\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_gui_py\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_py_common\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_console\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_msg\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_plot\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_publisher\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_py_console\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_service_caller\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_shell\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_srv\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rqt_top\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/sros2\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/test_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/topic_monitor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/urdf\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/kdl_parser\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/robot_state_publisher\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/dummy_robot_bringup\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/visualization_msgs\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/common_interfaces\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/yaml_cpp_vendor\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz_common\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz_default_plugins\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz2\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install/rviz_visual_testing_framework\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/opt/ros/crystal\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/cremebrule/ros2_ws/install\local_setup.ps1"

# source this prefix
$env:COLCON_CURRENT_PREFIX=(Split-Path $PSCommandPath -Parent)
_colcon_prefix_chain_powershell_source_script "$env:COLCON_CURRENT_PREFIX\local_setup.ps1"
