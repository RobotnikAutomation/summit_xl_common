cmake_minimum_required(VERSION 3.5)
project(summit_xl_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf robots
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(urdf_xacro test/test_robotnik_xacro.py)
endif()

ament_package()
