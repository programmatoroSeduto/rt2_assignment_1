cmake_minimum_required(VERSION 3.5)
project(rt2_assignment_1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## --- DEENDENCIES
find_package( ament_cmake REQUIRED )
find_package( rclcpp REQUIRED )
find_package( rosidl_default_generators REQUIRED )
find_package( std_msgs REQUIRED )

## --- BUILD OF INTERFACES
set( srv_files
	"srv/Command.srv"
	"srv/Position.srv"
	"srv/RandomPosition.srv"
)

rosidl_generate_interfaces( ${PROJECT_NAME} ${srv_files} DEPENDENCIES std_msgs )

## --- BUILD OF EXECUTABLES
add_executable( position_service src/position_service.cpp )
ament_target_dependencies( position_service rclcpp std_msgs )
rosidl_target_interfaces( position_service ${PROJECT_NAME} "rosidl_typesupport_cpp" )

add_executable( state_machine src/state_machine.cpp )
ament_target_dependencies( state_machine rclcpp std_msgs )
rosidl_target_interfaces( state_machine ${PROJECT_NAME} "rosidl_typesupport_cpp" )

install( TARGETS 
	position_service
	state_machine
DESTINATION lib/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
