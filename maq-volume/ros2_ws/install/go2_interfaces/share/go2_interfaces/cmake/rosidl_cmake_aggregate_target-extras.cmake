# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target go2_interfaces::go2_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${go2_interfaces_TARGETS}.
if(go2_interfaces_TARGETS AND NOT TARGET go2_interfaces::go2_interfaces)
  add_library(go2_interfaces::go2_interfaces INTERFACE IMPORTED)
  set_target_properties(go2_interfaces::go2_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${go2_interfaces_TARGETS}")
endif()
