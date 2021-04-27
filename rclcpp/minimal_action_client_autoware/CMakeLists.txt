# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(minimal_action_client_autoware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(MINIMAL_ACTION_CLIENT_AUTOWARE_LIB_SRC
  src/minimal_action_client_autoware.cpp
)

set(MINIMAL_ACTION_CLIENT_AUTOWARE_LIB_HEADERS
  include/minimal_action_client_autoware/minimal_action_client_autoware.hpp
  include/minimal_action_client_autoware/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${MINIMAL_ACTION_CLIENT_AUTOWARE_LIB_SRC}
  ${MINIMAL_ACTION_CLIENT_AUTOWARE_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(MINIMAL_ACTION_CLIENT_AUTOWARE_NODE_SRC
  src/minimal_action_client_autoware_node.cpp
)

set(MINIMAL_ACTION_CLIENT_AUTOWARE_NODE_HEADERS
  include/minimal_action_client_autoware/minimal_action_client_autoware_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${MINIMAL_ACTION_CLIENT_AUTOWARE_NODE_SRC}
  ${MINIMAL_ACTION_CLIENT_AUTOWARE_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::minimal_action_client_autoware::MinimalActionClientAutowareNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_minimal_action_client_autoware.cpp)
  set(TEST_MINIMAL_ACTION_CLIENT_AUTOWARE_EXE test_minimal_action_client_autoware)
  ament_add_gtest(${TEST_MINIMAL_ACTION_CLIENT_AUTOWARE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_MINIMAL_ACTION_CLIENT_AUTOWARE_EXE})
  target_link_libraries(${TEST_MINIMAL_ACTION_CLIENT_AUTOWARE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)
