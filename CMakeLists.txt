cmake_minimum_required(VERSION 3.8)
project(slam)

# C++ standard
if(${CMAKE_VERSION} VERSION_LESS "3.2.0")
  set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
else()
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Glog REQUIRED)
find_package(Pangolin REQUIRED)

# include ldso headers (relativer Pfad vom slam package)
include_directories(
  ${CMAKE_SOURCE_DIR}/ldso_slam/include
  ${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty
  ${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/g2o
  ${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/DBoW3
)

# collect third-party libs produced in the ldso_slam tree (if present)
file(GLOB LDSO_G2O_LIBS "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/g2o/lib/*.${CMAKE_SHARED_LIBRARY_SUFFIX}" "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/g2o/lib/*.so" "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/g2o/lib/*.a")
file(GLOB LDSO_DBO3_LIBS "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/DBoW3/build/src/*${CMAKE_SHARED_LIBRARY_SUFFIX}" "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/DBoW3/build/src/*.so" "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/DBoW3/build/src/*.a")

set(THIRD_PARTY_LIBS ${LDSO_G2O_LIBS} ${LDSO_DBO3_LIBS})
# explicit lib dirs
set(G2O_LIB_DIR "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/g2o/lib")
set(DBO3_LIB_DIR "${CMAKE_SOURCE_DIR}/ldso_slam/thirdparty/DBoW3/build/src")
set(LDSO_LIB_DIR "${CMAKE_SOURCE_DIR}/ldso_slam/lib")

# listener (subscriber)
add_executable(listener src/subscriber_lambda_function.cpp)
ament_target_dependencies(listener rclcpp sensor_msgs cv_bridge)

target_include_directories(listener PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/ldso_slam/include
  ${EIGEN3_INCLUDE_DIR}
)

# try to resolve Glog library variable from find_package; fallback to plain "glog"
if(DEFINED GLOG_LIBRARY)
  set(GLOG_LIB ${GLOG_LIBRARY})
elseif(GLOG_LIBRARIES)
  set(GLOG_LIB ${GLOG_LIBRARIES})
else()
  find_library(GLOG_LIB NAMES glog libglog)
endif()

target_link_libraries(listener
  ${CMAKE_SOURCE_DIR}/ldso_slam/lib/libldso.a           # static ldso core
  ${THIRD_PARTY_LIBS}                                   # libs built inside ldso_slam/thirdparty
  ${GLOG_LIB}                                           # glog
  ${Pangolin_LIBRARIES}                                 # Pangolin (if found)
  ${OpenCV_LIBS}
  Threads::Threads
  ${CMAKE_DL_LIBS}                                      # sometimes needed for dynamic loading
)

# ensure runtime loader finds g2o/DBoW3/ldso libs (set RPATH)
set_target_properties(listener PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN/../lib;${G2O_LIB_DIR};${DBO3_LIB_DIR};${LDSO_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()