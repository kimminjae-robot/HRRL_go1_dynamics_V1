cmake_minimum_required(VERSION 3.10)
project(go1 LANGUAGES C CXX)

# -------------------- Global settings --------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 폴더 구조
set(MUJOCO_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(ONNX_DIR   "${CMAKE_CURRENT_LIST_DIR}/onnxruntime")

# "번들" 배포를 고려한 기본 prefix:
# 1) GO1의 상위 폴더(번들 루트)에 include/ lib/ 가 있으면 그걸 기본
# 2) 아니면 개발용 기본값: ~/DynamicsTools/install
set(_bundle_root "${CMAKE_CURRENT_LIST_DIR}/..")
if(EXISTS "${_bundle_root}/include" AND EXISTS "${_bundle_root}/lib")
  set(_dyn_default "${_bundle_root}")
else()
  set(_dyn_default "$ENV{HOME}/DynamicsTools/install")
endif()
set(DYNTOOLS_PREFIX "${_dyn_default}" CACHE PATH "Prefix containing include/ and lib/ for RBDL/qpSWIFT")

# -------------------- Packages / System libs --------------------
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# GLFW from pkg-config
pkg_check_modules(GLFW REQUIRED glfw3)

# OpenGL (GL) - 보통 Linux에서 그냥 GL로 링크해도 되지만, 가능한 find_package 사용
find_package(OpenGL REQUIRED)

# -------------------- Local/third-party libs (MuJoCo, ONNXRuntime) --------------------
find_library(MUJOCO_LIB
  NAMES mujoco
  HINTS "${MUJOCO_DIR}/lib"
  REQUIRED
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS "${ONNX_DIR}/lib"
  REQUIRED
)

# -------------------- Helper: manual deps from DYNTOOLS_PREFIX --------------------
# qpSWIFT
find_path(QPSWIFT_INCLUDE_DIR
  NAMES qpSWIFT.h qpswift.h
  HINTS "${DYNTOOLS_PREFIX}/include"
  PATH_SUFFIXES qpSWIFT qpswift
)
find_library(QPSWIFT_LIBRARY
  NAMES qpSWIFT qpswift
  HINTS "${DYNTOOLS_PREFIX}/lib"
)

# RBDL (+ URDF reader)
find_path(RBDL_INCLUDE_DIR
  NAMES rbdl/rbdl.h
  HINTS "${DYNTOOLS_PREFIX}/include"
)
find_library(RBDL_LIBRARY
  NAMES rbdl
  HINTS "${DYNTOOLS_PREFIX}/lib"
)
find_library(RBDL_URDF_LIBRARY
  NAMES rbdl_urdfreader
  HINTS "${DYNTOOLS_PREFIX}/lib"
)

if(NOT QPSWIFT_INCLUDE_DIR OR NOT QPSWIFT_LIBRARY)
  message(FATAL_ERROR
    "qpSWIFT not found in '${DYNTOOLS_PREFIX}'.\n"
    "  expected: ${DYNTOOLS_PREFIX}/include/(qpSWIFT|qpswift) and ${DYNTOOLS_PREFIX}/lib/libqpSWIFT*\n"
  )
endif()

if(NOT RBDL_INCLUDE_DIR OR NOT RBDL_LIBRARY OR NOT RBDL_URDF_LIBRARY)
  message(FATAL_ERROR
    "RBDL (and urdfreader) not found in '${DYNTOOLS_PREFIX}'.\n"
    "  expected: ${DYNTOOLS_PREFIX}/include/rbdl/rbdl.h and ${DYNTOOLS_PREFIX}/lib/librbdl*\n"
  )
endif()

# -------------------- Targets --------------------
# simulate library
file(GLOB SIMULATE_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/simulate/*.cc")
add_library(simulate STATIC ${SIMULATE_SOURCES})
target_include_directories(simulate PRIVATE
  "${MUJOCO_DIR}/include"
  "${CMAKE_CURRENT_LIST_DIR}/src/simulate"
  ${GLFW_INCLUDE_DIRS}
)
target_link_libraries(simulate PRIVATE
  ${GLFW_LIBRARIES}
  ${MUJOCO_LIB}
  OpenGL::GL
  Threads::Threads
  dl
  m
)

# controllers library
file(GLOB_RECURSE CONTROLLER_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/controllers/*.cpp")
add_library(controllers STATIC ${CONTROLLER_SOURCES})

target_include_directories(controllers PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/src/controllers"
  "${CMAKE_CURRENT_LIST_DIR}/src/controllers/Dynamics"
  ${EIGEN3_INCLUDE_DIRS}             
  "${ONNX_DIR}/include"
  "${QPSWIFT_INCLUDE_DIR}"
  "${RBDL_INCLUDE_DIR}"
)

target_link_libraries(controllers PUBLIC
  Eigen3::Eigen
  ${ONNXRUNTIME_LIB}
  "${QPSWIFT_LIBRARY}"
  "${RBDL_LIBRARY}"
  "${RBDL_URDF_LIBRARY}"
)

# executable
add_executable(test "${CMAKE_CURRENT_LIST_DIR}/src/main.cpp")
target_link_libraries(test PRIVATE
  simulate
  controllers
  yaml-cpp
)

# -------------------- Runtime convenience (optional but recommended for bundle) --------------------
# bin/test 실행 시 ../lib 자동 탐색 (번들 배포에 유리)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# -------------------- Install (optional) --------------------
install(TARGETS test RUNTIME DESTINATION bin)
