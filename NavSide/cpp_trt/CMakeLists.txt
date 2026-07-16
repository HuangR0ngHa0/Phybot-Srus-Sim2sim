cmake_minimum_required(VERSION 3.20)
project(navside_cpp_trt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Python3 REQUIRED COMPONENTS Development NumPy)

set(TENSORRT_ROOT "" CACHE PATH "TensorRT root directory")
set(CUDA_ROOT "" CACHE PATH "CUDA root directory")

find_path(TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS
    ${TENSORRT_ROOT}/include
    /usr/include/aarch64-linux-gnu
    /usr/include
)

find_library(TENSORRT_NVINFER_LIB
  NAMES nvinfer
  HINTS
    ${TENSORRT_ROOT}/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)

find_library(TENSORRT_NVINONPARSER_LIB
  NAMES nvonnxparser
  HINTS
    ${TENSORRT_ROOT}/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)

find_path(CUDA_INCLUDE_DIR
  NAMES cuda_runtime_api.h
  HINTS
    ${CUDA_ROOT}/include
    /usr/local/cuda/include
    /usr/include
)

find_library(CUDA_CUDART_LIB
  NAMES cudart
  HINTS
    ${CUDA_ROOT}/lib64
    /usr/local/cuda/lib64
    /usr/lib/aarch64-linux-gnu
)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_NVINFER_LIB OR NOT TENSORRT_NVINONPARSER_LIB)
  message(FATAL_ERROR "TensorRT headers/libs not found. Set TENSORRT_ROOT if needed.")
endif()

if(NOT CUDA_INCLUDE_DIR OR NOT CUDA_CUDART_LIB)
  message(FATAL_ERROR "CUDA headers/libs not found. Set CUDA_ROOT if needed.")
endif()

add_library(navside_trt MODULE
  src/TensorRTEngine.cpp
  src/NavSideTRTRunner.cpp
  python/bindings.cpp
)

target_include_directories(navside_trt PRIVATE
  include
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
  ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(navside_trt PRIVATE
  Python3::Python
  ${TENSORRT_NVINFER_LIB}
  ${TENSORRT_NVINONPARSER_LIB}
  ${CUDA_CUDART_LIB}
)

target_compile_definitions(navside_trt PRIVATE
  PY_SSIZE_T_CLEAN
  NPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION
)

set_target_properties(navside_trt PROPERTIES PREFIX "")
