cmake_minimum_required(VERSION 3.12)  # 更新最小 CMake 版本
# cmake_policy(SET CMP0148 NEW)

set(CMAKE_WARN_DEVELOPER OFF)
set(CMAKE_BUILD_TYPE "Release")


add_compile_options(-O3  -ffast-math )

# Set the C standard to C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
# add_compile_options(-std=c++17)

project(phybot_software)
message(STATUS "Project source directory: ${PROJECT_SOURCE_DIR}/..")
add_compile_options(-Wno-return-type)

# # catkin 包信息
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs
# )

# set(THREADS_PREFER_PTHREAD_FLAG ON)
# find_package(Threads REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# link_directories("/usr/local/lib")
# INCLUDE_DIRECTORIES("/usr/local/include")
# link_directories("/usr/lib")
# INCLUDE_DIRECTORIES("/usr/include")
# include_directories("/usr/include/GL")
# find_package(OpenGL REQUIRED)
# include_directories(${OPENGL_INCLUDE_DIR})

set(CMAKE_PREFIX_PATH "/opt/ros/noetic" ${PROJECT_SOURCE_DIR}/ThirdParty/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
# find_package(yaml-cpp REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/MujocoInterface/include/simulate)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/libtorch/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/boost/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/yaml/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/spdlog/include)

include_directories(${PROJECT_SOURCE_DIR}/Joystick/include)
include_directories(${PROJECT_SOURCE_DIR}/MotorList/include)
link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/boost/lib)

link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/yaml/lib)
link_directories(${PROJECT_SOURCE_DIR}/MotorList/lib)


aux_source_directory(${PROJECT_SOURCE_DIR}/DataPackage/src dir_datapackage)
# add_library(DataPackage SHARED ${dir_datapackage})

# target_link_libraries(DataPackage pinocchio yaml-cpp)

# ======================== [修改] OpenCV 和 Orbbec 配置 ========================
# 1. 查找 OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Found: ${OpenCV_VERSION}")

# 2. Orbbec SDK 路径配置
# 设置 SDK 根目录
set(ORBBEC_SDK_DIR "${PROJECT_SOURCE_DIR}/ThirdParty/OrbbecSDK")

# 添加头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ORBBEC_SDK_DIR}/include)

# 添加库文件路径 
# (根据你的 ls 结果，库直接在 lib 下，没有 linux_x64)
link_directories(${ORBBEC_SDK_DIR}/lib)
# ============================================================================



if(WHICH_ENV AND(WHICH_ENV STREQUAL "realrobot_mini"))


    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/yaml/include)
    include_directories(${PROJECT_SOURCE_DIR}/MujocoInterface/include)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_amp/include)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_mimic/include)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/include)


    include_directories(${PROJECT_SOURCE_DIR}/DataPackage/include)
    include_directories(${PROJECT_SOURCE_DIR}/StateMachine/include)
    include_directories(${PROJECT_SOURCE_DIR}/MotorList/include)
    include_directories(${PROJECT_SOURCE_DIR}/LowPassFilter/include)
    include_directories(${PROJECT_SOURCE_DIR}/Mocap/include)
    include_directories(${PROJECT_SOURCE_DIR}/ZeroState/include)

    aux_source_directory(${PROJECT_SOURCE_DIR}/Joystick/src dir_js)
    aux_source_directory(${PROJECT_SOURCE_DIR}/StateMachine/src dir_FSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/MotorList/src dir_ML)
    aux_source_directory(${PROJECT_SOURCE_DIR}/LowPassFilter/src dir_LPF)
    aux_source_directory(${PROJECT_SOURCE_DIR}/Mocap/src dir_mocap)
    aux_source_directory(${PROJECT_SOURCE_DIR}/ZeroState/src dir_ZSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_amp/src dir_rl_amp)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_mimic/src dir_rl_mimic)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/src dir_rl_cpg)


    link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/yaml/lib)
    link_directories(${PROJECT_SOURCE_DIR}/MotorList/lib)
    link_directories(${PROJECT_SOURCE_DIR}/Mocap/lib)

    set(IMU_SOURCES
    # test.cpp
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/HipnucReader.cpp
    # main.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/serial_port.c  # Change this to C++ source if serial_port.c actually contains C++ code
    # commands.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/log.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/hex2bin.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/kboot.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/example_data.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/hipnuc_dec.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/nmea_dec.c
    )

    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib)



    add_executable(main ${PROJECT_SOURCE_DIR}/RobotStart/realrobot/run_mini.cpp

                        ${IMU_SOURCES}
                        ${dir_js}
                        ${dir_FSM}
                        ${dir_mocap}
                        ${dir_datapackage}
                        ${dir_ML}
                        ${dir_LPF}
                        ${dir_ZSM}
                        ${dir_rl_amp}
                        ${dir_rl_mimic}
                        ${dir_rl_cpg}
                        )
    target_link_libraries(main  PUBLIC 
                            ${TORCH_LIBRARIES}
                            pthread
                            LuMoSDK
                            yaml-cpp
                            MotorDrive
                            # HARDWARE_TOP
                            # Main_Control_Board_Device
                            ${OpenCV_LIBS} # 链接 OpenCV
                            OrbbecSDK 
                            )

    add_executable(set_zero ${PROJECT_SOURCE_DIR}/RobotStart/realrobot/set_zero_mini.cpp

                            # ${IMU_SOURCES}
                            ${dir_LPF}

                            ${dir_datapackage}
                            ${dir_ML}
                            ${dir_ZSM}
                            )
    target_link_libraries(set_zero  PUBLIC 
                                ${TORCH_LIBRARIES}
                                pthread
                                yaml-cpp
                                MotorDrive
                                # HARDWARE_TOP
                                # Main_Control_Board_Device
                                )
    


elseif(WHICH_ENV AND(WHICH_ENV STREQUAL "mujoco_sim_mini"))
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/mujoco)
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/glfw)
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/yaml/include)
    include_directories(${PROJECT_SOURCE_DIR}/MujocoInterface/include)
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/gl/include)


    # include_directories(${PROJECT_SOURCE_DIR}/DataLogger/include)
    include_directories(${PROJECT_SOURCE_DIR}/DataPackage/include)
    include_directories(${PROJECT_SOURCE_DIR}/StateMachine/include)
    include_directories(${PROJECT_SOURCE_DIR}/LowPassFilter/include)
    include_directories(${PROJECT_SOURCE_DIR}/ZeroState/include)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_amp/include)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_mimic/include)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/include)


    aux_source_directory(${PROJECT_SOURCE_DIR}/MujocoInterface/src dir_mujoco)
    aux_source_directory(${PROJECT_SOURCE_DIR}/Joystick/src dir_js)
    aux_source_directory(${PROJECT_SOURCE_DIR}/StateMachine/src dir_FSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/LowPassFilter/src dir_LPF)
    aux_source_directory(${PROJECT_SOURCE_DIR}/ZeroState/src dir_ZSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_amp/src dir_rl_amp)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_mimic/src dir_rl_mimic)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/src dir_rl_cpg)


    
    # message(STATUS "joystick: ${dir_js}")

    link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/mujoco/lib)
    link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/glfw/lib)

    set(IMU_SOURCES
    # test.cpp
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/HipnucReader.cpp
    # main.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/serial_port.c  # Change this to C++ source if serial_port.c actually contains C++ code
    # commands.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/log.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/hex2bin.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/kboot.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/example_data.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/hipnuc_dec.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/nmea_dec.c
    )

    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib)

    add_executable(main ${PROJECT_SOURCE_DIR}/RobotStart/mujoco_sim/main.cc
                        
                        ${dir_mujoco}
                        ${IMU_SOURCES}
                        ${dir_js}
                        ${dir_FSM}
                        ${dir_datapackage}
                        ${dir_LPF}
                        ${dir_ZSM}
                        ${dir_rl_amp}
                        ${dir_rl_mimic}
                        ${dir_rl_cpg}
                        )
    target_link_libraries(main  PUBLIC 
                            ${TORCH_LIBRARIES}
                            mujoco
                            glfw3 
                            # Threads::Threads
                            pthread

                            yaml-cpp
                            # tinysplinecxx 

                            # torch
                            ${OpenCV_LIBS}

                            OrbbecSDK
                            )

                            # target_link_libraries(${PROJECT} "${TORCH_LIBRARIES}")
                            # set_property(TARGET ${PROJECT} PROPERTY CXX_STANDARD 17)
    
elseif(WHICH_ENV AND(WHICH_ENV STREQUAL "gazebo_sim"))
    ## 查找 ROS 依赖
    # set(phybot_msgs_DIR /home/kairan/Documents/phybot_ws/src/phybot_msgs)
    set(phybot_msgs_DIR "${PROJECT_SOURCE_DIR}/../../devel/share/phybot_msgs/cmake")
    add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/gazebo_sim/")

    find_package(catkin REQUIRED COMPONENTS
        phybot_msgs
        roscpp
        std_msgs
        tf
        geometry_msgs
        rospy
        gazebo_ros
        controller_manager
        joint_state_controller
        robot_state_publisher
        genmsg
        

    )
    
    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs
    )
    
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/gazebo_sim/include
        ${PROJECT_SOURCE_DIR}/gazebo_sim/library/loop
        ${PROJECT_SOURCE_DIR}/gazebo_sim/library/rl_sdk

    )

    include_directories(${PROJECT_SOURCE_DIR}/gazebo_sim/include)

    add_library(rl_sdk ${PROJECT_SOURCE_DIR}/gazebo_sim/library/rl_sdk/rl_sdk.cpp)
    target_link_libraries(rl_sdk "${TORCH_LIBRARIES}" yaml-cpp)
    set_property(TARGET rl_sdk PROPERTY CXX_STANDARD 17)


    add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/gazebo_sim/src/rl_sim.cpp
                        # ${PROJECT_SOURCE_DIR}/gazebo_sim/library/rl_sdk/rl_sdk.cpp
                        # ${dir_wbc}
                        # ${dir_est}
                        # ${dir_datapackage}
                        ) 
                           

    target_link_libraries(${PROJECT_NAME}
                                # pinocchio_default 
                                # pinocchio_parsers
                                yaml-cpp
                                torch
                                rl_sdk
                                # qpOASES 
                                # urdfdom_model
                                ${catkin_LIBRARIES})



endif()





# install(TARGETS DataPackage DESTINATION ${PROJECT_SOURCE_DIR}/lib)

