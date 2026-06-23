#include "RL_deploy_cpg/include/rl_deploy.h"
// #include "RL_deploy_cpg/include/CPGControl.h"
#include <cmath>  // 包含 sin、cos 等数学函数

#include <chrono>
#include <iostream>
#include <iomanip> // 用于控制打印精度


#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <libobsensor/ObSensor.hpp>


rl_deploy_cpg::rl_deploy_cpg()
{
    torch::autograd::GradMode::set_enabled(false);
    std::string file_path = "../RL_deploy_cpg/config/rl_params.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    bool enable_video = false;
    if (config["enable_video"]) {
        enable_video = config["enable_video"].as<bool>();
    }
    // load the policy
    policy_path = config["policy_path"].as<std::string>();
    policy = torch::jit::load(policy_path, torch::kCPU);
    std::cout << "valinia policy init complete" << std::endl;


    decimation = config["decimation"].as<int>();
    num_proprio = config["num_proprio"].as<int>();

    clip_obs = config["clip_obs"].as<double>();
    clip_actions_lower = config["clip_actions_lower"].as<double>();
    clip_actions_upper = config["clip_actions_upper"].as<double>();

    num_of_dofs = config["num_of_dofs"].as<int>();
    history_len = config["history_len"].as<int>();
    lin_vel_scale = config["lin_vel_scale"].as<double>();
    ang_vel_scale = config["ang_vel_scale"].as<double>();
    dof_pos_scale = config["dof_pos_scale"].as<double>();
    dof_vel_scale = config["dof_vel_scale"].as<double>();

    last_actions_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    outputdata_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    pos_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    vel_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    action_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    rot_hist_buf_eigen = Eigen::VectorXd::Zero(15);
    ang_vel_hist_buf_eigen = Eigen::VectorXd::Zero(15);


    // Convert std::vector<double> to Eigen::VectorXd
    std::vector<double> default_dof_pos_vec = ReadVectorFromYaml<double>(config["default_dof_pos"]);
    default_dof_pos_eigen = Eigen::Map<Eigen::VectorXd>(default_dof_pos_vec.data(), default_dof_pos_vec.size());
    
    std::vector<double> rl_p_vec = ReadVectorFromYaml<double>(config["rl_p"]);
    rl_p = Eigen::Map<Eigen::VectorXd>(rl_p_vec.data(), rl_p_vec.size());
    std::vector<double> rl_d_vec = ReadVectorFromYaml<double>(config["rl_d"]);
    rl_d = Eigen::Map<Eigen::VectorXd>(rl_d_vec.data(), rl_d_vec.size());


    std::vector<double> action_scale_vec = ReadVectorFromYaml<double>(config["action_scale"]);
    action_scale = Eigen::Map<Eigen::VectorXd>(action_scale_vec.data(), action_scale_vec.size());


    indices_to_remove = ReadVectorFromYaml<int>(config["useless_joint_id"]);

    // Initialize task_obs logging
    enable_task_obs_logging = config["enable_task_obs_logging"].as<bool>();
    if (enable_task_obs_logging) {
        std::string log_path = config["task_obs_log_path"].as<std::string>();
        InitTaskObsLogging(log_path);
    }

    lin_vel = Eigen::VectorXd::Zero(3);
    imu_angular_vel = Eigen::VectorXd::Zero(3);
    commands = Eigen::VectorXd::Zero(3);

    base_euler = Eigen::VectorXd::Zero(3);

    dof_pos = Eigen::VectorXd::Zero(num_of_dofs);
    dof_vel = Eigen::VectorXd::Zero(num_of_dofs);
    last_actions = Eigen::VectorXd::Zero(num_of_dofs);

    torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1});
    default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"])).view({1, -1});

    commands_scale = torch::tensor({lin_vel_scale, lin_vel_scale, ang_vel_scale});
    output_dof_pos = torch::zeros({1, num_of_dofs});
    inputdata_eigen = Eigen::VectorXd::Zero(num_observations);

    num_observations = num_proprio + num_of_dofs * 5 *2  ;
    inputdata_eigen = Eigen::VectorXd::Zero(num_observations);

    CPGInit(4, 0.02, float(1.0 / 0.8));
    CPGReset(true);

    ///////////////
    global_x = 0.0;
    global_y = 0.0;
    global_yaw = 0.0;
    //初始化 UDP
    InitUDP();
    ///////////////
    if (enable_video) {
        // =============== 新增：初始化视频 UDP Socket ===============
        if ((video_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            std::cerr << "Video Socket creation failed" << std::endl;
        }
        // 视频 socket 不需要 bind，因为它只负责发送 (Send Only)
        // =============== 初始化 Orbbec 相机 ===============
        try {
            // 创建 Pipeline，这是与相机交互的管道
            pipe = std::make_shared<ob::Pipeline>();
            ob_config = std::make_shared<ob::Config>();
            // 配置彩色流 (RGB)
            // 注意：根据你的相机型号，分辨率可能需要调整 (640x480 或 848x480)
            auto profiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
            std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
            try {
                // 尝试查找 848x480 @ 30fps RGB 格式
                colorProfile = profiles->getVideoStreamProfile(848, 480, OB_FORMAT_RGB, 30);
            } catch(...) {
                // 如果找不到，使用默认配置
                colorProfile = std::const_pointer_cast<ob::VideoStreamProfile>(profiles->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>());
            }
            ob_config->enableStream(colorProfile);

            // 配置深度流 (Depth)
            try {
                auto depthProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
                std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
                try {
                    // 尝试查找 848x480 @ 30fps Y16 格式 (对应你的 RGB 分辨率)
                    // 如果报错，可以尝试改为 640x480
                    depthProfile = depthProfiles->getVideoStreamProfile(848, 480, OB_FORMAT_Y16, 30);
                } catch(...) {
                    // 如果找不到指定分辨率，就用默认的
                    depthProfile = std::const_pointer_cast<ob::VideoStreamProfile>(depthProfiles->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>());
                }
                ob_config->enableStream(depthProfile);
                std::cout << "Depth Stream Enabled!" << std::endl;
            } catch(ob::Error &e) {
                std::cerr << "Depth Init Failed: " << e.getMessage() << std::endl;
            }
            // ====================================================
            // 启动相机
            pipe->start(ob_config);
            align_filter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            std::cout << "Orbbec Camera Started Successfully!" << std::endl;
            // =============== 启动视频发送线程 ===============
            video_running = true;
            video_thread = std::thread(&rl_deploy_cpg::VideoTask, this);
        } catch(ob::Error &e) {
            std::cerr << "Orbbec Init Failed: " << e.getMessage() << std::endl;
            video_running = false;
        } catch(...) {
            std::cerr << "Camera Init Failed (Unknown Error)" << std::endl;
            video_running = false;
        }
        // ========================================================
    } else {
        video_running = false;
    }


}


// rl_deploy_cpg::~rl_deploy_cpg() {
// }
rl_deploy_cpg::~rl_deploy_cpg() {
    // 停止视频线程
    video_running = false;
    if (video_thread.joinable()) {
        video_thread.join();
    }

    // 停止相机
    if (pipe) {
        try {
            pipe->stop();
        } catch(...) {}
    }
    
    if (video_sockfd > 0) {
        close(video_sockfd);
    }
}


void rl_deploy_cpg::GetDataFromPackage(DataPackage &DataPackage){

    imu_angular_vel = DataPackage.imu_angular_vel;
    gravity_vec_eigen = Euler_ZYXToGravityVec(DataPackage.imu_zyx);
    q_origin = DataPackage.motor_pos;
    dot_q_origin = DataPackage.motor_vel;
    control_mode =DataPackage.control_mode;
    if (control_mode == 0){
        js_vx_desire = DataPackage.js_vx_desire;
        js_vy_desire = DataPackage.js_vy_desire;
        js_OmegaZ_desire = DataPackage.js_OmegaZ_desire;
    }

}


void rl_deploy_cpg::SetDataToPackage(DataPackage &data)
{

    data.torq_desire.setZero() ;

    
    for(int i =0; i<num_of_dofs; i++)
    {
        data.motor_Pos_desire(i) = outputdata_eigen[i] * action_scale[i] + default_dof_pos_eigen[i];
    }

    
    data.motor_Vel_desire.setZero();
    data.motor_Torque_desire.setZero();

    data.sim_P = rl_p*1.0;
    data.sim_D = rl_d*1.0;
}



void rl_deploy_cpg::Exit()
{
    // clear the hist buf and the step counter
    pos_hist_buf_eigen.setZero();
    vel_hist_buf_eigen.setZero();

}


//////////////////////
void rl_deploy_cpg::InitUDP() {
    // 创建 UDP Socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    memset(&python_addr, 0, sizeof(python_addr));

    // C++ 端监听端口
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(8080);

    int rcvbuf_size = 1024;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));

    // 绑定 socket
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        return;
    }

    // 设置为非阻塞模式 (关键！防止卡死控制循环)
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    udp_initialized = true;
    std::cout << "UDP Socket Initialized on port 8080" << std::endl;
}

void rl_deploy_cpg::CommunicateWithPlanner(const DataPackage &data, double timestamp_sec) {
    if (!udp_initialized) return;

    CommandPacket cmd_pkg;
    // int n = recvfrom(sockfd, &cmd_pkg, sizeof(cmd_pkg), 0, 
    //                  (struct sockaddr *)&cliaddr, &len);
    int n;
    socklen_t client_addr_len = sizeof(cliaddr); 
    bool received_new_data = false;

    // 1. 接收 Python 发来的指令
    while (true)
    {
        n = recvfrom(sockfd, &cmd_pkg, sizeof(cmd_pkg), 0, 
                     (struct sockaddr *)&cliaddr, &client_addr_len);
        if (n > 0) {
            received_new_data = true;
            python_addr = cliaddr; 
            
            // std::cout << "Received Cmd from Python! Update Video Target." << std::endl;
            // =============== 更新视频发送的目标地址 ===============
            // 这是一个简单的锁，防止多线程竞争
            {
                std::lock_guard<std::mutex> lock(addr_mutex);
                video_dest_addr = cliaddr;
                video_dest_addr.sin_port = htons(8082); // [重要] 视频发送到 Python 的 8082 端口
            }
            // ==========================================================

            // [核心] 更新 CPG/RL 需要的期望速度
            if (control_mode != 0) {
                js_vx_desire = (double)cmd_pkg.vx;
                js_vy_desire = (double)cmd_pkg.vy;
                js_OmegaZ_desire = (double)cmd_pkg.omega_z;
            }

            // js_vx_desire = (double)cmd_pkg.vx;
            // js_vy_desire = (double)cmd_pkg.vy;
            // js_OmegaZ_desire = (double)cmd_pkg.omega_z;

        } else
        {
            break;
        }
        
    }

    // if (n > 0) {
    //     // 如果收到了数据，保存 Python 的地址以便回传
    //     python_addr = cliaddr; 
        
    //     // [核心] 更新 CPG/RL 需要的期望速度
    //     js_vx_desire = (double)cmd_pkg.vx;
    //     js_vy_desire = (double)cmd_pkg.vy;
    //     js_OmegaZ_desire = (double)cmd_pkg.omega_z;

    //     // std::cout << "Recv Cmd: " << js_vx_desire << " " << js_vy_desire << std::endl;
    // }

    // 2. 发送机器人状态回 Python
    // 如果没有，需要在这里做一个简单的死算 (Dead Reckoning) 积分，或者从 DataPackage 获取。
    
    Eigen::Vector4d quat_xyzw = data.mocap_baseQuat;
    Eigen::Quaterniond quat_wxyz(
        quat_xyzw[3],
        quat_xyzw[0],
        quat_xyzw[1],
        quat_xyzw[2]
    );
    quat_wxyz.normalize();

    Eigen::Vector3d robot_pos_w = data.mocap_basePos;
    Eigen::Vector3d linear_vel_w = Eigen::Vector3d::Zero();
    if (nav_state_prev_valid) {
        double dt = timestamp_sec - nav_state_prev_timestamp;
        if (std::isfinite(dt) && dt > 1e-4 && dt < 1.0) {
            linear_vel_w = (robot_pos_w - nav_state_prev_pos) / dt;
            if (!linear_vel_w.allFinite() || linear_vel_w.norm() > 10.0) {
                linear_vel_w.setZero();
            }
        }
    }
    nav_state_prev_pos = robot_pos_w;
    nav_state_prev_timestamp = timestamp_sec;
    nav_state_prev_valid = true;

    Eigen::Vector3d linear_vel_b = quat_wxyz.conjugate() * linear_vel_w;
    Eigen::Vector3d projected_gravity_b = quat_wxyz.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);

    NavStatePacketV2 state_pkg{};
    state_pkg.magic = 0x32555253;  // 'SRU2'
    state_pkg.version = 2;
    state_pkg.flags = 0;
    state_pkg.seq = nav_state_seq++;
    state_pkg.timestamp_sec = timestamp_sec;

    for (int i = 0; i < 3; ++i) {
        state_pkg.linear_vel_b[i] = static_cast<float>(linear_vel_b[i]);
        state_pkg.angular_vel_b[i] = static_cast<float>(data.imu_angular_vel[i]);
        state_pkg.projected_gravity_b[i] = static_cast<float>(projected_gravity_b[i]);
        state_pkg.robot_pos_w[i] = static_cast<float>(robot_pos_w[i]);
    }
    state_pkg.robot_quat_wxyz[0] = static_cast<float>(quat_wxyz.w());
    state_pkg.robot_quat_wxyz[1] = static_cast<float>(quat_wxyz.x());
    state_pkg.robot_quat_wxyz[2] = static_cast<float>(quat_wxyz.y());
    state_pkg.robot_quat_wxyz[3] = static_cast<float>(quat_wxyz.z());

    // 只有在接收过一次 Python 数据后，知道发给谁，才发送
    if (python_addr.sin_family != 0) {
        sendto(sockfd, &state_pkg, sizeof(state_pkg), 0, 
               (const struct sockaddr *)&python_addr, sizeof(python_addr));
    }
}

///////////////////////

void rl_deploy_cpg::Step(DataPackage &data)
{
    if (step_num == 0){
        step_num = 1;
    }
    else{
        step_num += 1;
    }

    Eigen::Vector4d base_quat_xyzw;
    base_quat_xyzw = data.mocap_baseQuat;
    base_quat = base_quat_xyzw;
    // Yaw 角
    double yaw_1 = std::atan2(2.0 * (base_quat_xyzw[3]*base_quat_xyzw[2] + base_quat_xyzw[0]*base_quat_xyzw[1]), 1.0 - 2.0 * (base_quat_xyzw[1]*base_quat_xyzw[1] + base_quat_xyzw[2]*base_quat_xyzw[2]));
    // global_yaw = yaw_1;

    // global_x = data.mocap_basePos(0);
    // global_y = data.mocap_basePos(1);
    // CommunicateWithPlanner();

    if(step_num % decimation == 0)
    {   
        CPGHopfOscillator();
        UpdateGaitGeneratorPattern(commands);

        // 获取当前时间点
        auto now = std::chrono::system_clock::now();
        // 转换为自 1970-01-01 以来的秒数 (double 类型)
        double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
        // 打印，保留 6 位小数 (微秒级)
        // std::cout << std::fixed << std::setprecision(6) << "CPP Timestamp: " << timestamp << std::endl;
    
        global_yaw = yaw_1;
        global_x = data.mocap_basePos(0);
        global_y = data.mocap_basePos(1);
        CommunicateWithPlanner(data, timestamp);

        // std::cout<<"js_v: "<<js_vx_desire<<","<<js_vy_desire<<","<<js_OmegaZ_desire<<"CPP Pos: " << global_x << ", " << global_y<<std::endl;
        commands(0) = js_vx_desire * lin_vel_scale;
        commands(1) = js_vy_desire * lin_vel_scale;
        commands(2) = js_OmegaZ_desire * ang_vel_scale;

        inputdata_eigen.segment(0, 3) = imu_angular_vel * ang_vel_scale * 0;
        inputdata_eigen.segment(3, 3) =  imu_angular_vel * ang_vel_scale ;     
        inputdata_eigen.segment(6, 3) =  gravity_vec_eigen ;
        inputdata_eigen.segment(9, 3) = commands;
        inputdata_eigen.segment(12, 4) = CPGGetXNorm();

        inputdata_eigen.segment(16, 4) = CPGGetYNorm();

        inputdata_eigen.segment(20, num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
        inputdata_eigen.segment(20 + num_of_dofs, num_of_dofs) = dot_q_origin * dof_vel_scale ;
        inputdata_eigen.segment(20 + 2*num_of_dofs, num_of_dofs) = last_actions_eigen;


        inputdata_eigen.segment(20 + 3 * num_of_dofs, 5 * num_of_dofs) = pos_hist_buf_eigen.tail(5 * num_of_dofs);
        inputdata_eigen.segment(20 + 3 * num_of_dofs +  5 * num_of_dofs, 5 * num_of_dofs) = vel_hist_buf_eigen.tail(5 * num_of_dofs);

        
        std::vector<torch::jit::IValue> inputs;
        torch::Tensor inputdata_tensor = torch::zeros({num_observations}).toType(torch::kFloat);

        auto input_accessor = inputdata_tensor.accessor<float, 1>();
        for (int i = 0; i < num_observations; i++) {
            input_accessor[i] = static_cast<float>(inputdata_eigen(i));
        }
        inputdata_tensor = inputdata_tensor.to(torch::kCPU);
        inputdata_tensor = inputdata_tensor.unsqueeze(0);
        inputs.push_back(inputdata_tensor);

        
        torch::Tensor output_tensor;

        output_tensor = policy.forward(inputs).toTensor();

        if (output_tensor.dim() > 1 && output_tensor.size(0) == 1){
            output_tensor = output_tensor.squeeze(0);
        }

        auto accessor = output_tensor.accessor<float, 1>();
        for (int i = 0; i < num_of_dofs; i++) {
            outputdata_eigen(i) = static_cast<double>(accessor[i]);
        }
        


        last_actions_eigen = outputdata_eigen;

        pos_hist_buf_eigen.head(pos_hist_buf_eigen.size() - num_of_dofs) = pos_hist_buf_eigen.tail(pos_hist_buf_eigen.size() - num_of_dofs);
        pos_hist_buf_eigen.tail(num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
        
        vel_hist_buf_eigen.head(vel_hist_buf_eigen.size() - num_of_dofs) = vel_hist_buf_eigen.tail(vel_hist_buf_eigen.size() - num_of_dofs);
        vel_hist_buf_eigen.tail(num_of_dofs) = dot_q_origin * dof_vel_scale;

        action_hist_buf_eigen.head(action_hist_buf_eigen.size() - num_of_dofs) = action_hist_buf_eigen.tail(action_hist_buf_eigen.size() - num_of_dofs);
        action_hist_buf_eigen.tail(num_of_dofs) = last_actions_eigen;



        rot_hist_buf_eigen.head(12) = rot_hist_buf_eigen.tail(12);
        rot_hist_buf_eigen.tail(3) = gravity_vec_eigen;

        ang_vel_hist_buf_eigen.head(12) = ang_vel_hist_buf_eigen.tail(12);
        ang_vel_hist_buf_eigen.tail(3) = imu_angular_vel * ang_vel_scale;

        episode_length ++;

        ///////////
        // StatePacket state_pkg;
        // state_pkg.x = (float)global_x;
        // state_pkg.y = (float)global_y;
        // state_pkg.yaw = (float)global_yaw;
        // if (udp_initialized && python_addr.sin_family != 0) {
        //     sendto(sockfd, &state_pkg, sizeof(state_pkg), 0, 
        //            (const struct sockaddr *)&python_addr, sizeof(python_addr));

        // }
        ///////////

        // log
        LogTaskObs();

    }
}


void rl_deploy_cpg::CPGInit(int num_feet, float dt, float fre) {
    step_dt = dt;
    gait_cy = fre;

    cycle_r = Eigen::Vector4d::Ones();
    px = Eigen::Vector4d::Ones();
    py = Eigen::Vector4d::Zero();

    phase_offset = Eigen::VectorXd::Zero(num_feet);
    phase_offset_target = Eigen::VectorXd::Zero(num_feet);
    phase_offset_change_rate = 0.01 * (step_dt / 0.0005);

    CPGSetP(phase_offset);

    phase_offset_stand << 0.0, (0.0) * M_PI, 0.0, (0.0) * M_PI;
    phase_offset_walk << 0.0, (1.0) * M_PI, (-0.15) * M_PI, (-1.15) * M_PI;
    phase_offset_run << 0.0, (1.0) * M_PI, (-0.16) * M_PI, (-1.16) * M_PI;

    coupling_change_rate = Eigen::Vector4d::Ones(num_feet) * (2*2);
    coupling_change_rate_increase = 1.0 * (2*2);
    coupling_change_rate_decrease = 1.0 * (0.3+0.7);

    amplitude_change_rate = Eigen::Vector4d::Ones(num_feet);
    amplitude_change_rate_increase = std::pow(1.0 * (1+0.05), (step_dt / 0.0005));
    amplitude_change_rate_decrease = std::pow(1.0 * (1-0.005), (step_dt / 0.0005));

    CPGSetC(Eigen::VectorXd::Ones(4) * 0.5);

}

Eigen::VectorXd rl_deploy_cpg::CPGGammaCal() {
    Eigen::VectorXd term1 = Eigen::VectorXd::Zero(num_feet);
    Eigen::VectorXd term2 = Eigen::VectorXd::Zero(num_feet);
    for (int i = 0; i < num_feet; i++) {
        term1(i) = M_PI / ((1 - contact_ratio(i)) * (std::exp((-1) * coff_b * py(i)) + 1));
        term2(i) = M_PI / ((0 + contact_ratio(i)) * (std::exp(coff_b * py(i)) + 1));

    }
    return term1 + term2;
}

Eigen::MatrixXd rl_deploy_cpg::CPGCoupling() {
    Eigen::VectorXd coupling_x = Eigen::VectorXd::Zero(num_feet);
    Eigen::VectorXd coupling_y = Eigen::VectorXd::Zero(num_feet);
    double coupling_x_ij = 0.0;
    double coupling_y_ij = 0.0;
    double r_i = 0.0;
    double r_j = 0.0;
    
    for (int i = 0; i < num_feet; i++) {
        coupling_x(i) = 0.0;
        coupling_y(i) = 0.0;
        for (int j = 0; j < num_feet; j++) {
            if (i == j) {
                continue;
            }
            coupling_x_ij = std::cos(phase_offset(i) - phase_offset(j)) * px(j) 
                         - std::sin(phase_offset(i) - phase_offset(j)) * py(j);
            coupling_y_ij = std::sin(phase_offset(i) - phase_offset(j)) * px(j) 
                         + std::cos(phase_offset(i) - phase_offset(j)) * py(j);
            
            r_i = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
            r_j = std::sqrt(std::pow(px(j), 2) + std::pow(py(j), 2));
            
            coupling_x_ij = coupling_x_ij / r_j * r_i;
            coupling_y_ij = coupling_y_ij / r_j * r_i;
            
            coupling_x(i) += coupling_x_ij;
            coupling_y(i) += coupling_y_ij;
        }
    }
    
    Eigen::MatrixXd result(num_feet, 2);
    result.col(0) = coupling_x;
    result.col(1) = coupling_y;
    return result;
}

void rl_deploy_cpg::CPGPhaseChange() {
    phase_offset += phase_offset_change_rate * (phase_offset_target - phase_offset);
}

void rl_deploy_cpg::CPGHopfOscillator() {
    Eigen::Vector4d r;

    for (int i = 0; i < num_feet; i++) {
        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
    }
    
    Eigen::Vector4d gamma = CPGGammaCal();
    double alpha = gait_cy * rate;
    Eigen::Vector4d omega = gait_cy * gamma;
    
    CPGPhaseChange();
    Eigen::MatrixXd coupling_xy = CPGCoupling();
    Eigen::Vector4d coupling_x = coupling_xy.col(0);
    Eigen::Vector4d coupling_y = coupling_xy.col(1);

    Eigen::Vector4d dx;
    Eigen::Vector4d dy;
    
    for (int i = 0; i < num_feet; i++) {
        dx(i) = (alpha * (std::pow(cycle_r(i), 2) - std::pow(r(i), 2)) * px(i) - omega(i) * py(i)) 
              + coupling_change_rate(i) * coupling_x(i);
        dy(i) = (alpha * (std::pow(cycle_r(i), 2) - std::pow(r(i), 2)) * py(i) + omega(i) * px(i)) 
              + coupling_change_rate(i) * coupling_y(i);

        px(i) = px(i) + dx(i) * step_dt;
        py(i) = py(i) + dy(i) * step_dt;

        if (r(i) > cycle_r_min) {
            px(i) = px(i) * amplitude_change_rate(i);
            py(i) = py(i) * amplitude_change_rate(i);
        }

        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
        if (r(i) > cycle_r_max) {
            px(i) = px(i) / r(i);
            py(i) = py(i) / r(i);
        }
    }
    
}


void rl_deploy_cpg::CPGReset(bool flag) {
    if(flag) {
        px << std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI);
        py << std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI);
    } else {
        px << std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI);
        py << std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI);          
    }
    CPGSetP(phase_offset_target);
}

void rl_deploy_cpg::CPGSetCy(Eigen::VectorXd Cycle_r) {
    cycle_r = Cycle_r;
}

void rl_deploy_cpg::CPGSetCou(Eigen::VectorXd Coupling_change_rate) {
    coupling_change_rate = Coupling_change_rate;
}

void rl_deploy_cpg::CPGSetA(Eigen::VectorXd Amplitude_change_rate) {
    amplitude_change_rate = Amplitude_change_rate;
}

void rl_deploy_cpg::CPGSetP(Eigen::VectorXd Phase_offset) {
    phase_offset_target = Phase_offset;
}

void rl_deploy_cpg::CPGSetC(Eigen::VectorXd Contact_ratio) {
    contact_ratio = Contact_ratio;
}

void rl_deploy_cpg::CPGSetncy(double fre) {
    gait_cy = fre;
}


void rl_deploy_cpg::CPGSetSn() {
    CPGSetncy((1));
    CPGSetCy(Eigen::Vector4d::Ones(4) * cycle_r_min);
    CPGSetCou(Eigen::Vector4d::Ones(4) * coupling_change_rate_decrease);
    CPGSetA(Eigen::Vector4d::Ones(4) * amplitude_change_rate_decrease);
    CPGSetP(phase_offset_stand);
    CPGSetC(Eigen::Vector4d::Ones(4) * 0.5);

}

void rl_deploy_cpg::CPGSetWP_1() {
    CPGSetncy((1.25));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.5);

    
}
void rl_deploy_cpg::CPGSetWP_2() {
    CPGSetncy((1.25));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.4);

    
}
void rl_deploy_cpg::CPGSetWP_3() {
    CPGSetncy((1.4286));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.4);

    
}
void rl_deploy_cpg::CPGSetWP_4() {
    CPGSetncy((1.4286));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.3);

    
}
void rl_deploy_cpg::CPGSetWP_5() {
    CPGSetncy((1.4286));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.3);

    
}
void rl_deploy_cpg::CPGSetWP_6() {
    CPGSetncy((1.667));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.3);

    
}
void rl_deploy_cpg::CPGSetWP_7() {
    CPGSetncy((1.667));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.3);

    
}
void rl_deploy_cpg::CPGSetWP_8() {
    CPGSetncy((2));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.3);

    
}


Eigen::VectorXd rl_deploy_cpg::CPGGetXNorm() {
    Eigen::Vector4d r(num_feet);
    Eigen::Vector4d x_norm(num_feet);
    for (int i = 0; i < num_feet; i++) {
        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
        x_norm(i) = (r(i) > cycle_r_min) ? px(i) : 0.0;
    }
    return x_norm;
}

Eigen::VectorXd rl_deploy_cpg::CPGGetYNorm() {
    Eigen::Vector4d r(num_feet);
    Eigen::Vector4d y_norm(num_feet);
    for (int i = 0; i < num_feet; i++) {
        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
        y_norm(i) = (r(i) > cycle_r_min) ? py(i) : 0.0;
    }
    return y_norm;
}


void rl_deploy_cpg::UpdateGaitGeneratorPattern(Eigen::VectorXd Commands) {

    double vno = Commands.norm();
    if (vno < flag_1)
    {

        CPGSetSn();
    }

    if (vno >= flag_1 and vno < flag_2) 
    {

        CPGSetWP_1();
    }
    else if (vno >= flag_2 and vno < flag_3) 
    {

        CPGSetWP_2();
    }
    else if (vno >= flag_3 and vno < flag_4) 
    {

        CPGSetWP_3();
    }
    else if (vno >= flag_4 and vno < flag_5)
    {
        CPGSetWP_4();
    }
    else if (vno >= flag_5 and vno < flag_6)
    {
        CPGSetWP_5();
    }
    else if (vno >= flag_6 and vno < flag_7)
    {
        CPGSetWP_6();
    }
    else if (vno >= flag_7 and vno < flag_8)
    {
        CPGSetWP_7();
    }
    else if (vno >= flag_8 and vno < flag_9)
    {
        CPGSetWP_8();
    }
}

void rl_deploy_cpg::CommandRefinement() {

    Eigen::VectorXd Commands_buf = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd last_Commands_buf ;

}

Eigen::VectorXd rl_deploy_cpg::remove_indices(const Eigen::VectorXd& input, const std::vector<int>& indices_to_remove) 
{
    int original_size = input.size();

    // 构建保留的索引列表
    std::vector<int> keep_indices;
    for (int i = 0; i < original_size; ++i) {
        if (std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) {
            keep_indices.push_back(i);
        }
    }
    // 构造新向量
    Eigen::VectorXd output(keep_indices.size());
    for (size_t i = 0; i < keep_indices.size(); ++i) {
        output[i] = input[keep_indices[i]];
    }

    return output;
}


Eigen::Vector3d rl_deploy_cpg::Euler_ZYXToGravityVec(Eigen::Vector3d euler_a) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d grav_vec = -R.transpose().col(2);
    return grav_vec;
}


void rl_deploy_cpg::assign_with_skipped_zero(
    Eigen::VectorXd& target,
    const Eigen::VectorXd& source,
    const std::vector<int>& indices_to_skip)
{
    int target_size = target.size();
    int source_size = source.size();

    std::unordered_set<int> skip_set(indices_to_skip.begin(), indices_to_skip.end());

    int source_idx = 0;

    for (int i = 0; i < target_size; ++i) {
        if (skip_set.count(i)) {
            target[i] = 0.0;  // 跳过位置赋0
        } else {
            if (source_idx < source_size) {
                target[i] = source[source_idx++];
            } else {
                target[i] = 0.0;  // 如果 source 不够长，剩余位置补0（可选）
            }
        }
    }
    // 可选：保证 source 完全用完
    assert(source_idx == source_size && "Source vector is longer than available non-skipped slots");
}


// Task observation logging functions
void rl_deploy_cpg::InitTaskObsLogging(const std::string& log_file_path) {
    try {
        // Create logs directory if it doesn't exist
        std::string dir_path = "../RL_deploy_cpg/logs/";
        system(("mkdir -p " + dir_path).c_str());

        task_obs_log_file.open(log_file_path, std::ios::out | std::ios::trunc);
        if (!task_obs_log_file.is_open()) {
            std::cerr << "Failed to open task_obs log file: " << log_file_path << std::endl;
            enable_task_obs_logging = false;
            return;
        }

        // Write CSV header
        task_obs_log_file << "js_vx_desire,js_vy_desire,js_OmegaZ_desire,global_x,global_y,global_yaw";
        // for (int i = 0; i < num_observations; ++i) {
        //     task_obs_log_file << ",proprio_obs_" << i;
        // }
        task_obs_log_file << std::endl;

        std::cout << "Task observation logging initialized: " << log_file_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing task_obs logging: " << e.what() << std::endl;
        enable_task_obs_logging = false;
    }
}


void rl_deploy_cpg::LogTaskObs() {
    if (!enable_task_obs_logging || !task_obs_log_file.is_open()) {
        return;
    }

    try {
        // Convert tensor to CPU and double precision for logging
        // torch::Tensor task_obs_cpu = task_obs_tensor.to(torch::kCPU).to(torch::kDouble);

        // Write step info
        // task_obs_log_file << step_num << "," << frame_count << "," << control_mode;

        // Write task_obs values
        // auto accessor = task_obs_cpu.accessor<double, 2>();
        // for (int i = 0; i < num_observations; ++i) {
        //     task_obs_log_file << " " << std::fixed << std::setprecision(6) << accessor[0][i];
        // }
        task_obs_log_file <<js_vx_desire<<","<<js_vy_desire<<","<<js_OmegaZ_desire<<","<<global_x<<","<<global_y<<","<<global_yaw;
        task_obs_log_file << "\n";

        // Flush every 100 steps to ensure data is written
        if (step_num % 100 == 0) {
            task_obs_log_file.flush();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error logging task_obs: " << e.what() << std::endl;
    }
}


void rl_deploy_cpg::CloseTaskObsLogging() {
    if (task_obs_log_file.is_open()) {
        task_obs_log_file.close();
        std::cout << "Task observation logging closed." << std::endl;
    }
}


Eigen::MatrixXd rl_deploy_cpg::LoadMotionLib(const std::string& filename, bool skipHeader) {
    std::ifstream file(filename);
    std::string line;
    std::vector<std::vector<double>> data;

    // read w/o process
    if (skipHeader) {
        std::getline(file, line);
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (...) {
                row.push_back(0.0);  // 可以自定义处理方式
            }
        }

        if (!row.empty())
            data.push_back(row);
    }

    if (data.empty()) {
        throw std::runtime_error("No data found in file: " + filename);
    }

    int rows = data.size();
    int cols = data[0].size();
    num_motion_frames = rows;
    Eigen::MatrixXd mat(rows, cols);

    for (int i = 0; i < rows; ++i)
        mat.row(i) = Eigen::VectorXd::Map(&data[i][0], cols);
    std::cout << "motion lib loaded !!!" << std::endl;

    return mat;
}

void rl_deploy_cpg::VideoTask() 
{
    std::cout << "Video Thread Started (Aligned RGB-D)." << std::endl;
    
    // ------------------- 1. 压缩参数配置 -------------------
    // RGB 使用 JPEG, 质量 50 (平衡画质与带宽)
    std::vector<int> rgb_params;
    rgb_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    rgb_params.push_back(30); 

    // Depth 使用 PNG (无损), 压缩级别 1 (追求速度)
    std::vector<int> depth_params;
    depth_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    depth_params.push_back(1); 

    long frame_count = 0;
    while (video_running) 
    {
        // ------------------- 2. 检查目标 IP -------------------
        bool has_target = false;
        struct sockaddr_in target_addr_base;
        {
            std::lock_guard<std::mutex> lock(addr_mutex);
            if (video_dest_addr.sin_family != 0) {
                target_addr_base = video_dest_addr;
                has_target = true;
            }
        }

        if (!has_target) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue; 
        }

        // ------------------- 3. 获取并对齐帧 -------------------
        auto frameSet = pipe->waitForFrames(100);
        if (frameSet == nullptr) continue;

        frame_count ++ ;
        if (frame_count % 30 != 0)continue;
        // 【关键】执行 D2C 对齐：将深度图对齐到彩色图
        // process 方法会返回一个新的 frameSet，其中的 Depth 已经被变换
        if (frameSet->colorFrame() && frameSet->depthFrame()) {
            try {
                auto alignedFrame = align_filter->process(frameSet);
                // frameSet = align_filter->process(frameSet);
                frameSet = alignedFrame->as<ob::FrameSet>();
            } catch (ob::Error &e) {
                std::cerr << "Align Error: " << e.getMessage() << std::endl;
                continue;
            }
        } else {
            continue; // 缺帧则跳过
        }

        // ------------------- 4. 处理 RGB (端口 8082) -------------------
        auto colorFrame = frameSet->colorFrame();
        if (colorFrame != nullptr) {
            // Orbbec SDK 默认 RGB -> OpenCV BGR
            cv::Mat frame(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
            cv::Mat bgr_frame;
            cv::cvtColor(frame, bgr_frame, cv::COLOR_RGB2BGR);
            std::vector<uchar> buffer;
            cv::imencode(".jpg", bgr_frame, buffer, rgb_params);

            // UDP 发送限制判断
            if (buffer.size() < 60000) {
                struct sockaddr_in rgb_addr = target_addr_base;
                rgb_addr.sin_port = htons(8082); 
                // sendto(video_sockfd, buffer.data(), buffer.size(), 0,
                //        (const struct sockaddr *)&rgb_addr, sizeof(rgb_addr));
                int sent_bytes = sendto(video_sockfd, buffer.data(), buffer.size(), 0,
                   (const struct sockaddr *)&rgb_addr, sizeof(rgb_addr));
                if (sent_bytes < 0) {
                    // perror("RGB Sendto Failed"); // 如果发送失败，打印系统错误原因
                }
            }
        }

        // ------------------- 5. 处理 Depth (端口 8083) -------------------
        auto depthFrame = frameSet->depthFrame();
        if (depthFrame != nullptr) {
            // 对齐后，depth 分辨率 = color 分辨率 (例如 848x480)
            cv::Mat depthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());

            for(int r = 0; r < depthMat.rows; r++){
                uint16_t* ptr = depthMat.ptr<uint16_t>(r);
                for(int c =0;c<depthMat.cols;c++){
                    if(ptr[c]>0){
                        ptr[c]=ptr[c] & 0xFFF0;
                    }
                    if(ptr[c] > 6000){
                        ptr[c] = 6000;
                    }
                }
            }

            // cv::Mat depthFiltered;
            // cv::threshold(depthMat, depthFiltered,8000,0, cv::THRESH_TOZERO_INV);

            // for(int r = 0; r < depthFiltered.rows; r++){
            //     uint16_t* ptr = depthFiltered.ptr<uint16_t>(r);
            //     for(int c =0;c<depthFiltered.cols;c++){
            //         if(ptr[c]>0){
            //             ptr[c]=ptr[c] & 0xFFF0;
            //         }
            //     }
            // }


            cv::Mat depthSmall;
            cv::resize(depthMat, depthSmall, cv::Size(), 0.6, 0.6, cv::INTER_NEAREST);

            std::vector<uchar> depth_buffer;
            cv::imencode(".png", depthSmall, depth_buffer, depth_params);

            if (depth_buffer.size() < 600000) {
                struct sockaddr_in depth_addr = target_addr_base;
                depth_addr.sin_port = htons(8083);
                sendto(video_sockfd, depth_buffer.data(), depth_buffer.size(), 0,
                       (const struct sockaddr *)&depth_addr, sizeof(depth_addr));
            }
        }
    }
}
