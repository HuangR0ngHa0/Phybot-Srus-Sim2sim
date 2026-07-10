#include "MotorList.hpp"

MotorList::MotorList() : Motors_Map(nullptr)  {}

MotorList::~MotorList() {
    for (auto& filter : lowpass_) {
        delete filter;
    }
    lowpass_.clear();
}

void MotorList::Init(std::string path1, std::string path2, const DataPackage &DataPackage) {

    if (!MotorControl->LoadYAMLConfigFromFile(path1, path2)) {
        std::cerr << "Error: LoadYAMLConfigFromFile Failed for path: " << path1 << std::endl;
        throw std::runtime_error("Failed to load motor configuration");
    }





    // Motors_Map = MotorControl->GetMotorsMap();
    Motors_Map = std::shared_ptr<std::map<std::string, std::shared_ptr<MotorDriver>>>(
        MotorControl->GetMotorsMap(),
        [](auto*) {} // 空删除器，因为所有权在MotorsControl
    );


    num_motors = Motors_Map->size();

    std::vector<std::shared_ptr<MotorDriver>> motor_ptrs;
    std::vector<std::string> motor_names;
    for (const auto& Motor : *Motors_Map) {
        motor_ptrs.push_back(Motor.second);
        motor_names.push_back(Motor.first);
    }

    motor.reserve(num_motors);
    abs_zero.setZero(num_motors);
    
    qpos_cur.resize(num_motors);
    qvel_cur.resize(num_motors);
    qtor_cur.resize(num_motors);
    motor_lowpass.resize(num_motors);

    P_control_vector.resize(num_motors);
    D_control_vector.resize(num_motors);
    direction_vector.resize(num_motors);



    YAML::Node abs_zero_config = YAML::LoadFile("../MotorList/config/phybot_abszero.yaml");

    auto numbers_node = abs_zero_config["abs_zero"];
    if (!numbers_node || !numbers_node.IsSequence()) {
        std::cerr << "abs_zero不存在" << std::endl;
        // return ;
    }

    else
    {

        // 创建 Eigen 向量
        for (size_t i = 0; i < numbers_node.size(); ++i) {
            abs_zero(i) = numbers_node[i].as<double>();  // 直接赋值到 Eigen 向量
        }

    }



    std::cout<<"abs_zero: "<<abs_zero<<std::endl;

    lowpass_.resize(num_motors);
    for (int i = 0; i < num_motors; i++) {
        lowpass_[i] = new LowPassFilter(10.0, 0.707, DataPackage.control_period, 1);
    }

    Eigen::VectorXd p_params = MotorControl->GetAllPDKpVector();
    Eigen::VectorXd d_params = MotorControl->GetAllPDKdVector();
    Eigen::VectorXd dir_params = MotorControl->GetAllDirectionVector();

    if (p_params.size() != num_motors || d_params.size() != num_motors || dir_params.size() != num_motors) {
        throw std::runtime_error("Parameter size mismatch");
    }

    P_control_vector = p_params;
    D_control_vector = d_params;
    direction_vector = dir_params;


    for (int i = 0; i < num_motors; ++i) {
        std::cout << motor_names[i] << "\t\t" 
                << P_control_vector(i) << "\t\t"
                << D_control_vector(i) << "\t\t"
                << direction_vector(i) << std::endl;
    }
    std::cout << std::endl;

    int i = 0;
    for (const auto& motorPair : *Motors_Map) {
        float Pos, Vel, Tor;
        motorPair.second->GetPVCTFast(Pos, Vel, Tor);

        i++;
    }



    std::this_thread::sleep_for(std::chrono::seconds(1));

}


void MotorList::Init(std::string path1, std::string path2, std::string path3, const DataPackage &DataPackage) {

    if (!MotorControl->LoadYAMLConfigFromFile(path1, path2, path3)) {
        std::cerr << "Error: LoadYAMLConfigFromFile Failed for path: " << path1 << std::endl;
        throw std::runtime_error("Failed to load motor configuration");
    }





    // Motors_Map = MotorControl->GetMotorsMap();
    Motors_Map = std::shared_ptr<std::map<std::string, std::shared_ptr<MotorDriver>>>(
        MotorControl->GetMotorsMap(),
        [](auto*) {} // 空删除器，因为所有权在MotorsControl
    );


    num_motors = Motors_Map->size();

    std::vector<std::shared_ptr<MotorDriver>> motor_ptrs;
    std::vector<std::string> motor_names;
    for (const auto& Motor : *Motors_Map) {
        motor_ptrs.push_back(Motor.second);
        motor_names.push_back(Motor.first);
    }

    motor.reserve(num_motors);
    abs_zero.setZero(num_motors);
    
    qpos_cur.resize(num_motors);
    qvel_cur.resize(num_motors);
    qtor_cur.resize(num_motors);
    motor_lowpass.resize(num_motors);

    P_control_vector.resize(num_motors);
    D_control_vector.resize(num_motors);
    direction_vector.resize(num_motors);



    YAML::Node abs_zero_config = YAML::LoadFile("../MotorList/config/phybot_abszero.yaml");

    auto numbers_node = abs_zero_config["abs_zero"];
    if (!numbers_node || !numbers_node.IsSequence()) {
        std::cerr << "abs_zero不存在" << std::endl;
        // return ;
    }

    else
    {

        // 创建 Eigen 向量
        for (size_t i = 0; i < numbers_node.size(); ++i) {
            abs_zero(i) = numbers_node[i].as<double>();  // 直接赋值到 Eigen 向量
        }

    }



    std::cout<<"abs_zero: "<<abs_zero<<std::endl;

    lowpass_.resize(num_motors);
    for (int i = 0; i < num_motors; i++) {
        lowpass_[i] = new LowPassFilter(10.0, 0.707, DataPackage.control_period, 1);
    }

    Eigen::VectorXd p_params = MotorControl->GetAllPDKpVector();
    Eigen::VectorXd d_params = MotorControl->GetAllPDKdVector();
    Eigen::VectorXd dir_params = MotorControl->GetAllDirectionVector();

    if (p_params.size() != num_motors || d_params.size() != num_motors || dir_params.size() != num_motors) {
        throw std::runtime_error("Parameter size mismatch");
    }

    P_control_vector = p_params;
    D_control_vector = d_params;
    direction_vector = dir_params;


    for (int i = 0; i < num_motors; ++i) {
        std::cout << motor_names[i] << "\t\t" 
                << P_control_vector(i) << "\t\t"
                << D_control_vector(i) << "\t\t"
                << direction_vector(i) << std::endl;
    }
    std::cout << std::endl;

    int i = 0;
    for (const auto& motorPair : *Motors_Map) {
        float Pos, Vel, Tor;
        motorPair.second->GetPVCTFast(Pos, Vel, Tor);

        i++;
    }



    std::this_thread::sleep_for(std::chrono::seconds(1));

}



bool MotorList::Enable() {
    if (!Motors_Map) {
        std::cerr << "Error: Motors map not initialized" << std::endl;
        return false;
    }
    if (Motors_Map->size() != num_motors) {
        std::cerr << "Error: Motors map size (" << Motors_Map->size() 
                  << ") does not match expected number of motors (" << num_motors << ")" << std::endl;
        return false;
    }

    constexpr auto timeout = std::chrono::milliseconds(100000);
    constexpr auto retryInterval = std::chrono::milliseconds(1);
    constexpr auto individualRetryDelay = std::chrono::milliseconds(1);
    
    auto startTime = std::chrono::steady_clock::now();
    bool allEnabled = false;

    while (!allEnabled) {
        allEnabled = true;

        // if (std::chrono::steady_clock::now() - startTime >= timeout) {
        //     std::cerr << "Error: Failed to enable all motors within " 
        //               << timeout.count() << " milliseconds" << std::endl;
        //     return false;
        // }

        for (const auto& [motorId, motor] : *Motors_Map) {
            bool success = false;
            while (!success) {
                success = motor->SetControlWord(ControlWord_e::CTRL_SERVO_ON);
                
                if (!success) {
                    allEnabled = false;
                    std::cout << "*****************\n"
                              << "Motor failed: " << motorId << std::endl;
                    std::this_thread::sleep_for(individualRetryDelay);
                } else {
                    std::cout << motorId << " SetControlWord Enable Success..." << std::endl;
                }
            }
        }

        if (!allEnabled) {

            
            std::this_thread::sleep_for(retryInterval);
        }
    }

    std::cout << "All motors enabled successfully" << std::endl;
    return true;
}


bool MotorList::SetZero() {
    if (!Motors_Map) {
        std::cerr << "Error: Motors map not initialized" << std::endl;
        return false;
    }
    if (Motors_Map->size() != num_motors) {
        std::cerr << "Error: Motors map size (" << Motors_Map->size() 
                  << ") does not match expected number of motors (" << num_motors << ")" << std::endl;
        return false;
    }

    constexpr auto timeout = std::chrono::milliseconds(100000);
    constexpr auto retryInterval = std::chrono::milliseconds(1);
    constexpr auto individualRetryDelay = std::chrono::milliseconds(1);
    
    auto startTime = std::chrono::steady_clock::now();
    bool allEnabled = false;

    while (!allEnabled) {
        allEnabled = true;

        // if (std::chrono::steady_clock::now() - startTime >= timeout) {
        //     std::cerr << "Error: Failed to enable all motors within " 
        //               << timeout.count() << " milliseconds" << std::endl;
        //     return false;
        // }

        for (const auto& [motorId, motor] : *Motors_Map) {
            bool success = false;
            while (!success) {
                success = motor->SetControlWord(ControlWord_e::CTRL_POSITION_SET_ZERO);
                
                if (!success) {
                    allEnabled = false;
                    std::cout << "*****************\n"
                              << "Motor failed: " << motorId << std::endl;
                    std::this_thread::sleep_for(individualRetryDelay);
                } else {
                    std::cout << motorId << " SetControlWord SetZero Success..." << std::endl;
                }
            }
        }

        if (!allEnabled) {

            
            std::this_thread::sleep_for(retryInterval);
        }
    }

    std::cout << "All motors setzero successfully" << std::endl;
    return true;
}


// void MotorList::CloseUdp() {
//     MotorControl->CloseMainBoardUdp();

// }

void MotorList::SetCommands(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd, Eigen::VectorXd dir) {

    
    if (!Motors_Map || Motors_Map->size() != num_motors) {
        throw std::runtime_error("Invalid motors map");
    }


    if (dir.size() != num_motors) {
        throw std::invalid_argument("Direction vector size does not match motor count");
    }


    int i = 0;
    for (const auto& motorPair : *Motors_Map) {

        // if(i == 0)
        // {
        //     std::cout<<"pos_desire2: "<<(pos_cmd[i] )* dir[i] * 1  +  abs_zero(i)<<std::endl;

        // }
        // std::cout << "((((((((((((((((()))))))))))))))))" << std::endl;
        // std::cout << "I : "<<i<<"  set name: " << motorPair.first << std::endl;
        // std::cout << "I : "<<i<<"  set id: " << motorPair.second->Id << std::endl;
        // std::cout << "set cur pos: " << pos_cmd[i] << std::endl;
        // cout << "Name: " << Motor.first << " Pos: " << Pos << " Vel: " << Vel << " Tor: " << Tor << endl;

        // std::cout << "set cur dir: " << dir[i] << std::endl;
        // std::cout << "set desire pos: " << pos_cmd[i] * dir[i] << std::endl;
        motorPair.second->SetBigparam((pos_cmd[i] )* dir[i] * 1  +  abs_zero(i), vel_cmd[i] * dir[i] * 0,  0 * tor_cmd[i] * dir[i]);
        i++;
    }

    MotorControl->GetMotorNet2()->setPlanningPose();
    MotorControl->GetMotorNet1()->setPlanningPose();
    // MotorControl->GetMotorNet3()->setPlanningPose();
    // std::this_thread::sleep_for(std::chrono::microseconds(10));
    

    // if (auto net1 = MotorControl->GetMotorNet1()) {
    //     net1->setPlanningPose();
    // }
    // if (auto net2 = MotorControl->GetMotorNet2()) {
    //     net2->setPlanningPose();
    // }
}

void MotorList::SetCommandsFromPackage(Eigen::VectorXd& pos_cmd, Eigen::VectorXd& vel_cmd, Eigen::VectorXd& tor_cmd, 
                          Eigen::VectorXd dir, DataPackage &data) {
    pos_cmd = data.motor_Pos_desire;
    vel_cmd = data.motor_Vel_desire;
    tor_cmd = data.motor_Torque_desire;

    if (!Motors_Map || Motors_Map->size() != num_motors) {
        throw std::runtime_error("Invalid motors map");
    }


    if (dir.size() != num_motors) {
        throw std::invalid_argument("Direction vector size does not match motor count");
    }


    int i = 0;
    for (const auto& motorPair : *Motors_Map) {

        // std::cout << "((((((((((((((((()))))))))))))))))" << std::endl;
        // std::cout << "set name: " << motorPair.first << std::endl;
        // std::cout << "set cur pos: " << pos_cmd[i] << std::endl;
        // std::cout << "set cur dir: " << dir[i] << std::endl;
        // std::cout << "set desire pos: " << pos_cmd[i] * dir[i] << std::endl;
        motorPair.second->SetBigparam((pos_cmd[i] )* dir[i] * 1  +  abs_zero(i), vel_cmd[i] * dir[i] * 0,  0 * tor_cmd[i] * dir[i]);
        i++;
    }

    MotorControl->GetMotorNet2()->setPlanningPose();
    MotorControl->GetMotorNet1()->setPlanningPose();
    // MotorControl->GetMotorNet3()->setPlanningPose();

    // std::this_thread::sleep_for(std::chrono::microseconds(10));
    

    // if (auto net1 = MotorControl->GetMotorNet1()) {
    //     net1->setPlanningPose();
    // }
    // if (auto net2 = MotorControl->GetMotorNet2()) {
    //     net2->setPlanningPose();
    // }
    
}



void MotorList::GetStates(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor, Eigen::VectorXd dir) {
    
    // 1. 检查输入向量大小
    if (pos.size() != num_motors || vel.size() != num_motors || 
        tor.size() != num_motors || dir.size() != num_motors) {
        throw std::invalid_argument("State vector size does not match motor count");
    }
    
    // 2. 检查Motors_Map是否有效
    if (!Motors_Map || Motors_Map->size() != num_motors) {
        throw std::runtime_error("Invalid motors map");
    }
    

    // 3. 安全遍历电机
    int i = 0;
    for (const auto& motorPair : *Motors_Map) {
        // 检查电机驱动对象是否有效
        if (!motorPair.second) {
            throw std::runtime_error("Invalid motor driver at index " + std::to_string(i));
        }
        
        float Pos, Vel, Tor;
        motorPair.second->GetPVCTFast(Pos, Vel, Tor);



   
        pos[i] = (Pos - abs_zero(i)) * direction_vector[i];
        vel[i] = Vel * direction_vector[i];
        tor[i] = Tor * direction_vector[i];


        // std::cout << "1111111111111111" << std::endl;
        // std::cout << "get name: " << motorPair.first << std::endl;
        // std::cout << "get cur pos: " << Pos << std::endl;
        // std::cout << "get cur dir: " << dir[i] << std::endl;
        // std::cout << "get desire pos: " << pos[i] << std::endl;

        i++;
    }
}

void MotorList::GetStatesToPackage(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor, 
                         Eigen::VectorXd dir, DataPackage &data, int filtertype) {
    if (pos.size() != num_motors || vel.size() != num_motors || 
        tor.size() != num_motors || dir.size() != num_motors) {
        throw std::invalid_argument("State vector size does not match motor count");
    }

    GetStates(pos, vel, tor, dir);

    switch (filtertype) {
        case 0: 
            break;
            
        case 1: { 
            Eigen::VectorXd q(1);
            for (int i = 0; i < num_motors; i++) {
                q(0) = vel(i);
                motor_lowpass(i) = lowpass_[i]->mFilter(q)(0);
            }
            vel = motor_lowpass;
            break;
        }
            
        default:
            std::cerr << "Warning: Unknown filter type " << filtertype << std::endl;
            break;
    }

    data.motor_pos = pos;
    data.motor_vel = vel;
    data.motor_torque = tor;
}

Eigen::VectorXd MotorList::SmoothToZero(Eigen::VectorXd initial_pos) {
    if (initial_pos.size() != num_motors) {
        throw std::invalid_argument("Initial position vector size does not match motor count");
    }

    Eigen::VectorXd a0 = initial_pos;
    Eigen::VectorXd a1 = Eigen::VectorXd::Zero(num_motors);
    Eigen::VectorXd a2 = (-3.0 / (duration * duration)) * initial_pos;
    Eigen::VectorXd a3 = (2.0 / (duration * duration * duration)) * initial_pos;

    Eigen::VectorXd positions = a0 + a1 * dt + a2 * dt * dt + a3 * dt * dt * dt;

    if (dt < duration) {
        dt += 0.002;
    }

    return positions;
}

void MotorList::Disable() {
    auto Motors_Map = MotorControl->GetMotorsMap();
    if (!Motors_Map || Motors_Map->size() != num_motors) {
        throw std::runtime_error("Invalid motors map");
    }

    for (const auto& motorPair : *Motors_Map) {
        motorPair.second->SetControlWord(ControlWord_e::CTRL_SERVO_OFF);
    }
}

Eigen::MatrixXd readCSV(const std::string& filename, bool skipHeader) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    std::vector<std::vector<double>> data;

    if (skipHeader && !std::getline(file, line)) {
        throw std::runtime_error("Empty file after skipping header: " + filename);
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (...) {
                row.push_back(0.0); // 转换失败时使用默认值
            }
        }

        if (!row.empty()) {
            data.push_back(row);
        }
    }

    if (data.empty()) {
        throw std::runtime_error("No valid data found in file: " + filename);
    }

    size_t cols = data[0].size();
    for (const auto& row : data) {
        if (row.size() != cols) {
            throw std::runtime_error("Inconsistent column count in CSV file: " + filename);
        }
    }

    Eigen::MatrixXd mat(data.size(), cols);
    for (size_t i = 0; i < data.size(); ++i) {
        mat.row(i) = Eigen::VectorXd::Map(&data[i][0], cols);
    }

    return mat;
}

void interpolateJointPosVel(
    const Eigen::MatrixXd& arm_joint_pos,
    double t_arm, 
    double T_arm_total,
    Eigen::VectorXd& arm_JointPos_desire,
    Eigen::VectorXd& arm_JointVel_desire) {
    
    if (arm_joint_pos.rows() == 0 || arm_joint_pos.cols() == 0) {
        throw std::invalid_argument("Empty joint position matrix");
    }

    int time_steps = arm_joint_pos.rows();
    int joint_num = arm_joint_pos.cols();

    if (arm_JointPos_desire.size() != joint_num || arm_JointVel_desire.size() != joint_num) {
        throw std::invalid_argument("Output vector size mismatch");
    }

    double t_norm = t_arm / T_arm_total;
    t_norm = (t_norm < 0.0) ? 0.0 : (t_norm > 1.0) ? 1.0 : t_norm;

    double index_f = t_norm * (time_steps - 1);
    int idx = static_cast<int>(std::floor(index_f));
    double frac = index_f - idx;

    if (idx >= time_steps - 1) {
        idx = time_steps - 2;
        frac = 1.0;
    } else if (idx < 0) {
        idx = 0;
        frac = 0.0;
    }

    double dt_steps = T_arm_total / (time_steps - 1);

    for (int j = 0; j < joint_num; ++j) {
        double p0 = arm_joint_pos(idx, j);
        double p1 = arm_joint_pos(idx + 1, j);

        arm_JointPos_desire(j) = (1 - frac) * p0 + frac * p1;

        double vel = (p1 - p0) / dt_steps;
        
        if (t_norm <= 0.0 || t_norm >= 1.0) {
            vel = 0.0;
        }

        arm_JointVel_desire(j) = vel;
    }
}

void ThirdpolyVector(
    const Eigen::VectorXd& p0, const Eigen::VectorXd& p0_dot,
    const Eigen::VectorXd& p1, const Eigen::VectorXd& p1_dot,
    double totalTime, double currentTime,
    Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot) {
    
    if (totalTime <= 0) {
        throw std::invalid_argument("Total time must be positive");
    }

    int dim = p0.size();
    if (p0_dot.size() != dim || p1.size() != dim || 
        p1_dot.size() != dim || pd.size() != dim || 
        pd_dot.size() != dim) {
        throw std::invalid_argument("Vector size mismatch in ThirdpolyVector");
    }

    if (currentTime < 0) {
        pd = p0;
        pd_dot.setZero();
    } else if (currentTime <= totalTime) {
        for (int i = 0; i < dim; ++i) {
            double a0 = p0(i);
            double a1 = p0_dot(i);
            double m = p1(i) - p0(i) - p0_dot(i) * totalTime;
            double n = p1_dot(i) - p0_dot(i);
            double a2 = 3 * m / (totalTime * totalTime) - n / totalTime;
            double a3 = -2 * m / (totalTime * totalTime * totalTime) + n / (totalTime * totalTime);

            pd(i) = a3 * std::pow(currentTime, 3) + a2 * std::pow(currentTime, 2) + a1 * currentTime + a0;
            pd_dot(i) = 3 * a3 * currentTime * currentTime + 2 * a2 * currentTime + a1;
        }
    } else {
        pd = p1;
        pd_dot.setZero();
    }
}


