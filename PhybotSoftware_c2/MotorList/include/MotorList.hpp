#ifndef MOTORLIST_H_
#define MOTORLIST_H_
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <yaml-cpp/yaml.h> 
#include <vector>
#include <string>
// #include "syst.hpp"

#include <iostream>
#include <string>
#include "MotorDrive.h"
#include "MainEdition.h"
#include "MotorsControl.h"

#include "DataPackage/include/DataPackage.h"
#include "LowPassFilter/include/LowPassFilter.h"


class MotorList{
    public:
    MotorList();
    ~MotorList();
    int num_motors ;
    std::vector<std::string> motor_names;
    std::vector<std::shared_ptr<MotorDriver>> motor_ptrs;

    std::shared_ptr<std::map<std::string, std::shared_ptr<MotorDriver>>> Motors_Map;

    std::shared_ptr<std::map<std::string, std::shared_ptr<MainEdition>>> MainBoards_Map;

    void Init(std::string path1, std::string path2, const DataPackage &DataPackage);
    void Init(std::string path1, std::string path2,  std::string path3, const DataPackage &DataPackage);
    bool Enable();
    bool SetZero();
    // void CloseUdp();
    void SetCommands(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd, Eigen::VectorXd dir);
    void SetCommandsFromPackage(Eigen::VectorXd& pos_cmd, Eigen::VectorXd& vel_cmd, Eigen::VectorXd& tor_cmd, Eigen::VectorXd dir, DataPackage &data);
    void GetStates(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor, Eigen::VectorXd dir);
    void GetStatesToPackage(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor, Eigen::VectorXd dir, DataPackage &data, int filtertype);
    Eigen::VectorXd SmoothToZero(Eigen::VectorXd initial_pos);
    void Disable();


    // 主控板 和 电机
    std::vector<MotorDriver*> motor;

    Eigen::VectorXd qpos_cur;
    Eigen::VectorXd qvel_cur;
    Eigen::VectorXd qtor_cur;

    double duration =1.0;
    double dt =0.001;
    std::vector<LowPassFilter *> lowpass_;
    Eigen::VectorXd motor_lowpass;

    std::shared_ptr<MotorsControl> MotorControl = std::make_shared<MotorsControl>();
    

    Eigen::VectorXd P_control_vector;  
    Eigen::VectorXd D_control_vector;  
    Eigen::VectorXd direction_vector;  
    Eigen::VectorXd abs_zero;

};

Eigen::MatrixXd readCSV(const std::string& filename, bool skipHeader);
void interpolateJointPosVel(
    const Eigen::MatrixXd& arm_joint_pos, 
    double t_arm, double T_arm_total,    
    Eigen::VectorXd& arm_JointPos_desire,  
    Eigen::VectorXd& arm_JointVel_desire
);


void ThirdpolyVector(
    const Eigen::VectorXd& p0, const Eigen::VectorXd& p0_dot,
    const Eigen::VectorXd& p1, const Eigen::VectorXd& p1_dot,
    double totalTime, double currentTime,
    Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot);


#endif