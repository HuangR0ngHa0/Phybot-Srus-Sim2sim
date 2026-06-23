#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <atomic>
#include <vector>
using namespace std;



typedef enum : uint8_t{

    ReadData = 0x03,  //读数据
    WriteData = 0x06  //写数据

}Red_Wrt;

typedef enum {

    CTRL_NONE = 0x0000,

    CTRL_SERVO_OFF = 0xF001,    // 失能
    CTRL_SERVO_ON = 0xF002,     // 使能
    CTRL_CLEAR_FAULT = 0xF086,  // 清除错误

    // 传感器相关
    CTRL_CALIBRATE_ENCODER = 0x101F,  // 编码器校准
    CTRL_RETURN_ZERO = 0x102F,        // 执行器自动回零
    CTRL_POSITION_SET_ZERO = 0x104F,  // 执行器位置置零

    // 控制相关
    CTRL_FRICTION_IDENTIFY = 0x201F,  // 摩擦校准
    CTRL_COGGING_IDENTIFY = 0x202F,   // 齿槽转矩校准

    // 功能开关 0x3xxxF
    CTRL_FRICTION_COMP_OFF = 0x300F,       // 失能摩擦补偿
    CTRL_FRICTION_COMP_ON = 0x301F,        // 使能摩擦补偿
    CTRL_COGGING_COMP_OFF = 0x302F,        // 失能齿槽转矩补偿
    CTRL_COGGING_COMP_ON = 0x303F,         // 使能齿槽转矩补偿
    CTRL_TLOAD_COMP_OFF = 0x304F,          // 失能负载补偿
    CTRL_TLOAD_COMP_ON = 0x305F,           // 使能负载补偿
    CTRL_SOFT_POS_LIMIT_OFF = 0x306F,      // 失能软限位
    CTRL_SOFT_POS_LIMIT_ON = 0x307F,       // 使能软限位
    CTRL_OVER_TEMP_PROTECT_OFF = 0x308F,   // 失能过温保护
    CTRL_OVER_TEMP_PROTECT_ON = 0x309F,    // 使能过温保护
    CTRL_HEART_BEAT_PROTECT_OFF = 0x30AF,  // 失能心跳保护
    CTRL_HEART_BEAT_PROTECT_ON = 0x30BF,   // 使能心跳保护

} ControlWord_e;

// 电机控制模式
typedef enum {

    MOTOR_CTRL_MODE_NONE = 0,         // 无控制模式
    MOTOR_CTRL_MODE_CURRENT = 1,      // 电流控制模式
    MOTOR_CTRL_MODE_VELOCITY = 2,     // 速度控制模式
    MOTOR_CTRL_MODE_POSITION = 3,     // 位置控制模式
    MOTOR_CTRL_MODE_PD = 4,           // 转矩位置控制模式
    MOTOR_CTRL_MODE_BRAKE = 5,        // 制动模式
    MOTOR_CTRL_MODE_OPENLOOP = 0xff,  // 开环控制模式

} MotorCtrlMode_e;

// 电机状态码
typedef enum {

    IDLE_WM = 0,                  // 空闲状态
    Init_WM = 1,                  // 初始化状态
    Normal_WM = 2,                // 正常控制状态
    Fault_WM = 3,                 // 异常状态
    ENCODER_CAIL_WM = 4,          // 特殊状态 电机编码器校准
    LINER_HALL_CAIL_WM = 5,       // 特殊状态 电机编码器校准
    OUTPUT_ENCODER_CAIL_WM = 6,   // 特殊状态 出轴编码器校准
    ACTUATOR_RETURN_ZERO_WM = 7,  // 特殊状态 执行器回零模式
    FRICTION_IDENTIFY_WM = 8,     // 特殊状态 摩擦辨识模式

} UserWorkMode_e;

//通信控制指令
typedef enum : uint16_t{
  
    ConfigNetWork = 0x600,        //网络通信参数设置
    SetId,                        //设置电机ID
    ControlMode,                  //控制模式
    ControlWord,                  //控制字
    WorkMode,                     //工作模式
    FastMode,                     //快速上报模式
    Reboot,                       //软件重启                     
    ControlSetPos,                //指令位置
    ControlSetVel,                //指令速度
    ControlSetTor,                //指令力矩
    ControlSetCur,                //指令电流
    ControlSetPID,                //指令PID
    ControlSetPD,                 //指令PD
    GetMotorPVCT,                 //反馈PVCT
    ErrCode,                      //反馈错误码
    Inverter,                     //反馈逆变器状态

    SetDirction,                  //指令方向
    SetOutputShaftRatio,          //指令减速比
    SetPosThreshold,              //指令位置限制
    SetVCTMax,                    //指令最大VCT
    SetOutputOffset,              //指令出轴传感器位置偏置
    SetOutEnable,                 //指令输出轴编码器使能
    SetTorToCur,                  //指令转矩到电流多项式系数

    AcBaseParams,                 //执行器基础参数集合

    SendCalibrationData,	    // 发送校准数据到SDK
    SetFocThetaSorce,			// 设置角度源
    SetM1Voltage,				// 设置强拖电压

    OTA_CMD_HandShanking,      // OTA 握手
    OTA_CMD_UpDateStart,       // OTA 开始下载
    OTA_CMD_FileTransmission,  // OTA 文件传输
    OTA_CMD_UpDateEnd,         // OTA 下载结束
    OTA_CMD_SendVersion,       // OTA 发送版本号
    Model_SendVersion,		   // Model 型号
    SetMotor_Mode,			   // 设置电机类型  
}Command_Types;


/*角度获取模式*/
typedef enum {
    e_ForceTheta = 0,    // 强制角度标志
    e_SensorLess = 1,    // 无感运行标志
    e_Sensor = 2,        // 有感运行标志
    e_SensorFusion = 3,  // 传感器融合角度
    e_ThetaSorceAll,
} ThetaSorce_e;



//电机类型
typedef enum { 
    PhyTypeNULL = 0, 
    PhyArc47_S72V1_KV99 = 0x001, 
    PhyArc68_S72V1_KV99 = 0x002, 
    PhyArc78_S72V1_KV99 = 0x003, 
    PhyArc95_S72V1_KV99 = 0x004, 
    PhyArc102_S72V1_KV99 = 0x005, 
    PhyArc150_S72V1_KV99 = 0x006,
    PhyArc102_S72V2_KV99 = 0x015,
    PhyArc47_G72V1_KV99  = 0x101,
    PhyArc68_G72V1_KV99  = 0x102,
    PhyArc78_G72V1_KV99  = 0x103,
    PhyArc95_G72V1_KV99  = 0x104,
    PhyArc102_G72V1_KV99 = 0x105,
    PhyArc150_G72V1_KV99 = 0x106,
} ActuatorType_e;


//网络参数
typedef struct{

     uint32_t LocalIP;               //本地IP
    uint16_t LocalPort;              //本地端口
    uint32_t TargetIP;               //目标IP

} NetWorkParams;

typedef struct{

    float PVCT_InterfaceLpfWc_V1;           //位置反馈滤波器截止频率
    float PVCT_InterfaceLpfWc_V2;           //速度反馈滤波器截止频率
    float PVCT_InterfaceLpfWc_V3;           //电流/力矩反馈滤波器截止频率

} PVCT_InterfaceLpfWc;

//执行器基础参数
typedef struct{

    int32_t Dirction;                       //方向
    float OutputShaftRatio;                 //减速比
    float MaxPos_Rad;                       //最大位置
    float MinPos_Rad;                       //最小位置
    float MaxVel_RadS;                      //最大速度
    float MaxCur_A;                         //最大电流
    float MaxTorque_NM;                     //最大力矩
    float MaxAcc_RadSS;                     //最大加速度
    PVCT_InterfaceLpfWc InterfaceLpfWc;


} ActuatorBaseParams;



//执行器传感器参数
typedef struct{

    float CoilNTCParamR_Max;
    float CoilNTCParamR_R_High;
    float CoilNTCParamR_Mid;
    float CoilNTCParamR_Min;
    float CoilNTCParamR_GND;
    float CoilNTCParamPoly_Between_MaxHigh[5];
    float CoilNTCParamPoly_Between_HighMid[5];
    float CoilNTCParamPoly_Between_MidMin[5];
    float MosNTCParamR_Max;
    float MosNTCParamR_R_High;
    float MosNTCParamR_Mid;
    float MosNTCParamR_Min;
    float MosNTCParamR_GND;
    float MosNTCParamPoly_Between_MaxHigh[5];
    float MosNTCParamPoly_Between_HighMid[5];
    float MosNTCParamPoly_Between_MidMin[5];

    uint32_t FocSensorType;                  //FOC传感器类型
    uint32_t FocSensorIsReversal;            //FOC传感器是否反向
    uint32_t OutputShaftSensorType;          //出轴传感器类型
    uint32_t OutputShaftSensorIsReversal;    //出轴传感器是否反向
    float OutputShaftSensorOffset;           //出轴传感器零位偏置

} ActuatorSensorParams;

//执行器控制参数
typedef struct{

    uint32_t PosLoopDiv;                    //位置环分频
    float PosKp;                            //电流位置控制比例增益
    float PosKd;                            //电流位置控制微分增益
    uint32_t VelLoopDiv;                    //速度环分频
    float VelKp;                            //电流速度控制比例增益
    float VelKi;                            //电流速度控制积分增益
    float VelFdbkLpfWc;                     //速度反馈低通滤波截止频率
    uint32_t CurLoopDiv;                    //电流环分频
    float CurLoopWc;                        //电流环带宽
    float CurLoopLowFreqGain;               //电流环低频增益
    uint32_t PDLoopDiv;                     //PD环分频
    float PDKp;                             //PD控制比例增益
    float PDKd;                             //PD控制微分增益
    float BreakPosKp;                       //刹车位置控制比例增益
    float BreakVelKp;                       //刹车速度控制比例增益
    float BreakVelKi;                       //刹车速度控制积分增益

} ActuatorControlParams;

//执行器功能参数
typedef struct{

    uint32_t CtrlMode;                      //控制模式
    uint32_t FocThetaSorce;                 //FOC角度源选择
    uint32_t FrictionCompEnable;            //摩擦补偿使能
    uint32_t TorqueLoadCompEnable;          //负载转矩补偿
    uint32_t CoggingCompEnable;             //齿槽转矩补偿使能
    uint32_t OutShaftSensorEnable;          //输出轴编码器使能

} ActuatorFuncParams;

//电机参数
typedef struct{

    uint32_t NPP;                          //极对数
    float Rs;                              //相电阻
    float Ld;                              //D轴电感
    float Lq;                              //Q轴电感
    float FluxWb;                          //磁链系数
    float RotorInertia;                    //转子惯量

} MotorParams;

//硬件参数
typedef struct{

    uint32_t ADCFullVal;                   //ADC最大值
    float TimerFreq_MHz;                   //定时器频率
    float IsCenterAlign;                   //是否为中央对齐模式
    float PWMFreq_KHz;                     //PWM频率
    float VBusRange;                       //母线电压范围
    float PhaseCurRange;                   //相电流范围
    float PhaseCurOffset;                  //相电流偏置
    float PhaseVolRange;                   //相电压范围
    float SampingRsNum;                    //采样电阻数量
    float DeadTime_nS;                     //死区时间
    float CurAmpGain;                      //电流放大增益
    float GateDriverType;                  //预驱类型

} HardwareParams;

//强拖控制参数
typedef struct{

   float Acc_RadSS;                         //强拖加速度    
    float ElecVelocity_Hz;                  //强拖速度
    float Theta_Rad;                        //强拖角度
    float Current;                          //强拖电流
    float Voltage;                          //强拖电压


} ForceParams;

//无感控制参数
typedef struct{

    float PLLWc_Hz;                         //锁相环带宽
    float PllLpfWc_Hz;                      //锁相环速度二次滤波带宽
    float CompVel_Hz;                       //Id补偿速度区间
    float CompId_A;                         //补偿电流
    float CompGain;                         //补偿增益
    float DeadVel_Hz;                       //无传感器模式死区速度
    float FusionMinVel_Hz;                  //有感融合其实转速


} FulxObsParams;

//电流力矩标定参数
typedef struct{

    float TorqueToCurrentA;                  //转矩->电流多项式系数A
    float TorqueToCurrentB;                  //转矩->电流多项式系数B
    float TorqueToCurrentC;                  //转矩->电流多项式系数C  
    float CurrentToTorqueX;                  //转矩->电流多项式系数X
    float CurrentToTorqueY;                  //转矩->电流多项式系数Y
    float CurrentToTorqueZ;                  //转矩->电流多项式系数Z
    float TorqueMax;                         //

} TorqueCailbParams;

//摩擦补偿参数
typedef struct{

    float Fc;                               //库仑摩擦力                          
    float Fs;                               //最大静摩擦力
    float B;                                //粘滞摩擦系数
    float DeadVel;                          //死区速度
    uint32_t CaliFlag;                      //摩擦补偿校准成功标志
    float Percent;                          //摩擦补偿百分比


} FrictionParams;

//错误检测参数
typedef struct{

    float Iq_OverCur;                        //Iq保护电流
    float Iq_OverCur_S;                      //Iq保护电流时间 秒
    float ChipOverTemp;                      //芯片过温保护阈值
    float MosOverTemp;                       //Mos过温保护阈值
    float CoilOverTemp;                      //绕组过温保护阈值
    float OverTemp_S;                        //过温保护时间 秒
    float MosTempWarn;                       //Mos温度警告阈值
    float CoilTempWarn;                      //绕组温度警告阈值
    float PosMax_Rad;                        //位置限制最大值
    float PosMin_Rad;                        //位置限制最小值
    float MosOverTempRecLimit_mS;            //Mos过温恢复时间限制(ms)
    float CoilOverTempRecLimit_mS;           //绕组过温恢复时间限制(ms)
    float LossPhaseCheckMinVel_RadS;         //缺相检测最小速度(rads)
    float OverVbus;                          //过压阈值
    float UnderVbus;                         //低压阈值
    float ComTimeOutMs;                      //通信超时保护      

} ErrDectParams;



//FastMode参数
typedef struct{

    std::atomic<int> FastPos_;
    std::atomic<int> FastVel_;
    std::atomic<int> FastTor_e_;
    std::atomic<int> FastErrorCode;
    std::atomic<int> FastIsOnline;

}FastParams;



//电机所有参数
typedef struct{

    uint32_t ControlMode_;
    uint32_t WorkMode_;

    uint32_t Pos_;
    uint32_t Vel_;
    uint32_t Cur_;
    uint32_t Tor_l_;
    uint32_t Tor_e_;
    uint32_t Fault_;
    uint32_t FaultExt2_;
    uint32_t FaultExt3_;
    uint32_t FaultExt4_;
    uint32_t MosTemp_;
    uint32_t CoilTemp_;
    uint32_t VBusTemp_;
    uint32_t State; //状态版本号
    uint32_t Major; //主版本号
    uint32_t Minor; //次版本号
    uint32_t Patch; //修订版本号
    int32_t EncoderValue; //编码器值
    std::atomic<uint16_t> canLineId; //can线id
    std::atomic<int> ACK_;
    std::atomic<int> CurrCmd;
    std::atomic<int> Serial_Number; //传输文件序号
    char model[20];  //电机型号
    ActuatorBaseParams BaseParams_;
    ActuatorSensorParams SensorParams_;
    ActuatorControlParams ControlParams_;
    ActuatorFuncParams FuncParams_;
    MotorParams MotorParams_;
    HardwareParams HardwareParams_;
    ForceParams ForceParams_;
    FulxObsParams FulxObsParams_;
    TorqueCailbParams TorqueCailbParams_;
    // FrictionParams FrictionParams_;
    // ErrDectParams ErrDectParams_;
    

} TotalParams;

//创建电机对象参数
typedef struct{
    uint16_t canId;     //can  id
    uint16_t canLind;   //can线 id
}MotorCan;
//存储升级文件组
using ByteGroup = std::vector<std::uint8_t>;

//版本号
//主版本.次版本.修订版本
//│      │      │        
//│      │      │        
//│      │      │        
//│      │     向下兼容的 Bug 修复
//│      └ 向下兼容的功能新增
//└不兼容的 API 变更
#define VERSION "1.9.0"

#endif // TYPES_H