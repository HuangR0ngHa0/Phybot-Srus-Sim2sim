#ifndef MOTORDRIVE_H
#define MOTORDRIVE_H

#include "./Udp/DataProcess.h"
#include "Types.h"
#include <string>
#include <yaml-cpp/yaml.h>

using namespace std;

typedef struct
{
   
} Device_Param;



class MotorDriver {

public:

    /**
     * @brief 初始化类
     * @param id  电机canId
     * @param MainEditionProcess 通信指针
     * @param canlineid  can线id
     */
    MotorDriver(uint16_t id,std::shared_ptr<DataProcess> MainEditionProcess,uint16_t canlineid=0);
    ~MotorDriver();

    /**
     * @brief 配置主控板网络通信参数及建立连接
     * @param LocalIp 本地IP地址
     * @param LocalPort 本地端口号,默认15020
     * @param DestIp 设置目标IP地址，默认192.168.3.200
     * @return true 配置连接成功 false 配置连接失败
     * @warning 广播报文，你必须用ID 0x00FF的Motor对象使用
     */

    bool ConfigureNetworkParams(string LocalIp, uint16_t LocalPort = 15020, string DestIp = "192.168.3.200");

    /**
     * @brief 关闭连接
     */

    void CloseConnect();

    /**
     * @brief 设置电机ID
     * @param NewId 新的电机ID值
     * @return true 设置成功 false 设置失败
     */

    bool SetMotorId(uint16_t NewId);

    /**
     * @brief 设置控制模式
     * @param Index 控制模式索引值 详见枚举 MotorCtrlMode_e
     * @return true 设置成功 false 设置失败
     */

    bool SetControlMode(uint32_t Index);

    /**
     * @brief 设置控制字
     * @param Word 控制字值 详见枚举 ControlWord_e
     * @return true 设置成功 false 设置失败
     */

    bool SetControlWord(uint32_t Word);

    /**
     * @brief 设置快速模式参数
     * @param Mode 快速模式是否开启 1 开启 0关闭
     * @param RepRate 快速模式上报频率
     * @return  true 设置成功 false 设置失败
     */

    bool SetFastMode(uint32_t Mode, uint32_t RepRate);

    /**
     * @brief 设置电机重启状态
     * @param state 重启状态值 1 重启 2不重启
     * @return void
     */
    void SetMotorReboot(uint32_t state);

    /**
     * @brief 设置位置、速度和电流控制参数
     *
     * @param Pos 位置设定值 单位:rad
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void SetPos(float Pos, float Vel=0, float Cur=0);


     /**
     * @brief 设置大包位置、速度和电流控制参数
     *
     * @param Pos 位置设定值 单位:rad
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void SetBigparam(float Pos, float Vel, float Cur);


     /**
     * @brief 获取大包位置、速度和电流控制参数
     *
     * @param Pos 位置设定值 单位:rad
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void GetBigparam(float& Pos, float& Vel, float& Cur);

    /**
     * @brief 设置速度和电流控制参数
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void SetVel(float Vel, float Cur);

    /**
     * @brief 设置转矩控制参数
     * @param Tor 转矩设定值 单位: N.m
     * @return void
     */

    void SetTor(float Tor);

    /**
     * @brief 设置电流控制参数
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void SetCur(float Cur);

    /**
     * @brief 设置位置、速度PID控制参数
     * @param PosKp 位置环比例系数
     * @param VelKp 速度环比例系数
     * @param VelKi 速度环积分系数
     * @return void 
     */

    void SetPID(float PosKp, float VelKp, float VelKi);

    /**
     * @brief 设置位置PD控制参数
     * @param PDKp 位置环比例系数
     * @param PDKd 位置环微分系数
     * @return void
     */

    void SetPD(float PDKp, float PDKd);

    /**
     * @brief 从文件加载电机控制参数
     * @param FileName 参数文件路径
     * @return void
     */

    void SetParam(string FileName);


    /**
     * @brief 设置方向
     * @param Dirct 方向  1正向 2反向
     * @return 设置成功返回true，失败返回false
     */

    bool SetDirction(int32_t Dirct);

    /**
     * @brief设置减速比
     * @param vlaue 速比值
     * @return 设置成功返回true，失败返回false
     */

    bool SetOutputShaftRatio(float vlaue);

    /**
     * @brief设置位置阈值范围
     *
     * @param MaxPos 最大位置阈值
     * @param MinPos 最小位置阈值
     * @return 设置成功返回true，失败返回false
     */

    bool SetPosThreshold(float MaxPos, float MinPos);

    /**
     * 设置速度、电流和扭矩的最大值限制
     *
     * @param MaxVel 最大速度限制
     * @param MaxCur 最大电流限制
     * @param MaxTor 最大扭矩限制
     * @return 设置成功返回true，失败返回false
     */

    bool SetVCTMax(float MaxVel, float MaxCur, float MaxTor);

    /**
     * 出轴传感器零位偏置
     *
     * @param offset 输出偏移值
     * @return 设置成功返回true，失败返回false
     */

    bool SetOutputOffset(float offset);

    /**
     * 输出轴编码器使能
     *
     * @param Enable 使能控制值
     * @return 设置成功返回true，失败返回false
     */
    bool SetOutEnable(uint32_t Enable);

    /**
     * 设置扭矩到电流多项式系数
     *
     * @param TTCA 转换参数A
     * @param TTCB 转换参数B
     * @param TTCC 转换参数C
     * @param TTCX 转换参数X
     * @param TTCY 转换参数Y
     * @param TTCZ 转换参数Z
     * @param TorMax 最大扭矩值
     * @return 设置成功返回true，失败返回false
     */
    
    bool SetTorToCur(float TTCA,float TTCB,float TTCC,float TTCX,float TTCY,float TTCZ,float TorMax);


    /**
     * @brief 获取当前控制模式 详见枚举 MotorCtrlMode_e
     *
     * @param Index 存储控制模式索引的变量引用
     * @return true 获取成功 false 获取失败
     */

    bool GetControlMode(uint32_t &Index);

    /**
     * @brief 快速获取位置、速度、电流和估计转矩 此函数用于高速采样场景
     *
     * @param Pos 存储位置值的引用  单位:rad
     * @param Vel 存储速度值的引用  单位:rad/s
     * @param Cur 存储电流值的引用
     * @param Tor_e 存储估计转矩值的引用  单位: N.m
     * @param FastStateMechine 状态机状态
     * @param FastBusVoltage 母线电压
     * @param FastErrorCode 错误码
     */

    void GetPVCTFast(float &Pos, float &Vel, float &Cur, float &Tor_e,
      uint32_t& FastStateMechine,uint32_t& FastBusVoltage, uint32_t& FastErrorCode);

    /**
     * @brief 获取位置、速度、电流和转矩信息
     *
     * @param Pos 存储位置值的引用  单位:rad
     * @param Vel 存储速度值的引用  单位:rad/s
     * @param Cur 存储电流值的引用  单位:A
     * @param Tor_l 存储负载转矩值的引用  单位: N.m
     * @param Tor_e 存储电磁转矩值的引用  单位: N.m
     * @return true 获取成功 false 获取失败
     */

    bool GetPVCT(float &Pos, float &Vel, float &Cur, float &Tor_l, float &Tor_e);



    /**
   * @brief 获取编码器值
   *
   * @param EncoderValue 编码器值
   * @return true 获取成功 false 获取失败
   */

    bool GetEncoderValue(int & EncoderValue);



    /*
    * @brief 设置强拖
    * @param Anglesource 开环枚举，见Types.h 枚举ThetaSorce_e
    * @param 转速  
    * @return true 获取成功 false 获取失败
    */

    bool SetStrongDragging(uint32_t& Anglesource,float Voltage);

    /*
    * @brief 获取电机强拖模式
    * @param Anglesource 角度源枚举，见Types.h 枚举ThetaSorce_e
    * @return true 获取成功 false 获取失败
    */
    bool GetStrongDragging(uint32_t& Anglesource);

    /**
   * @brief 设置角度源
   * @param Anglesource 角度源
   * @return true 获取成功 false 获取失败
   */

    bool SetAngleSource(uint32_t Anglesource);


    /**
     * @brief 设置电压
     * @param Voltage 电压
     * @return true 获取成功 false 获取失败
     */

    bool SetVoltage(float Voltage);

    /**
     * @brief 保存电机控制参数到文件
     *
     * @param FileName 参数文件路径
     */

    void GetParam(string FileName);

    /**
     * @brief 获取电机故障代码
     *
     * @param Fault 主故障代码
     * @param FaultExt2 扩展故障代码2
     * @param FaultExt3 扩展故障代码3
     * @param FaultExt4 扩展故障代码4
     * @return true 获取成功 false 获取失败
     */

    bool GetErrCode(uint32_t &Fault, uint32_t &FaultExt2, uint32_t &FaultExt3, uint32_t &FaultExt4);

    /**
     * @brief 获取逆变器状态信息
     *
     * @param Mos MOSFET温度
     * @param Coil 线圈温度
     * @param Bus 母线电压
     * @return true 获取成功 false 获取失败
     */

    bool GetInverterStatus(float &Mos, float &Coil, float &Bus);

    /**
     * @brief 获取在线电机ID
     *
     * @return 在线电机ID
     * @warning 你必须用ID 0x00FF的Motor对象使用
     */

    uint16_t GetMotorOnline();



    /**
   * @brief 升级前握手（擦falsh）
   * @return 主板准备是否成功 true成功，false 失败
   * @warning
   */
    bool StartOTAUpgrade();


    /**
    * @brief 升级固件
    * @param filename   升级文件名称
    * @return 返回升级是否成功 1为成功,
    */
    int OTAUpgradeing(const string filename);



    /**
    * @brief 查看固件版本
    * @param version   版本号
    * @return 返回升级是否成功
    */
    bool GetOTAUpgradeVersion( string& version);

    /**
    * @brief 查看电机型号
    * @param version   型号
    * @return 返回升级是否成功
    */
    bool GetModelVersion(string& version);

    /**
    * @brief 设置电机型号
    * @param version   型号
    * @return 返回升级是否成功
    */
    bool SetModelVersion(uint32_t & type);



   /**
   * @brief 一键固件升级
   * @param filename   升级文件名称
   * @param void (*func)(int,int)  回调函数，传递进度和错误码，1为成功，非1失败 -1: 握手失败，-2:发送开始标志失败,-3:传输文件失败,-4:发送停止标志失败
   * @return 返回升级是否成功 1为成功,
   */
    int OneClickOTAUpgradeing(const string filename,void (*func)(int,int));


    uint16_t Id;
    uint16_t canLineId;

  private:

      /**
     * @brief 读取文件
     * @param filename   升级文件名称
     * @return 以32字节为一组的字节列表
     */
    std::vector<ByteGroup> load_file(const std::string& path);
      /**
      * @brief 发送开始升级指令
      * @param num   升级文件按32字节读取总组数量
      * @return 返回升级是否成功 true 成功，false失败
      */
    bool OTAUpgradeSign(const uint32_t  num);


      /**
      * @brief 发送数据
      * @param num   序号
      * @param data  有效数据
      * @return 返回升级是否成功 true成功，false失败
      */
    bool sendOTAUpgradeData(const uint32_t  num, const string data);


      /**
      * @brief 发送停止升级指令
      * @return 返回升级是否成功 true成功，false失败
      */
    bool StopOTAUpgradeSign();


    std::shared_ptr<DataProcess> Process;
    
    TotalParams TotalParamsData;
    FastParams FastParamsData;

    float m_pos;
    float m_vel;
    float m_cur;



};

#endif // MOTORDRIVE_H    
