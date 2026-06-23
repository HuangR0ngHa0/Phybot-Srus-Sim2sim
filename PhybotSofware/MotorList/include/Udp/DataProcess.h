#ifndef DATAPROCESS_H
#define DATAPROCESS_H

#include "UdpSocket.h"
#include "Types.h"
#include "PackQueue.h"
#include "FastPackQueue.h"
#include <string>
#include <thread>
#include <memory>
#include <map>
#include <atomic>
#include <chrono>
#include <thread>

using namespace std;


typedef struct{
  uint16_t Can_Id;
  uint16_t Cmd;
  uint8_t Re_Wr;
  string Data;
  uint16_t canLine_id;
 
} Comb_Cmd;

typedef struct{
  uint16_t Can_Id;
  string Data;
} CombBig_Cmd;

class DataProcess {

public:
    DataProcess();
    ~DataProcess();



    bool InitUdp(uint16_t localPort, string remoteIp, uint16_t remotePort, string &Err);

 
       /**
     * @brief 关闭UDP并停止接收线程
     *
     * @warning 未调用此函数可能导致资源泄漏
     */

    void CloseUdp();

    void StartReceive();

    void StopReceive();

 

    /**
     * @brief 编码组合命令并发送UDP包
     *
     * @param Cmd 组合命令结构
     * @note 自动调用EncodeUdpPacket添加帧头和校验
     */

    void EncodeUdpStringAndSend(Comb_Cmd Cmd);



    /**
     * @brief 编码组合命令并发送UDP大pos包
     *
     * @param Cmd 组合命令结构
     * @note 自动调用EncodeUdpPacket添加帧头和校验
     */
    void EncodeUdpStringBigAndSend(CombBig_Cmd cmd);

    /**
     * @brief 编码UDP包（添加帧头和CRC校验）
     *
     * @param Packet 待编码的数据包（输入/输出）
     * @note 会在Packet前后添加帧头和CRC校验
     */

    void EncodeUdpPacket(string &Packet);



    /**
     * @brief 编码UDP pos 大包（添加帧头和CRC校验）
     *
     * @param Packet 待编码的数据包（输入/输出）
     * @note 会在Packet前后添加帧头和CRC校验
     */

    void EncodeUdpBigPacket(string &Packet);

    /**
     * @brief 等待特定命令响应
     *
     * @param cmd 命令ID
     * @param timeoutMs 超时时间（毫秒），默认200ms
     * @return true 收到响应 false 超时或错误
     */

    bool WaitForRespond(int cmd_,std::atomic<int>* Currcmd, int timeoutMs = 200);


    bool WaitForRespondNetwork(int cmd_,std::atomic<int>* Currcmd, Comb_Cmd cmd_data,int timeoutMs = 5000);

    /**
     * @brief 浮点型数据主机字节序转网络字节序
     *
     * @param net_float 待转换的浮点数
     * @return float 转换后的浮点数
     */

    float htonf(float net_float);

public:
    map<uint16_t,TotalParams*> TotalParamsData;
    map<uint16_t,FastParams*> FastDataMap;
    std::atomic<int> ConnectState;

protected:

    /**
     * @brief UDP数据接收线程函数
     *
     * @note 内部循环调用socket.recv
     */

    void UdpReceiver();

    /**
     * @brief 普通UDP帧解析并加入处理队列
     *
     * @note 调用CheckValid进行数据校验
     */

    void UnpackUdp();

    /**
     * @brief 解析Fast模式UDP数据
     *
     * @param RecvPacket 接收到的UDP数据包
     */

    void UnpackFastUdp(string RecvPacket);


     /**
     * @brief 解析Fast模式带有错误码UDP数据
     *
     * @param RecvPacket 接收到的UDP数据包
     */

    void UnpackErrorCodeFastUdp(string RecvPacket);

    /**
     * @brief UDP数据有效性校验
     *
     * @param RecvPacket 接收到的UDP数据包
     * @return true 数据有效 false 校验失败
     */

    bool CheckValid(string RecvPacket);

    
    /**
     * @brief 命令处理函数
     *
     * @param RecvPacket 接收到的UDP数据包
     * @note 根据命令类型更新不同参数表
     */

    void CommandHandler(string RecvPacket);

    /**
     * @brief 给线程使用解析Fast模式UDP数据
     *
     * @param RecvPacket 接收到的UDP数据包
     */
    void UnpackFastUdpThread();

private:
    /**
     * @brief UDP数据发送函数
     *
     * @param Packet 待发送的UDP数据包
     * @note 内部调用sendto系统函数
     */

    void UdpSender(string Packet);

    /**
     * @brief CRC16校验计算
     *
     * @param data 待校验数据
     * @param len 数据长度
     * @return uint16_t 计算得到的CRC16值
     * @note 使用直接计算法，多项式为0xA001
     */

    uint16_t crc16_direct(uint8_t *data, size_t len);


    /**
     * @brief 按数量分割字符串函数
     * @param input 待分割字符串
     * @param bytesPerChunk 分割大小
     * @return  std::vector<std::string>  将分割后的字符串存入容器中
    */


    std::vector<std::string> splitStringByBytes(const std::string& input, int bytesPerChunk = 16);

    UdpSocket* UdpSocket_;
    std::thread RecvThread_;
    std::thread UnPackThread_;
    std::thread FastUnPackThread_;

    PackQueue PackQueue_;
    PackQueue PackQueue1_;
    // FastPackQueue PackQueue1_;

    uint16_t LocalPort = 15020;
    uint16_t RemotePort = 15000;
    uint16_t SendFrameHeader = htons(0xAAF2);
    uint16_t SendBigFrameHeader = htons(0xAAF5);
    uint16_t RecvErrorCodeFrameHeader = 0xAAF6;
    uint16_t RecvFrameHeader = 0xAAF3;
    uint16_t FastRecvFrameHeader = 0xAAF4;
    string RemoteIp = "192.168.3.203";
};



#endif // DATAPROCESS_H    