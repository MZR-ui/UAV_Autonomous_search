#ifndef MSP_INTERFACE_SERIAL_DRIVER_H
#define MSP_INTERFACE_SERIAL_DRIVER_H

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <termios.h>   // 用于 termios 结构体

namespace msp_interface
{

/**
 * @brief Linux 串口驱动类，支持异步读取和同步写入
 * 
 * 封装了 POSIX 串口操作，内部创建独立线程进行非阻塞读取，
 * 通过回调函数将接收到的数据传递给上层。
 */
class SerialDriver
{
public:
    /// 数据接收回调函数类型，参数为数据指针和长度
    using DataCallback = std::function<void(const uint8_t* data, size_t len)>;

    /**
     * @brief 构造函数
     */
    SerialDriver();

    /**
     * @brief 析构函数，自动关闭串口和线程
     */
    ~SerialDriver();

    /**
     * @brief 打开串口设备
     * @param port      设备路径，例如 "/dev/ttyUSB0"
     * @param baud_rate 波特率，支持 9600, 19200, 38400, 57600, 115200, 230400
     * @return 成功返回 true，失败返回 false
     */
    bool open(const std::string& port, int baud_rate);

    /**
     * @brief 关闭串口并停止读取线程
     */
    void close();

    /**
     * @brief 检查串口是否已打开
     * @return 已打开返回 true
     */
    bool isOpen() const;

    /**
     * @brief 启动异步读取
     * @param callback 接收到数据时调用的回调函数
     * 
     * 该函数会创建一个新线程，循环读取串口数据并调用 callback。
     * 如果之前已启动，则不会重复启动。
     */
    void startAsyncRead(DataCallback callback);

    /**
     * @brief 同步写入数据
     * @param data 数据缓冲区指针
     * @param len  要写入的字节数
     * @return 成功写入所有字节返回 true，失败返回 false
     */
    bool write(const uint8_t* data, size_t len);

private:
    /**
     * @brief 读取线程的主循环
     */
    void readLoop();

private:
    int fd_;                          ///< 串口文件描述符
    std::atomic<bool> running_;        ///< 读取线程运行标志
    std::thread read_thread_;          ///< 读取线程对象
    DataCallback data_callback_;       ///< 数据接收回调

    std::string port_;                 ///< 串口设备路径
    int baud_rate_;                    ///< 波特率
    struct termios tty_;                ///< 终端属性结构体（保存配置）
};

} // namespace msp_interface

#endif // MSP_INTERFACE_SERIAL_DRIVER_H