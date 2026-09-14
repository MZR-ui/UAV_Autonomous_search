#include "remote_info/serial_driver.h"

#include <fcntl.h>       // open, O_RDWR, etc.
#include <unistd.h>      // read, write, close
#include <termios.h>     // termios, tcgetattr, etc.
#include <sys/ioctl.h>   // ioctl
#include <cerrno>        // errno
#include <cstring>       // strerror
#include <iostream>      // std::cerr, std::endl
#include <chrono>        // std::chrono::milliseconds
#include <thread>        // std::this_thread::sleep_for

namespace remote_info
{

SerialDriver::SerialDriver() : fd_(-1), running_(false)
{
}

SerialDriver::~SerialDriver()
{
    close();
}

bool SerialDriver::open(const std::string& port, int baud_rate)
{
    if (isOpen())
    {
        std::cerr << "Serial port already open" << std::endl;
        return false;
    }

    port_ = port;
    baud_rate_ = baud_rate;

    // 打开串口设备，O_NOCTTY 表示不成为控制终端，O_NDELAY 表示非阻塞
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0)
    {
        std::cerr << "Failed to open serial port " << port << ": " << strerror(errno) << std::endl;
        return false;
    }

    // 获取当前终端属性
    memset(&tty_, 0, sizeof(tty_));
    if (tcgetattr(fd_, &tty_) != 0)
    {
        std::cerr << "tcgetattr() failed: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 设置为原始模式（raw mode），避免特殊字符处理
    cfmakeraw(&tty_);

    // 启用接收，忽略调制解调器控制线
    tty_.c_cflag |= (CLOCAL | CREAD);

    // 设置数据位为 8 位
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= CS8;

    // 禁用奇偶校验
    tty_.c_cflag &= ~PARENB;

    // 1 位停止位
    tty_.c_cflag &= ~CSTOPB;

    // 禁用硬件流控
    tty_.c_cflag &= ~CRTSCTS;

    // 设置输入模式：原始输入，不处理特殊字符
    tty_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 设置输出模式：原始输出
    tty_.c_oflag &= ~OPOST;

    // 设置控制字符：VMIN = 0, VTIME = 1（非阻塞，1 分秒超时）
    tty_.c_cc[VMIN] = 0;
    tty_.c_cc[VTIME] = 1;

    // 设置波特率
    speed_t speed;
    switch (baud_rate)
    {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            std::cerr << "Unsupported baud rate: " << baud_rate << ", using 115200" << std::endl;
            speed = B115200;
            break;
    }
    cfsetispeed(&tty_, speed);
    cfsetospeed(&tty_, speed);

    // 应用设置
    if (tcsetattr(fd_, TCSANOW, &tty_) != 0)
    {
        std::cerr << "tcsetattr() failed: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 清除串口缓冲区
    tcflush(fd_, TCIOFLUSH);

    std::cout << "Serial port " << port << " opened successfully (baud=" << baud_rate << ")" << std::endl;
    return true;
}

void SerialDriver::close()
{
    if (running_)
    {
        running_ = false;
        if (read_thread_.joinable())
            read_thread_.join();
    }

    if (fd_ != -1)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialDriver::isOpen() const
{
    return fd_ != -1;
}

void SerialDriver::startAsyncRead(DataCallback callback)
{
    if (!isOpen())
    {
        std::cerr << "Cannot start async read: serial port not open" << std::endl;
        return;
    }

    if (running_)
    {
        std::cerr << "Async read already running" << std::endl;
        return;
    }

    data_callback_ = callback;
    running_ = true;
    read_thread_ = std::thread(&SerialDriver::readLoop, this);
}

bool SerialDriver::write(const uint8_t* data, size_t len)
{
    if (!isOpen())
    {
        std::cerr << "Cannot write: serial port not open" << std::endl;
        return false;
    }

    size_t total_written = 0;
    while (total_written < len)
    {
        ssize_t n = ::write(fd_, data + total_written, len - total_written);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 暂时无法写入，稍后重试（非阻塞模式下可能发生）
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            std::cerr << "Write error: " << strerror(errno) << std::endl;
            return false;
        }
        total_written += n;
    }
    return true;
}

void SerialDriver::readLoop()
{
    uint8_t buffer[256];
    while (running_)
    {
        ssize_t n = ::read(fd_, buffer, sizeof(buffer));
        if (n > 0)
        {
            if (data_callback_)
            {
                data_callback_(buffer, static_cast<size_t>(n));
            }
        }
        else if (n < 0)
        {
            // 非阻塞模式下，如果没有数据可读，read 可能返回 -1 并设置 EAGAIN
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                std::cerr << "Read error: " << strerror(errno) << std::endl;
                break;
            }
        }
        // 短暂休眠避免 CPU 占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

} // namespace msp_interface