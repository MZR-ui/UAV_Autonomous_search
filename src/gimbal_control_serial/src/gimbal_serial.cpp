#include "gimbal_control_serial/gimbal_serial.hpp"
#include <iostream>
#include <iomanip>

using namespace gimbal_protocol;

GimbalSerial::GimbalSerial(const std::string& port, int baudrate)
    : serial_(io_), port_(port), baudrate_(baudrate)
{
}

GimbalSerial::~GimbalSerial()
{
    close();
}

void GimbalSerial::setReceiveCallback(ReceiveCallback cb)
{
    receive_cb_ = cb;
}

bool GimbalSerial::open()
{
    try
    {
        serial_.open(port_);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        ROS_INFO("Serial port %s opened at %d baud", port_.c_str(), baudrate_);
        
        running_ = true;
        if (!read_thread_.joinable()) {
            read_thread_ = std::thread(&GimbalSerial::readLoop, this);
        }
        return true;
    }
    catch (std::exception& e)
    {
        ROS_ERROR_THROTTLE(1.0, "Failed to open serial port %s : %s", port_.c_str(), e.what());
        return false;
    }
}

// Read and parse packet from receive buffer
bool GimbalSerial::readPacket(Gbc2GcuPkt_t& pkt)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Packet size: header + data length
    size_t pkt_size = sizeof(Gbc2GcuPkt_t);
    if (recv_buffer_.size() < 2) return false;

    // Search for packet header: 0xB5 0x9A
    size_t i = 0;
    while (i + 1 < recv_buffer_.size()) {
        if (recv_buffer_[i] == 0xB5 && recv_buffer_[i+1] == 0x9A) {
            // Check if there is enough data from current position to buffer end
            if (recv_buffer_.size() - i < pkt_size) {
                return false;
            }

            // Copy complete packet to pkt
            memcpy(&pkt, &recv_buffer_[i], pkt_size);

            // Perform CRC check: CRC is located in the last 2 bytes of packet tail
            uint16_t crc_calc = gimbal_protocol::CalculateCrc16(reinterpret_cast<uint8_t*>(&pkt), pkt_size - 2);
            uint16_t crc_recv = static_cast<uint16_t>(pkt.crc[1]) | (static_cast<uint16_t>(pkt.crc[0]) << 8);

            if (crc_calc == crc_recv) {
                // CRC check passed, remove processed data from buffer
                recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + i + pkt_size);
                return true;
            } else {
                // CRC check failed, skip current 0xB5 and continue searching for next header
                ROS_WARN_THROTTLE(1.0, "GimbalSerial: CRC mismatch in received packet, skipping this header");
                i += 1; // move forward one byte (skip current 0xB5)
                continue;
            }
        } else {
            ++i;
        }
    }

    // If buffer is too large, there may be invalid data accumulation
    // To prevent unlimited memory growth, trim buffer when it exceeds threshold
    const size_t max_buffer_allowed = 4096;
    if (recv_buffer_.size() > max_buffer_allowed) {
        // Keep the tail portion of buffer, discard old data at the front
        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.end() - 1024);
        ROS_WARN_THROTTLE(1.0, "GimbalSerial: recv_buffer_ too large, trimming.");
    }

    return false;
}

void GimbalSerial::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    if (serial_.is_open())
    {
        serial_.close();
    }
    if (read_thread_.joinable()) {
        // Unblock thread if waiting on serial
        read_thread_.detach();
    }
}



bool GimbalSerial::sendPacket(const Gcu2GbcPkt_t& pkt)
{
    if (!isOpen()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    size_t len = sizeof(pkt);
    boost::asio::write(serial_, boost::asio::buffer(data, len));
    return true;
}


void GimbalSerial::readLoop()
{
    while (ros::ok()) {
        if (!running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (serial_.is_open()) {
            try {
                int bytes_avail = 0;
                ioctl(serial_.native_handle(), FIONREAD, &bytes_avail);
                if (bytes_avail > 0) {
                    std::vector<uint8_t> temp_buf(bytes_avail);
                    size_t bytes_read = boost::asio::read(serial_, boost::asio::buffer(temp_buf), boost::asio::transfer_at_least(1));
                    std::lock_guard<std::mutex> lock(mutex_);
                    recv_buffer_.insert(recv_buffer_.end(), temp_buf.begin(), temp_buf.begin() + bytes_read);
                }
            } catch (std::exception& e) {
            }
        }
        
        Gbc2GcuPkt_t pkt;
        if (readPacket(pkt)) {
            if (receive_cb_) {
                receive_cb_(pkt);
            }
        } else {
            // brief sleep to prevent 100% CPU on failure
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

bool GimbalSerial::sendAndWaitReply(const Gcu2GbcPkt_t& tx_pkt,
                          Gbc2GcuPkt_t& rx_pkt,
                          int timeout_ms)
{
    if (!sendPacket(tx_pkt)) {
        ROS_WARN("GimbalSerial: failed to send packet");
        return false;
    }

    auto start = std::chrono::steady_clock::now();
    std::vector<uint8_t> recv_buf;
    recv_buf.reserve(256);

    while (true) {
        // ---------- Timeout check ----------
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
            ROS_WARN("GimbalSerial: response timeout");
            return false;
        }

        // ---------- Read data ----------
        int bytes_available = 0;
        int fd = serial_.lowest_layer().native_handle();
        if (ioctl(fd, FIONREAD, &bytes_available) < 0) {
            ROS_ERROR("GimbalSerial: ioctl(FIONREAD) failed");
            return false;
        }

        if (bytes_available > 0) {
            std::vector<uint8_t> tmp(bytes_available);
            boost::system::error_code ec;

            size_t r = serial_.read_some(boost::asio::buffer(tmp), ec);
            if (ec) {
                ROS_ERROR("GimbalSerial: read_some error: %s", ec.message().c_str());
                return false;
            }
            recv_buf.insert(recv_buf.end(), tmp.begin(), tmp.begin() + r);
        }

        // ---------- Parse data and verify ----------
        for (size_t i = 0; i + 1 < recv_buf.size(); ++i) {
            if (recv_buf[i] == 0xB5 && recv_buf[i + 1] == 0x9A) {
                size_t remain = recv_buf.size() - i;
                if (remain >= sizeof(Gbc2GcuPkt_t)) {
                    memcpy(&rx_pkt, &recv_buf[i], sizeof(Gbc2GcuPkt_t));

                    uint16_t crc_calc = CalculateCrc16(reinterpret_cast<uint8_t*>(&rx_pkt),
                                                       sizeof(Gbc2GcuPkt_t) - 2);
                    uint16_t crc_recv = rx_pkt.crc[1] | (rx_pkt.crc[0] << 8);

                    // --- Debug: print received data ---
                    // std::ostringstream oss;
                    // oss << "Recv data: ";
                    // for (size_t j = 0; j < sizeof(Gbc2GcuPkt_t); ++j) {
                    //     oss << std::hex << std::setw(2) << std::setfill('0')
                    //         << static_cast<int>(reinterpret_cast<uint8_t*>(&rx_pkt)[j]) << " ";
                    // }
                    // ROS_INFO("%s", oss.str().c_str());

                    // // --- Debug: print CRC ---
                    // ROS_INFO("CRC calc=0x%04X, CRC recv=0x%04X", crc_calc, crc_recv);
                    
                    if (crc_calc == crc_recv) {
                        return true;  // CRC check passed
                    } else {
                        ROS_WARN_THROTTLE(1.0, "CRC mismatch (calc=0x%04X, recv=0x%04X)", crc_calc, crc_recv);
                        return false;
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}





