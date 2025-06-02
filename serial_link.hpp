#ifndef __SERIALLINK_HPP__
#define __SERIALLINK_HPP__

#pragma once

#include "kato/log.hpp"

#include <functional>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>

// ====================================================================================================================
// Define the orders that can be sent and received
enum class _order : uint8_t
{
    HELLO = 0,
    ALREADY_CONNECTED = 1,
    SCAN = 2,
    READ = 3,
    START = 4,
    STOP = 5,
    ERROR = 255,
};
typedef enum _order order_t;
// ====================================================================================================================
class SerialLink
{
private:
    std::string m_port;
    speed_t m_baudrate;
    std::function<void()> m_error_callback;

public:
    int fd = -1;
    bool is_connected;
    // ----------------------------------------------------------------------------------------------------------------
    SerialLink(const char *_port, const speed_t _baudrate, std::function<void()> _error_callback) : m_port(_port), m_baudrate(_baudrate), m_error_callback(_error_callback), is_connected(false)
    {
        open_link();
    }
    void open_link()
    {
        fd = open(m_port.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1)
        {
            kato::log::cerr << KATO_RED << "Error opening serial port: " << strerror(errno) << KATO_RESET << std::endl;
            exit(1);
        }
        if (configure() != 0)
        {
            close(fd);
            exit(1);
        }
    }
    // ----------------------------------------------------------------------------------------------------------------
    ~SerialLink()
    {
        close(fd);
    }
    // ----------------------------------------------------------------------------------------------------------------
    int configure()
    {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            kato::log::cerr << KATO_RED << "Error getting attributes: " << strerror(errno) << KATO_RESET << std::endl;
            return -1;
        }

        cfsetospeed(&tty, m_baudrate); // Set output baud rate
        cfsetispeed(&tty, m_baudrate); // Set input baud rate

        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // One stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 8 data bits
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

        tty.c_lflag &= ~ICANON; // Set raw mode
        tty.c_lflag &= ~(ECHO | ECHOE | ISIG);

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
        tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);

        tty.c_oflag &= ~OPOST; // Disable output processing

        tcflush(fd, TCIFLUSH);

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            kato::log::cerr << KATO_RED << "Error setting attributes: " << strerror(errno) << KATO_RESET << std::endl;
            return -1;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    bool is_data_available()
    {
        int bytesAvailable;
        if (ioctl(fd, FIONREAD, &bytesAvailable) == -1)
        {
            kato::log::cerr << KATO_RED << "Error getting available bytes: " << strerror(errno) << KATO_RESET << std::endl;
            open_link();
            return false;
        }
        else
            return bytesAvailable > 0;
    }
    // ----------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------------------------------------------------------------
    void write_object(const void *_object, const size_t _size)
    {
        if (write(fd, _object, _size) == -1)
        {
            kato::log::cerr << KATO_RED << "Error writing to fd: " << strerror(errno) << KATO_RESET << std::endl;
            open_link();
        }
    }
    // ----------------------------------------------------------------------------------------------------------------
    void write_order(const order_t _order)
    {
        write_uint8((uint8_t)_order);
    }
    // ----------------------------------------------------------------------------------------------------------------
    void write_uint8(const uint8_t _value)
    {
        write_object(&_value, sizeof(uint8_t));
    }
    // ----------------------------------------------------------------------------------------------------------------
    void write_uint64(const uint64_t _value)
    {
        write_object(&_value, sizeof(uint64_t));
    }
    // ----------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------------------------------------------------------------
    void read_object(void *_object, const size_t _size)
    {
        if (read(fd, _object, _size) == -1)
        {
            kato::log::cerr << KATO_RED << "Error reading from fd: " << strerror(errno) << KATO_RESET << std::endl;
            open_link();
        }
    }
    // ----------------------------------------------------------------------------------------------------------------
    order_t read_order()
    {
        return (order_t)read_uint8();
    }
    // ----------------------------------------------------------------------------------------------------------------
    uint8_t read_uint8()
    {
        uint8_t value;
        read_object(&value, sizeof(uint8_t));
        return value;
    }
    // ----------------------------------------------------------------------------------------------------------------
    uint64_t read_uint64()
    {
        uint64_t value;
        read_object(&value, sizeof(uint64_t));
        return value;
    }
    // ----------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------------------------------------------------------------
    void read_and_respond()
    {
        if (is_data_available())
        {
            order_t order = read_order();
            if (order == order_t::HELLO)
            {
                // kato::log::cout << KATO_BLUE << "RX [HELLO]" << KATO_RESET << std::endl;
                if (!is_connected)
                {
                    is_connected = true;
                    write_order(order_t::HELLO);
                    // kato::log::cout << KATO_BLUE << "TX [HELLO]" << KATO_RESET << std::endl;
                }
                else
                {
                    write_order(order_t::ALREADY_CONNECTED);
                    // kato::log::cout << KATO_BLUE << "TX [ALREADY_CONNECTED]" << KATO_RESET << std::endl;
                }
            }
            else if (order == order_t::ALREADY_CONNECTED)
            {
                // kato::log::cout << KATO_BLUE << "RX [ALREADY_CONNECTED]" << KATO_RESET << std::endl;
                is_connected = true;
            }
        }
    }
};
// ====================================================================================================================

#endif //__SERIALLINK_HPP__
