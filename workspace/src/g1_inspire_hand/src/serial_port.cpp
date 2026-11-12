/**
 * @file serial_port.cpp
 * @brief The hand's RS485 line over termios.
 */

#include "g1_inspire_hand/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <format>
#include <optional>
#include <system_error>

namespace g1_inspire_hand
{
namespace
{

std::optional<speed_t> speedFor(int baud_rate)
{
    switch (baud_rate)
    {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 921600:
            return B921600;
        default:
            return std::nullopt;
    }
}

std::string errnoText() { return std::system_category().message(errno); }

int msUntil(std::chrono::steady_clock::time_point deadline)
{
    const auto left =
        std::chrono::ceil<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    return static_cast<int>(std::max<std::int64_t>(0, left.count()));
}

}  // namespace

SerialPort::~SerialPort()
{
    if (fd_ >= 0)
    {
        static_cast<void>(::close(fd_));
    }
}

std::unique_ptr<SerialPort>
SerialPort::open(const std::string& path, int baud_rate, std::string& error)
{
    const std::optional<speed_t> speed = speedFor(baud_rate);
    if (!speed)
    {
        error = std::format("{} baud is not a rate the hand takes", baud_rate);
        return nullptr;
    }
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
    {
        error = std::format("cannot open {}: {}", path, errnoText());
        return nullptr;
    }
    auto    port = std::make_unique<SerialPort>(fd);
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0)
    {
        error = std::format("{} is not a serial line: {}", path, errnoText());
        return nullptr;
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS);
    tio.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
    // Non-blocking reads paced by poll(), so a silent hand costs a deadline, never a hang.
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (::cfsetispeed(&tio, *speed) != 0 || ::cfsetospeed(&tio, *speed) != 0 ||
        ::tcsetattr(fd, TCSANOW, &tio) != 0)
    {
        error = std::format("cannot set {} to {} 8N1: {}", path, baud_rate, errnoText());
        return nullptr;
    }
    static_cast<void>(::tcflush(fd, TCIOFLUSH));
    return port;
}

bool SerialPort::write(std::span<const std::uint8_t> bytes)
{
    std::size_t sent = 0;
    while (sent < bytes.size())
    {
        const ssize_t n = ::write(fd_, bytes.data() + sent, bytes.size() - sent);
        if (n > 0)
        {
            sent += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR)
        {
            continue;
        }
        if (n < 0 && errno == EAGAIN)
        {
            pollfd        ready{ .fd = fd_, .events = POLLOUT, .revents = 0 };
            constexpr int kWriteWaitMs = 50;
            if (::poll(&ready, 1, kWriteWaitMs) <= 0)
            {
                return false;
            }
            continue;
        }
        return false;
    }
    return true;
}

std::size_t
SerialPort::read(std::span<std::uint8_t> buffer, std::chrono::steady_clock::time_point deadline)
{
    std::size_t got = 0;
    while (got < buffer.size())
    {
        pollfd    ready{ .fd = fd_, .events = POLLIN, .revents = 0 };
        const int polled = ::poll(&ready, 1, msUntil(deadline));
        if (polled < 0 && errno == EINTR)
        {
            continue;
        }
        if (polled <= 0 || (ready.revents & POLLIN) == 0)
        {
            break;
        }
        const ssize_t n = ::read(fd_, buffer.data() + got, buffer.size() - got);
        if (n > 0)
        {
            got += static_cast<std::size_t>(n);
        }
        else if (n == 0 || (errno != EINTR && errno != EAGAIN))
        {
            break;
        }
    }
    return got;
}

void SerialPort::discardInput() { static_cast<void>(::tcflush(fd_, TCIFLUSH)); }

}  // namespace g1_inspire_hand
