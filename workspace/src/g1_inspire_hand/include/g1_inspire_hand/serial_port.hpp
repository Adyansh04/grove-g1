#ifndef G1_INSPIRE_HAND__SERIAL_PORT_HPP_
#define G1_INSPIRE_HAND__SERIAL_PORT_HPP_

/**
 * @file serial_port.hpp
 * @brief A byte stream to the hand, and the RS485 serial line that implements it.
 */

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace g1_inspire_hand
{

/**
 * @brief Raw bytes to and from one hand, so the register client runs over a fake in tests.
 */
class ByteStream
{
public:
    ByteStream()                             = default;
    ByteStream(const ByteStream&)            = delete;
    ByteStream& operator=(const ByteStream&) = delete;
    ByteStream(ByteStream&&)                 = delete;
    ByteStream& operator=(ByteStream&&)      = delete;
    virtual ~ByteStream()                    = default;

    /**
     * @brief Sends every byte, blocking until they are handed to the line.
     *
     * @return false if the line refused them.
     */
    [[nodiscard]] virtual bool write(std::span<const std::uint8_t> bytes) = 0;

    /**
     * @brief Reads until @p buffer is full or @p deadline passes.
     *
     * @return How many bytes arrived, at most the buffer's size.
     */
    [[nodiscard]] virtual std::size_t
    read(std::span<std::uint8_t> buffer, std::chrono::steady_clock::time_point deadline) = 0;

    /// Drops whatever has arrived unasked, such as the tail of a late reply.
    virtual void discardInput() = 0;
};

/**
 * @brief An RS485 line through a USB serial adapter, or a pseudo-terminal: raw 8N1, no flow
 *        control.
 *
 * @warning One master per line: a second one, such as Unitree's own hand service left running,
 *          garbles both, and nothing on the line can tell.
 */
class SerialPort final : public ByteStream
{
public:
    /**
     * @brief Takes ownership of an open, configured line, closing it on destruction.
     */
    explicit SerialPort(int fd)
      : fd_(fd)
    {}

    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&)                 = delete;
    SerialPort& operator=(SerialPort&&)      = delete;
    ~SerialPort() override;

    /**
     * @brief Opens and configures a line.
     *
     * @param path The device, such as /dev/ttyUSB1.
     * @param baud_rate 9600, 19200, 57600, 115200 or 921600.
     * @param[out] error Why it failed, when it returns null.
     * @return Null if the device could not be opened or configured.
     */
    [[nodiscard]] static std::unique_ptr<SerialPort>
    open(const std::string& path, int baud_rate, std::string& error);

    [[nodiscard]] bool write(std::span<const std::uint8_t> bytes) override;
    [[nodiscard]] std::size_t
    read(std::span<std::uint8_t> buffer, std::chrono::steady_clock::time_point deadline) override;
    void discardInput() override;

private:
    int fd_{ -1 };
};

}  // namespace g1_inspire_hand

#endif  // G1_INSPIRE_HAND__SERIAL_PORT_HPP_
