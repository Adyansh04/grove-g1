#include "g1_sensor_relay/frame_reader.hpp"

#include <cstring>

namespace g1_sensor_relay
{

FrameStatus tryReadFrame(std::vector<std::uint8_t>& buffer, CloudFrame& out)
{
    using grove_g1::SensorFrameHeader;
    using grove_g1::SensorFrameKind;

    if (buffer.size() < sizeof(SensorFrameHeader))
    {
        return FrameStatus::Incomplete;
    }

    SensorFrameHeader header;
    std::memcpy(&header, buffer.data(), sizeof(header));

    if (header.magic != grove_g1::kSensorFrameMagic)
    {
        return FrameStatus::BadMagic;
    }
    if (header.version != grove_g1::kSensorFrameVersion)
    {
        return FrameStatus::BadVersion;
    }
    if (header.kind != static_cast<std::uint32_t>(SensorFrameKind::PointCloud))
    {
        return FrameStatus::BadKind;
    }
    if (header.point_count > kMaxPoints)
    {
        return FrameStatus::BadLength;
    }
    // Checked before the length is trusted for anything: a desynchronised stream otherwise
    // reserves whatever garbage it read.
    if (header.payload_bytes != header.point_count * 3u * sizeof(float))
    {
        return FrameStatus::BadLength;
    }

    const std::size_t total = sizeof(header) + header.payload_bytes;
    if (buffer.size() < total)
    {
        return FrameStatus::Incomplete;
    }

    out.sim_time_s = header.sim_time_s;
    std::memcpy(out.sensor_pos, header.sensor_pos, sizeof(out.sensor_pos));
    std::memcpy(out.sensor_quat, header.sensor_quat, sizeof(out.sensor_quat));
    out.points.resize(static_cast<std::size_t>(header.point_count) * 3u);
    if (header.payload_bytes > 0)
    {
        std::memcpy(out.points.data(), buffer.data() + sizeof(header), header.payload_bytes);
    }

    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(total));
    return FrameStatus::Ok;
}

const char* toString(FrameStatus status)
{
    switch (status)
    {
        case FrameStatus::Ok:
            return "ok";
        case FrameStatus::Incomplete:
            return "incomplete";
        case FrameStatus::BadMagic:
            return "bad magic (stream desynchronised or not ours)";
        case FrameStatus::BadVersion:
            return "version mismatch between simulator and relay";
        case FrameStatus::BadKind:
            return "unknown frame kind";
        case FrameStatus::BadLength:
            return "payload length inconsistent with point count";
    }
    return "unknown";
}

}  // namespace g1_sensor_relay
