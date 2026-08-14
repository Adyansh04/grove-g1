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
        return FrameStatus::kIncomplete;
    }

    SensorFrameHeader header;
    std::memcpy(&header, buffer.data(), sizeof(header));

    if (header.magic != grove_g1::kSensorFrameMagic)
    {
        return FrameStatus::kBadMagic;
    }
    if (header.version != grove_g1::kSensorFrameVersion)
    {
        return FrameStatus::kBadVersion;
    }
    const bool is_cloud   = header.kind == static_cast<std::uint32_t>(SensorFrameKind::PointCloud);
    const bool is_depth   = header.kind == static_cast<std::uint32_t>(SensorFrameKind::Depth);
    const bool is_objects = header.kind == static_cast<std::uint32_t>(SensorFrameKind::ObjectPoses);
    const bool is_imu     = header.kind == static_cast<std::uint32_t>(SensorFrameKind::Imu);
    if (!is_cloud && !is_depth && !is_objects && !is_imu)
    {
        return FrameStatus::kBadKind;
    }
    // Every length is checked before it is trusted for anything: a desynchronised stream
    // otherwise reserves whatever garbage it read.
    if (is_cloud)
    {
        if (header.point_count > kMaxPoints ||
            header.payload_bytes !=
                static_cast<std::size_t>(header.point_count) * 3U * sizeof(float))
        {
            return FrameStatus::kBadLength;
        }
    }
    else if (is_imu)
    {
        if (header.payload_bytes != sizeof(grove_g1::ImuSampleRecord))
        {
            return FrameStatus::kBadLength;
        }
    }
    else if (is_objects)
    {
        // The record count is derived rather than carried, so the only thing to check is
        // that the payload is a whole number of records and not an absurd number of them.
        if (header.payload_bytes % sizeof(grove_g1::ObjectPoseRecord) != 0 ||
            header.payload_bytes / sizeof(grove_g1::ObjectPoseRecord) > kMaxObjects)
        {
            return FrameStatus::kBadLength;
        }
    }
    else
    {
        const std::uint64_t pixels =
            static_cast<std::uint64_t>(header.width) * static_cast<std::uint64_t>(header.height);
        const std::uint64_t expect =
            pixels * sizeof(float) + static_cast<std::uint64_t>(header.rgb_bytes);
        if (pixels == 0 || pixels > kMaxPoints || header.payload_bytes != expect ||
            (header.rgb_bytes != 0 && header.rgb_bytes != pixels * 3U))
        {
            return FrameStatus::kBadLength;
        }
    }

    const std::size_t total = sizeof(header) + header.payload_bytes;
    if (buffer.size() < total)
    {
        return FrameStatus::kIncomplete;
    }

    out.sim_time_s = header.sim_time_s;
    std::memcpy(out.sensor_pos.data(), header.sensor_pos, sizeof(out.sensor_pos));
    std::memcpy(out.sensor_quat.data(), header.sensor_quat, sizeof(out.sensor_quat));
    out.width    = header.width;
    out.height   = header.height;
    out.fovy_deg = header.fovy_deg;
    if (is_cloud)
    {
        out.kind = FrameKind::kPointCloud;
        out.depth.clear();
        out.objects.clear();
        out.points.resize(static_cast<std::size_t>(header.point_count) * 3U);
        if (header.payload_bytes > 0)
        {
            std::memcpy(out.points.data(), buffer.data() + sizeof(header), header.payload_bytes);
        }
    }
    else if (is_imu)
    {
        out.kind = FrameKind::kImu;
        out.depth.clear();
        out.rgb.clear();
        out.points.clear();
        out.objects.clear();
        std::memcpy(&out.imu, buffer.data() + sizeof(header), sizeof(out.imu));
    }
    else if (is_objects)
    {
        out.kind = FrameKind::kObjectPoses;
        out.depth.clear();
        out.points.clear();
        out.objects.resize(header.payload_bytes / sizeof(grove_g1::ObjectPoseRecord));
        if (header.payload_bytes > 0)
        {
            std::memcpy(out.objects.data(), buffer.data() + sizeof(header), header.payload_bytes);
        }
        // The name is used to build a topic-visible id, so a producer that somehow sent an
        // unterminated one must not run off the end of the record.
        for (grove_g1::ObjectPoseRecord& record : out.objects)
        {
            record.name[sizeof(record.name) - 1] = '\0';
        }
    }
    else
    {
        out.kind = FrameKind::kDepth;
        out.points.clear();
        out.objects.clear();
        const std::size_t px          = static_cast<std::size_t>(header.width) * header.height;
        const std::size_t depth_bytes = px * sizeof(float);
        out.depth.resize(px);
        std::memcpy(out.depth.data(), buffer.data() + sizeof(header), depth_bytes);
        out.rgb.resize(header.rgb_bytes);
        if (header.rgb_bytes != 0)
        {
            std::memcpy(
                out.rgb.data(),
                buffer.data() + sizeof(header) + depth_bytes,
                header.rgb_bytes);
        }
    }

    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(total));
    return FrameStatus::kOk;
}

const char* toString(FrameStatus status)
{
    switch (status)
    {
        case FrameStatus::kOk:
            return "ok";
        case FrameStatus::kIncomplete:
            return "incomplete";
        case FrameStatus::kBadMagic:
            return "bad magic (stream desynchronised or not ours)";
        case FrameStatus::kBadVersion:
            return "version mismatch between simulator and relay";
        case FrameStatus::kBadKind:
            return "unknown frame kind";
        case FrameStatus::kBadLength:
            return "payload length inconsistent with point count";
    }
    return "unknown";
}

}  // namespace g1_sensor_relay
