#include <gmock/gmock.h>

#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include "g1_sensor_relay/frame_reader.hpp"

using namespace g1_sensor_relay;
using grove_g1::SensorFrameHeader;

namespace
{

std::vector<std::uint8_t> makeFrame(std::uint32_t point_count, double sim_time = 1.5)
{
    SensorFrameHeader header{};
    header.magic          = grove_g1::kSensorFrameMagic;
    header.version        = grove_g1::kSensorFrameVersion;
    header.kind           = static_cast<std::uint32_t>(grove_g1::SensorFrameKind::PointCloud);
    header.point_count    = point_count;
    header.payload_bytes  = point_count * 3u * sizeof(float);
    header.sim_time_s     = sim_time;
    header.sensor_pos[2]  = 1.26;
    header.sensor_quat[0] = 1.0;

    std::vector<std::uint8_t> bytes(sizeof(header) + header.payload_bytes);
    std::memcpy(bytes.data(), &header, sizeof(header));
    for (std::uint32_t i = 0; i < point_count * 3u; ++i)
    {
        const float v = static_cast<float>(i);
        std::memcpy(bytes.data() + sizeof(header) + i * sizeof(float), &v, sizeof(float));
    }
    return bytes;
}

}  // namespace

TEST(FrameReader, ReadsAWholeFrameAndConsumesExactlyIt)
{
    auto       bytes = makeFrame(4);
    const auto size  = bytes.size();
    CloudFrame frame;

    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::Ok);
    EXPECT_TRUE(bytes.empty()) << "consumed " << (size - bytes.size()) << " of " << size;
    EXPECT_EQ(frame.points.size(), 12u);
    EXPECT_DOUBLE_EQ(frame.sim_time_s, 1.5);
    EXPECT_FLOAT_EQ(frame.points[11], 11.0f);
}

TEST(FrameReader, WaitsWhenTheFrameIsOnlyPartlyArrived)
{
    const auto full = makeFrame(8);
    CloudFrame frame;

    // A stream socket splits writes anywhere; every prefix must be safe to retry.
    for (std::size_t n = 0; n < full.size(); ++n)
    {
        std::vector<std::uint8_t> partial(full.begin(), full.begin() + n);
        const auto                before = partial.size();
        EXPECT_EQ(tryReadFrame(partial, frame), FrameStatus::Incomplete) << "at " << n << " bytes";
        EXPECT_EQ(partial.size(), before) << "consumed bytes from an incomplete frame";
    }
}

TEST(FrameReader, ReadsBackToBackFramesFromOneBuffer)
{
    auto       bytes  = makeFrame(2, 1.0);
    const auto second = makeFrame(3, 2.0);
    bytes.insert(bytes.end(), second.begin(), second.end());

    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::Ok);
    EXPECT_DOUBLE_EQ(frame.sim_time_s, 1.0);
    EXPECT_EQ(frame.points.size(), 6u);

    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::Ok);
    EXPECT_DOUBLE_EQ(frame.sim_time_s, 2.0);
    EXPECT_EQ(frame.points.size(), 9u);
    EXPECT_TRUE(bytes.empty());
}

TEST(FrameReader, RejectsGarbageRatherThanGuessing)
{
    CloudFrame frame;

    auto bad_magic = makeFrame(2);
    bad_magic[0] ^= 0xFF;
    EXPECT_EQ(tryReadFrame(bad_magic, frame), FrameStatus::BadMagic);

    auto                bad_version = makeFrame(2);
    const std::uint32_t v           = grove_g1::kSensorFrameVersion + 1;
    std::memcpy(bad_version.data() + 4, &v, sizeof(v));
    EXPECT_EQ(tryReadFrame(bad_version, frame), FrameStatus::BadVersion);

    auto                bad_kind = makeFrame(2);
    const std::uint32_t k        = 99;
    std::memcpy(bad_kind.data() + 8, &k, sizeof(k));
    EXPECT_EQ(tryReadFrame(bad_kind, frame), FrameStatus::BadKind);
}

TEST(FrameReader, RefusesALengthThatDoesNotMatchThePointCount)
{
    // The failure mode that matters: a desynchronised stream yields a plausible header whose
    // length is nonsense. Trusting it means allocating whatever it says.
    auto                bytes = makeFrame(4);
    const std::uint32_t lie   = 4u * 3u * sizeof(float) + 4u;
    std::memcpy(bytes.data() + 12, &lie, sizeof(lie));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::BadLength);
}

TEST(FrameReader, RefusesAnAbsurdPointCountBeforeAllocating)
{
    auto                bytes = makeFrame(1);
    const std::uint32_t huge  = kMaxPoints + 1;
    std::memcpy(bytes.data() + 80, &huge, sizeof(huge));
    // Keep payload_bytes consistent so only the cap can reject it.
    const std::uint32_t payload = huge * 3u * sizeof(float);
    std::memcpy(bytes.data() + 12, &payload, sizeof(payload));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::BadLength);
}

TEST(FrameReader, HandlesAnEmptyCloud)
{
    auto       bytes = makeFrame(0);
    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::Ok);
    EXPECT_TRUE(frame.points.empty());
}

TEST(WireFormat, TheTwoCopiesOfSensorFrameAreIdentical)
{
    // The simulator compiles its own copy under workspace/vendor. If they drift, the relay
    // reinterprets bytes and the failure looks like corrupt geometry, not a build problem.
    const char* ours   = "include/g1_sensor_relay/sensor_frame.h";
    const char* theirs = "../../vendor/unitree_mujoco/sensor_frame.h";

    std::ifstream a(ours), b(theirs);
    ASSERT_TRUE(a.is_open()) << "cannot open " << ours;
    ASSERT_TRUE(b.is_open()) << "cannot open " << theirs;

    std::stringstream sa, sb;
    sa << a.rdbuf();
    sb << b.rdbuf();
    EXPECT_EQ(sa.str(), sb.str())
        << "workspace/vendor/unitree_mujoco/sensor_frame.h and the relay's copy have drifted";
}
