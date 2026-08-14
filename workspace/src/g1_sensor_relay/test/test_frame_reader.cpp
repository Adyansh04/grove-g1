#include <gmock/gmock.h>

#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include "g1_sensor_relay/frame_reader.hpp"

using g1_sensor_relay::CloudFrame;
using g1_sensor_relay::FrameKind;
using g1_sensor_relay::FrameStatus;
using g1_sensor_relay::kMaxObjects;
using g1_sensor_relay::kMaxPoints;
using g1_sensor_relay::tryReadFrame;
using grove_g1::SensorFrameHeader;

namespace
{

std::vector<std::uint8_t> makeFrame(std::uint32_t point_count, double sim_time = 1.5)
{
    SensorFrameHeader header{};
    header.magic       = grove_g1::kSensorFrameMagic;
    header.version     = grove_g1::kSensorFrameVersion;
    header.kind        = static_cast<std::uint32_t>(grove_g1::SensorFrameKind::PointCloud);
    header.point_count = point_count;
    header.payload_bytes =
        static_cast<std::uint32_t>(static_cast<std::size_t>(point_count) * 3U * sizeof(float));
    header.sim_time_s     = sim_time;
    header.sensor_pos[2]  = 1.26;
    header.sensor_quat[0] = 1.0;

    std::vector<std::uint8_t> bytes(sizeof(header) + header.payload_bytes);
    std::memcpy(bytes.data(), &header, sizeof(header));
    for (std::uint32_t i = 0; i < point_count * 3U; ++i)
    {
        const auto v = static_cast<float>(i);
        std::memcpy(bytes.data() + sizeof(header) + i * sizeof(float), &v, sizeof(float));
    }
    return bytes;
}

// Depth frames carry `width * height` floats and, when colour is on, `width * height * 3`
// rgb bytes behind them in the same payload.
std::vector<std::uint8_t> makeDepthFrame(std::uint32_t w, std::uint32_t h, bool with_color)
{
    const std::uint32_t pixels = w * h;
    const std::uint32_t rgb    = with_color ? pixels * 3U : 0U;

    SensorFrameHeader header{};
    header.magic          = grove_g1::kSensorFrameMagic;
    header.version        = grove_g1::kSensorFrameVersion;
    header.kind           = static_cast<std::uint32_t>(grove_g1::SensorFrameKind::Depth);
    header.width          = w;
    header.height         = h;
    header.fovy_deg       = 58.0F;
    header.rgb_bytes      = rgb;
    header.payload_bytes  = pixels * sizeof(float) + rgb;
    header.sim_time_s     = 2.5;
    header.sensor_quat[0] = 1.0;

    std::vector<std::uint8_t> bytes(sizeof(header) + header.payload_bytes);
    std::memcpy(bytes.data(), &header, sizeof(header));
    for (std::uint32_t i = 0; i < pixels; ++i)
    {
        const float d = 1.0F + static_cast<float>(i);
        std::memcpy(bytes.data() + sizeof(header) + i * sizeof(float), &d, sizeof(float));
    }
    for (std::uint32_t i = 0; i < rgb; ++i)
    {
        bytes[sizeof(header) + pixels * sizeof(float) + i] = static_cast<std::uint8_t>(i & 0xFF);
    }
    return bytes;
}

// Object frames are a flat array of fixed-size records, with no count in the header: the
// record count follows from payload_bytes, so there is no second number to disagree.
std::vector<std::uint8_t> makeObjectFrame(const std::vector<std::string>& names)
{
    SensorFrameHeader header{};
    header.magic   = grove_g1::kSensorFrameMagic;
    header.version = grove_g1::kSensorFrameVersion;
    header.kind    = static_cast<std::uint32_t>(grove_g1::SensorFrameKind::ObjectPoses);
    header.payload_bytes =
        static_cast<std::uint32_t>(names.size() * sizeof(grove_g1::ObjectPoseRecord));
    header.sim_time_s     = 3.5;
    header.sensor_quat[0] = 1.0;

    std::vector<std::uint8_t> bytes(sizeof(header) + header.payload_bytes);
    std::memcpy(bytes.data(), &header, sizeof(header));
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        grove_g1::ObjectPoseRecord record{};
        std::strncpy(record.name, names[i].c_str(), sizeof(record.name) - 1);
        record.pos[0]  = 1.0 + static_cast<double>(i);
        record.pos[1]  = 2.0 + static_cast<double>(i);
        record.pos[2]  = 0.78;
        record.quat[0] = 1.0;
        record.size[0] = record.size[1] = record.size[2] = 0.06;
        std::memcpy(bytes.data() + sizeof(header) + i * sizeof(record), &record, sizeof(record));
    }
    return bytes;
}

// An IMU frame is one fixed-size record, with the attitude in the header fields every frame
// already carries.
std::vector<std::uint8_t> makeImuFrame()
{
    SensorFrameHeader header{};
    header.magic          = grove_g1::kSensorFrameMagic;
    header.version        = grove_g1::kSensorFrameVersion;
    header.kind           = static_cast<std::uint32_t>(grove_g1::SensorFrameKind::Imu);
    header.payload_bytes  = sizeof(grove_g1::ImuSampleRecord);
    header.sim_time_s     = 12.25;
    header.sensor_quat[0] = 1.0;
    header.sensor_pos[2]  = 1.22;

    grove_g1::ImuSampleRecord sample{};
    sample.gyro[1] = 0.5;
    sample.acc[2]  = 9.81;

    std::vector<std::uint8_t> bytes(sizeof(header) + sizeof(sample));
    std::memcpy(bytes.data(), &header, sizeof(header));
    std::memcpy(bytes.data() + sizeof(header), &sample, sizeof(sample));
    return bytes;
}

}  // namespace

TEST(FrameReader, ReadsAWholeFrameAndConsumesExactlyIt)
{
    auto       bytes = makeFrame(4);
    const auto size  = bytes.size();
    CloudFrame frame;

    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);
    EXPECT_TRUE(bytes.empty()) << "consumed " << (size - bytes.size()) << " of " << size;
    EXPECT_EQ(frame.points.size(), 12U);
    EXPECT_DOUBLE_EQ(frame.sim_time_s, 1.5);
    EXPECT_FLOAT_EQ(frame.points[11], 11.0F);
}

TEST(FrameReader, WaitsWhenTheFrameIsOnlyPartlyArrived)
{
    const auto full = makeFrame(8);
    CloudFrame frame;

    // A stream socket splits writes anywhere; every prefix must be safe to retry.
    for (std::size_t n = 0; n < full.size(); ++n)
    {
        std::vector<std::uint8_t> partial(
            full.begin(),
            full.begin() + static_cast<std::ptrdiff_t>(n));
        const auto before = partial.size();
        EXPECT_EQ(tryReadFrame(partial, frame), FrameStatus::kIncomplete) << "at " << n << " bytes";
        EXPECT_EQ(partial.size(), before) << "consumed bytes from an incomplete frame";
    }
}

TEST(FrameReader, ReadsBackToBackFramesFromOneBuffer)
{
    auto       bytes  = makeFrame(2, 1.0);
    const auto second = makeFrame(3, 2.0);
    bytes.insert(bytes.end(), second.begin(), second.end());

    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);
    EXPECT_DOUBLE_EQ(frame.sim_time_s, 1.0);
    EXPECT_EQ(frame.points.size(), 6U);

    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);
    EXPECT_DOUBLE_EQ(frame.sim_time_s, 2.0);
    EXPECT_EQ(frame.points.size(), 9U);
    EXPECT_TRUE(bytes.empty());
}

TEST(FrameReader, RejectsGarbageRatherThanGuessing)
{
    CloudFrame frame;

    auto bad_magic = makeFrame(2);
    bad_magic[0] ^= 0xFF;
    EXPECT_EQ(tryReadFrame(bad_magic, frame), FrameStatus::kBadMagic);

    auto                bad_version = makeFrame(2);
    const std::uint32_t v           = grove_g1::kSensorFrameVersion + 1;
    std::memcpy(bad_version.data() + 4, &v, sizeof(v));
    EXPECT_EQ(tryReadFrame(bad_version, frame), FrameStatus::kBadVersion);

    auto                bad_kind = makeFrame(2);
    const std::uint32_t k        = 99;
    std::memcpy(bad_kind.data() + 8, &k, sizeof(k));
    EXPECT_EQ(tryReadFrame(bad_kind, frame), FrameStatus::kBadKind);
}

TEST(FrameReader, RefusesALengthThatDoesNotMatchThePointCount)
{
    // The failure mode that matters: a desynchronised stream yields a plausible header whose
    // length is nonsense. Trusting it means allocating whatever it says.
    auto       bytes = makeFrame(4);
    const auto lie =
        static_cast<std::uint32_t>(static_cast<std::size_t>(4) * 3U * sizeof(float) + 4U);
    std::memcpy(bytes.data() + 12, &lie, sizeof(lie));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::kBadLength);
}

TEST(FrameReader, RefusesAnAbsurdPointCountBeforeAllocating)
{
    auto                bytes = makeFrame(1);
    const std::uint32_t huge  = kMaxPoints + 1;
    std::memcpy(bytes.data() + 80, &huge, sizeof(huge));
    // Keep payload_bytes consistent so only the cap can reject it.
    const auto payload =
        static_cast<std::uint32_t>(static_cast<std::size_t>(huge) * 3U * sizeof(float));
    std::memcpy(bytes.data() + 12, &payload, sizeof(payload));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::kBadLength);
}

TEST(FrameReader, HandlesAnEmptyCloud)
{
    auto       bytes = makeFrame(0);
    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);
    EXPECT_TRUE(frame.points.empty());
}

TEST(WireFormat, TheTwoCopiesOfSensorFrameAreIdentical)
{
    // The simulator compiles its own copy under workspace/vendor. If they drift, the relay
    // reinterprets bytes and the failure looks like corrupt geometry, not a build problem.
    const char* ours   = "include/g1_sensor_relay/sensor_frame.h";
    const char* theirs = "../../vendor/unitree_mujoco/sensor_frame.h";

    std::ifstream a(ours);
    std::ifstream b(theirs);
    ASSERT_TRUE(a.is_open()) << "cannot open " << ours;
    ASSERT_TRUE(b.is_open()) << "cannot open " << theirs;

    std::stringstream sa;
    std::stringstream sb;
    sa << a.rdbuf();
    sb << b.rdbuf();
    EXPECT_EQ(sa.str(), sb.str())
        << "workspace/vendor/unitree_mujoco/sensor_frame.h and the relay's copy have drifted";
}

TEST(FrameReader, ReadsADepthFrameWithColour)
{
    auto       bytes = makeDepthFrame(4, 3, /*with_color=*/true);
    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);
    EXPECT_TRUE(bytes.empty());
    EXPECT_EQ(frame.kind, FrameKind::kDepth);
    EXPECT_EQ(frame.width, 4U);
    EXPECT_EQ(frame.height, 3U);
    EXPECT_EQ(frame.depth.size(), 12U);
    EXPECT_EQ(frame.rgb.size(), 36U);
    EXPECT_FLOAT_EQ(frame.depth.front(), 1.0F);
    EXPECT_FLOAT_EQ(frame.depth.back(), 12.0F);
    // The colour bytes must come from behind the depth floats, not overlap them.
    EXPECT_EQ(frame.rgb.front(), 0U);
    EXPECT_EQ(frame.rgb.back(), 35U);
}

TEST(FrameReader, ReadsADepthFrameWithoutColour)
{
    auto       bytes = makeDepthFrame(4, 3, /*with_color=*/false);
    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);
    EXPECT_EQ(frame.depth.size(), 12U);
    EXPECT_TRUE(frame.rgb.empty());
}

TEST(FrameReader, RefusesAnRgbLengthThatDoesNotMatchThePixelCount)
{
    // The payload length still adds up, so only the rgb_bytes == pixels*3 rule catches
    // this. Without it the relay would publish an rgb8 image of the wrong size.
    auto              bytes = makeDepthFrame(4, 3, /*with_color=*/true);
    SensorFrameHeader header{};
    std::memcpy(&header, bytes.data(), sizeof(header));
    header.rgb_bytes -= 1;
    header.payload_bytes -= 1;
    bytes.resize(bytes.size() - 1);
    std::memcpy(bytes.data(), &header, sizeof(header));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::kBadLength);
}

TEST(FrameReader, RefusesAnAbsurdImageSizeBeforeAllocating)
{
    // 60000 x 60000 would be 14 GB of depth. The pixel cap has to reject it from the
    // header alone, before any resize.
    auto              bytes = makeDepthFrame(2, 2, /*with_color=*/false);
    SensorFrameHeader header{};
    std::memcpy(&header, bytes.data(), sizeof(header));
    header.width  = 60000;
    header.height = 60000;
    std::memcpy(bytes.data(), &header, sizeof(header));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::kBadLength);
}

TEST(FrameReader, ReadsAnObjectPoseFrame)
{
    auto       bytes = makeObjectFrame({ "red_cube", "green_cylinder" });
    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);

    EXPECT_EQ(frame.kind, FrameKind::kObjectPoses);
    ASSERT_EQ(frame.objects.size(), 2U);
    EXPECT_STREQ(frame.objects[0].name, "red_cube");
    EXPECT_STREQ(frame.objects[1].name, "green_cylinder");
    EXPECT_DOUBLE_EQ(frame.objects[1].pos[0], 2.0);
    EXPECT_DOUBLE_EQ(frame.objects[0].quat[0], 1.0);
    EXPECT_DOUBLE_EQ(frame.objects[0].size[2], 0.06);
    // The other payload interpretations must be cleared, not left over from a previous frame.
    EXPECT_TRUE(frame.points.empty());
    EXPECT_TRUE(frame.depth.empty());
    EXPECT_TRUE(bytes.empty());
}

TEST(FrameReader, RefusesAnObjectPayloadThatIsNotAWholeNumberOfRecords)
{
    auto              bytes = makeObjectFrame({ "red_cube" });
    SensorFrameHeader header{};
    std::memcpy(&header, bytes.data(), sizeof(header));
    header.payload_bytes -= 1;
    bytes.resize(bytes.size() - 1);
    std::memcpy(bytes.data(), &header, sizeof(header));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::kBadLength);
}

TEST(FrameReader, RefusesAnAbsurdObjectCountBeforeAllocating)
{
    auto              bytes = makeObjectFrame({ "red_cube" });
    SensorFrameHeader header{};
    std::memcpy(&header, bytes.data(), sizeof(header));
    header.payload_bytes =
        (kMaxObjects + 1U) * static_cast<std::uint32_t>(sizeof(grove_g1::ObjectPoseRecord));
    std::memcpy(bytes.data(), &header, sizeof(header));

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::kBadLength);
}

TEST(FrameReader, TerminatesAnObjectNameThatArrivesUnterminated)
{
    // A producer that filled all 32 bytes would otherwise leave the name running into the
    // pose that follows it, and the relay copies that name straight into a message field.
    auto bytes = makeObjectFrame({ "red_cube" });
    std::memset(
        bytes.data() + sizeof(SensorFrameHeader),
        'x',
        sizeof(grove_g1::ObjectPoseRecord::name));

    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);
    ASSERT_EQ(frame.objects.size(), 1U);
    EXPECT_EQ(std::strlen(frame.objects[0].name), sizeof(grove_g1::ObjectPoseRecord::name) - 1);
}

TEST(FrameReader, ReadsAnImuFrame)
{
    auto       bytes = makeImuFrame();
    CloudFrame frame;
    ASSERT_EQ(tryReadFrame(bytes, frame), FrameStatus::kOk);

    EXPECT_EQ(frame.kind, FrameKind::kImu);
    EXPECT_DOUBLE_EQ(frame.imu.gyro[1], 0.5);
    EXPECT_DOUBLE_EQ(frame.imu.acc[2], 9.81);
    EXPECT_DOUBLE_EQ(frame.sim_time_s, 12.25);
    EXPECT_DOUBLE_EQ(frame.sensor_quat[0], 1.0);
    // The other payload interpretations must be cleared, not left over from a previous frame.
    EXPECT_TRUE(frame.points.empty());
    EXPECT_TRUE(frame.depth.empty());
    EXPECT_TRUE(frame.objects.empty());
    EXPECT_TRUE(bytes.empty());
}

TEST(FrameReader, RefusesAnImuPayloadOfTheWrongSize)
{
    auto              bytes = makeImuFrame();
    SensorFrameHeader header{};
    std::memcpy(&header, bytes.data(), sizeof(header));
    header.payload_bytes += 8;
    std::memcpy(bytes.data(), &header, sizeof(header));
    bytes.resize(bytes.size() + 8);

    CloudFrame frame;
    EXPECT_EQ(tryReadFrame(bytes, frame), FrameStatus::kBadLength);
}
