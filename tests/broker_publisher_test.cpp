#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "can/can_frames.h"
#include "sim/broker_publisher.hpp"

namespace {

std::string bytes(std::initializer_list<int> values) {
  std::string out;
  for (const int v : values) out.push_back(static_cast<char>(v));
  return out;
}

}  // namespace

// Os mesmos bytes sao conferidos no lado C# (server/SimTelemetry.Tests/BrokerTests.cs), gerados
// pelo codigo do protoc: os dois lados concordam sobre o fio sem compartilhar codigo.
TEST(BrokerPublisher, SampleMatchesTheProtobufGoldenBytes) {
  const can_telemetry_t telemetry{1.5, 0.0, 0.0, 0.0, 4, 0};
  EXPECT_EQ(sim::encode_sample(300, telemetry),
            bytes({0x08, 0xAC, 0x02, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F, 0x30, 0x04}));
}

TEST(BrokerPublisher, SessionStartAndPublishMatchTheGoldenBytes) {
  EXPECT_EQ(sim::encode_session_start("pista", 1000.0),
            bytes({0x0A, 0x05, 0x70, 0x69, 0x73, 0x74, 0x61, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x8F, 0x40}));
  EXPECT_EQ(sim::encode_publish("t", bytes({1, 2})), bytes({0x12, 0x07, 0x0A, 0x01, 0x74, 0x12, 0x02, 0x01, 0x02}));
}

TEST(BrokerPublisher, DefaultValuesStayOutOfTheWireLikeProto3) {
  const can_telemetry_t zero{0.0, 0.0, 0.0, 0.0, 0, 0};
  EXPECT_TRUE(sim::encode_sample(0, zero).empty());
  EXPECT_TRUE(sim::encode_session_start("", 0.0).empty());
}

TEST(BrokerPublisher, EncodesEveryFieldAndLargeSequences) {
  const can_telemetry_t telemetry{83.25, 14500.0, 0.5, 1.0, 8, 7};
  const std::string out = sim::encode_sample(1ull << 40, telemetry);
  // sequencia 2^40: varint de 6 bytes (0x80 x5, 0x20); depois 4 doubles (9 bytes cada) e 2 varints.
  EXPECT_EQ(out.substr(0, 7), bytes({0x08, 0x80, 0x80, 0x80, 0x80, 0x80, 0x20}));
  EXPECT_EQ(out.size(), 7u + 4u * 9u + 2u + 2u);
  EXPECT_EQ(out.substr(out.size() - 4), bytes({0x30, 0x08, 0x38, 0x07}));
}

TEST(BrokerPublisher, FramesCarryABigEndianLength) {
  const std::string message(300, 'x');
  const std::string frame = sim::frame_message(message);
  ASSERT_EQ(frame.size(), 304u);
  EXPECT_EQ(frame.substr(0, 4), bytes({0x00, 0x00, 0x01, 0x2C}));
  EXPECT_EQ(frame.substr(4), message);
  EXPECT_TRUE(sim::frame_message("").empty());
  EXPECT_FALSE(sim::frame_message(std::string(sim::kMaxBrokerFrame, 'x')).empty());
  EXPECT_TRUE(sim::frame_message(std::string(sim::kMaxBrokerFrame + 1, 'x')).empty());
}

TEST(BrokerPublisher, FailsCleanlyWithoutABroker) {
  sim::BrokerPublisher publisher;
  EXPECT_FALSE(publisher.publish(sim::kTopicSample, "x"));  // nunca conectou
  EXPECT_FALSE(publisher.connect("127.0.0.1", 1));          // porta fechada
  EXPECT_FALSE(publisher.connected());
  EXPECT_FALSE(publisher.publish(sim::kTopicSample, "x"));
}
