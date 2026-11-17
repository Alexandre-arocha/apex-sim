#include <gtest/gtest.h>

#include <cstdint>

#include "can/can_frames.h"
#include "sim/telemetry_client.hpp"

using sim::TelemetryClient;

TEST(TelemetryClient, WireRecordMatchesTheServerFormat) {
  can_frame_t frame;
  const can_telemetry_t telemetry{123.45, 9876.0, 0.5, 0.0, 4, 2};
  can_telemetry_pack(&telemetry, &frame);
  const auto record = sim::to_wire(frame);
  ASSERT_EQ(record.size(), 13u);
  EXPECT_EQ(record[0], 0x00);  // id 0x400 little-endian
  EXPECT_EQ(record[1], 0x04);
  EXPECT_EQ(record[2], 0x00);
  EXPECT_EQ(record[3], 0x00);
  EXPECT_EQ(record[4], 8);  // dlc
  EXPECT_EQ(record[5], 0x39);  // 12345 = 0x3039, little-endian
  EXPECT_EQ(record[6], 0x30);
  EXPECT_EQ(record[7], 0x94);  // 9876 = 0x2694
  EXPECT_EQ(record[8], 0x26);
  EXPECT_EQ(record[9], 125);  // acelerador 0,5 / 0,4%
  EXPECT_EQ(record[10], 0);   // freio
  EXPECT_EQ(record[11], 4);   // marcha
  EXPECT_EQ(record[12], 2);   // flags
}

TEST(TelemetryClient, ShortFramesArePaddedWithZeros) {
  can_frame_t frame;
  can_gear_pack(5, &frame);
  frame.data[3] = 0xFF;  // lixo alem do dlc nao pode vazar para o fio
  const auto record = sim::to_wire(frame);
  EXPECT_EQ(record[4], 1);
  EXPECT_EQ(record[5], 5);
  for (std::size_t i = 6; i < record.size(); ++i) EXPECT_EQ(record[i], 0) << i;
}

TEST(TelemetryClient, FailsCleanlyWithoutAServer) {
  TelemetryClient client;
  can_frame_t frame;
  can_gear_pack(1, &frame);
  EXPECT_FALSE(client.send(frame));  // nunca conectou
  EXPECT_FALSE(client.connected());
  EXPECT_FALSE(client.connect("127.0.0.1", 1));  // porta fechada
  EXPECT_FALSE(client.connected());
  EXPECT_FALSE(client.connect("nome.invalido.sim-corrida.test", 5055));
  EXPECT_FALSE(client.send(frame));
}
