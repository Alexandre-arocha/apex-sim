#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "can/can_frames.h"
#include "sim/can_bus.hpp"

using sim::CanBus;
using sim::CanBusConfig;
using sim::CanDelivery;

namespace {

// CRC-15/CAN por divisao polinomial longa sobre o fluxo de bits: confere o registrador de
// deslocamento do firmware por um caminho independente.
std::uint16_t reference_crc15(std::uint32_t id, std::uint8_t dlc, const std::uint8_t* data) {
  std::vector<int> bits;
  bits.push_back(0);  // SOF
  for (int i = 10; i >= 0; --i) bits.push_back(static_cast<int>((id >> i) & 1u));
  bits.insert(bits.end(), 3, 0);  // RTR, IDE, r0
  for (int i = 3; i >= 0; --i) bits.push_back((dlc >> i) & 1);
  for (int b = 0; b < dlc; ++b) {
    for (int i = 7; i >= 0; --i) bits.push_back((data[b] >> i) & 1);
  }
  const std::size_t message_bits = bits.size();
  bits.insert(bits.end(), 15, 0);
  // Gerador x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1 = 0xC599 em 16 bits.
  constexpr int kGenerator[16] = {1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1};
  for (std::size_t i = 0; i < message_bits; ++i) {
    if (bits[i] == 0) continue;
    for (std::size_t k = 0; k < 16; ++k) bits[i + k] ^= kGenerator[k];
  }
  std::uint16_t crc = 0;
  for (std::size_t i = message_bits; i < bits.size(); ++i) {
    crc = static_cast<std::uint16_t>((static_cast<unsigned>(crc) << 1) | static_cast<unsigned>(bits[i]));
  }
  return crc;
}

can_frame_t frame_with(std::uint32_t id, std::uint8_t dlc, std::uint8_t seed) {
  can_frame_t f{};
  f.id = id;
  f.dlc = dlc;
  for (std::size_t i = 0; i < 8; ++i) f.data[i] = static_cast<std::uint8_t>(seed * 37u + i * 59u + 11u);
  f.crc = can_frame_crc(&f);
  return f;
}

}  // namespace

TEST(CanCrc, MatchesPolynomialLongDivision) {
  std::uint8_t seed = 1;
  for (const std::uint32_t id : {0u, 0x100u, 0x200u, 0x300u, 0x310u, 0x400u, 0x555u, 0x7FFu}) {
    for (std::uint8_t dlc = 0; dlc <= 8; ++dlc) {
      const can_frame_t f = frame_with(id, dlc, seed++);
      EXPECT_EQ(can_frame_crc(&f), reference_crc15(f.id, f.dlc, f.data)) << id << "/" << int{dlc};
      EXPECT_LT(can_frame_crc(&f), 0x8000u);  // 15 bits
    }
  }
}

TEST(CanCrc, DetectsEverySingleBitError) {
  const can_frame_t good = frame_with(0x300, 8, 9);
  ASSERT_TRUE(can_frame_crc_ok(&good));
  for (std::size_t bit = 0; bit < 64; ++bit) {
    can_frame_t bad = good;
    bad.data[bit / 8] ^= static_cast<std::uint8_t>(1u << (bit % 8));
    EXPECT_FALSE(can_frame_crc_ok(&bad)) << "bit de dados " << bit;
  }
  for (unsigned bit = 0; bit < 11; ++bit) {
    can_frame_t bad = good;
    bad.id ^= 1u << bit;
    EXPECT_FALSE(can_frame_crc_ok(&bad)) << "bit de id " << bit;
  }
}

TEST(CanCrc, PackFillsItAndUnpackRejectsCorruptedFrames) {
  can_frame_t frames[4];
  const can_sensors_t sensors{50.0, 9000.0, 3};
  const can_command_t command{0.5, 0.1, -0.2, 1};
  const can_telemetry_t telemetry{60.0, 8000.0, 0.4, 0.0, 4, 0};
  can_sensors_pack(&sensors, &frames[0]);
  can_command_pack(CAN_ID_DRIVER, &command, &frames[1]);
  can_gear_pack(5, &frames[2]);
  can_telemetry_pack(&telemetry, &frames[3]);
  for (const can_frame_t& f : frames) EXPECT_TRUE(can_frame_crc_ok(&f)) << f.id;

  can_sensors_t s;
  can_command_t c;
  int gear;
  can_telemetry_t t;
  for (can_frame_t& f : frames) f.data[0] ^= 0x10;  // um bit trocado em cada um
  EXPECT_EQ(can_sensors_unpack(&frames[0], &s), -1);
  EXPECT_EQ(can_command_unpack(CAN_ID_DRIVER, &frames[1], &c), -1);
  EXPECT_EQ(can_gear_unpack(&frames[2], &gear), -1);
  EXPECT_EQ(can_telemetry_unpack(&frames[3], &t), -1);
}

TEST(CanBus, FrameBitsFollowTheStandardWorstCase) {
  EXPECT_EQ(CanBus::frame_bits(8), 135.0);  // o numero classico para 8 bytes
  EXPECT_EQ(CanBus::frame_bits(5), 105.0);
  EXPECT_EQ(CanBus::frame_bits(0), 55.0);
}

TEST(CanBus, LowestIdWinsArbitration) {
  CanBus bus;
  bus.submit(frame_with(0x300, 1, 1), 0.0);
  bus.submit(frame_with(0x100, 5, 2), 0.0);
  bus.submit(frame_with(0x200, 7, 3), 0.0);
  const std::vector<CanDelivery> out = bus.run_until(10000.0);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0].frame.id, 0x100u);
  EXPECT_EQ(out[1].frame.id, 0x200u);
  EXPECT_EQ(out[2].frame.id, 0x300u);
  EXPECT_LT(out[0].delivered_us, out[1].delivered_us);
  EXPECT_LT(out[1].delivered_us, out[2].delivered_us);
}

TEST(CanBus, TransmissionTakesTheTimeOfItsBitsAtTheBitrate) {
  CanBus bus;  // 1 Mbit/s: 1 bit = 1 us
  bus.submit(frame_with(0x100, 8, 1), 0.0);
  bus.submit(frame_with(0x200, 8, 2), 0.0);
  const std::vector<CanDelivery> out = bus.run_until(1000.0);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out[0].delivered_us, 135.0);
  EXPECT_DOUBLE_EQ(out[1].delivered_us, 270.0);
  EXPECT_DOUBLE_EQ(out[1].latency_us(), 270.0);  // esperou o primeiro

  CanBus slow{CanBusConfig{.bitrate_bps = 500e3}};
  slow.submit(frame_with(0x100, 8, 1), 0.0);
  EXPECT_DOUBLE_EQ(slow.run_until(1000.0).at(0).delivered_us, 270.0);
}

TEST(CanBus, FramesWaitUntilTheyAreReadyAndFinishWhatTheyStarted) {
  CanBus bus;
  bus.submit(frame_with(0x100, 8, 1), 500.0);
  EXPECT_TRUE(bus.run_until(400.0).empty());
  EXPECT_DOUBLE_EQ(bus.now_us(), 400.0);
  EXPECT_EQ(bus.pending(), 1u);
  // Comeca em 500 e termina em 635, mesmo com o limite em 520.
  const std::vector<CanDelivery> out = bus.run_until(520.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].delivered_us, 635.0);
  EXPECT_DOUBLE_EQ(out[0].latency_us(), 135.0);
  EXPECT_GE(bus.now_us(), 635.0);
}

TEST(CanBus, OverloadBuildsABacklogAndSaturatesTheBus) {
  CanBus bus;
  for (int ms = 0; ms < 50; ++ms) {
    for (std::uint8_t k = 0; k < 20; ++k) bus.submit(frame_with(0x100u + k, 8, k), ms * 1000.0);
    static_cast<void>(bus.run_until((ms + 1) * 1000.0));  // 20 x 135 us = 2,7 ms de carga por ms
  }
  EXPECT_GT(bus.pending(), 500u);
  EXPECT_GT(bus.utilization(), 0.99);
  EXPECT_GT(bus.stats().max_latency_us, 10000.0);
}

TEST(CanBus, CorruptedFramesAreDetectedByCrcAndRetransmitted) {
  CanBus bus{CanBusConfig{.frame_error_rate = 0.3, .seed = 7}};
  std::vector<can_frame_t> sent;
  for (std::uint8_t i = 0; i < 100; ++i) {
    sent.push_back(frame_with(0x100u + i, static_cast<std::uint8_t>(i % 9), i));
    bus.submit(sent.back(), 0.0);
  }
  const std::vector<CanDelivery> out = bus.run_until(1.0e7);
  ASSERT_EQ(out.size(), sent.size());          // nenhum se perde: o erro e refeito
  EXPECT_GT(bus.stats().retransmissions, 10u);  // e houve erros de fato
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_EQ(out[i].frame.id, sent[i].id);  // ainda por prioridade de id
    EXPECT_TRUE(can_frame_crc_ok(&out[i].frame));
    for (std::size_t b = 0; b < sent[i].dlc; ++b) EXPECT_EQ(out[i].frame.data[b], sent[i].data[b]);
  }
}

TEST(CanBus, ErrorsAreDeterministicPerSeed) {
  const auto retransmissions = [](std::uint32_t seed) {
    CanBus bus{CanBusConfig{.frame_error_rate = 0.2, .seed = seed}};
    for (std::uint8_t i = 0; i < 200; ++i) bus.submit(frame_with(0x100u + i, 8, i), 0.0);
    static_cast<void>(bus.run_until(1.0e8));
    return bus.stats().retransmissions;
  };
  EXPECT_EQ(retransmissions(5), retransmissions(5));
  EXPECT_NE(retransmissions(5), retransmissions(6));
}

TEST(CanBus, ACleanBusNeverRetransmits) {
  CanBus bus;
  for (std::uint8_t i = 0; i < 50; ++i) bus.submit(frame_with(0x100u + i, 8, i), 0.0);
  EXPECT_EQ(bus.run_until(1.0e7).size(), 50u);
  EXPECT_EQ(bus.stats().retransmissions, 0u);
}
