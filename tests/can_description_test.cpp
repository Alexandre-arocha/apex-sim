// Confere can/sim.dbc (a descricao dos quadros no formato DBC) contra o codigo C que os monta:
// codifica valores fisicos so pelo que o arquivo diz (bit inicial, tamanho, sinal, escala) e exige
// os mesmos bytes que can_*_pack produz.
#include <gtest/gtest.h>

#include <bitset>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "can/can_frames.h"

namespace {

struct Signal {
  std::string name;
  int start = 0;
  int length = 0;
  bool is_signed = false;
  double scale = 1.0;
  double offset = 0.0;
};

struct Message {
  unsigned id = 0;
  std::string name;
  int dlc = 0;
  std::vector<Signal> signals;
};

std::vector<Message> load_dbc() {
  std::ifstream file(std::string(SIM_SOURCE_DIR) + "/can/sim.dbc");
  EXPECT_TRUE(file.is_open()) << "can/sim.dbc nao encontrado";
  std::vector<Message> messages;
  std::string line;
  while (std::getline(file, line)) {
    unsigned id = 0;
    int dlc = 0;
    char name[64] = {};
    if (std::sscanf(line.c_str(), "BO_ %u %63[^:]: %d", &id, name, &dlc) == 3) {
      messages.push_back(Message{id, name, dlc, {}});
      continue;
    }
    int start = 0;
    int length = 0;
    char sign = '+';
    double scale = 1.0;
    double offset = 0.0;
    if (!messages.empty() &&
        std::sscanf(line.c_str(), " SG_ %63s : %d|%d@1%c (%lf,%lf)", name, &start, &length, &sign,
                    &scale, &offset) == 6) {
      messages.back().signals.push_back(Signal{name, start, length, sign == '-', scale, offset});
    }
  }
  return messages;
}

// Codificacao generica de um sinal DBC little-endian (Intel).
std::vector<std::uint8_t> encode(const Message& m, const std::map<std::string, double>& values) {
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(m.dlc), 0);
  for (const Signal& s : m.signals) {
    long long raw = std::llround((values.at(s.name) - s.offset) / s.scale);
    if (raw < 0) raw += 1LL << s.length;  // complemento de dois no tamanho do sinal
    for (int bit = 0; bit < s.length; ++bit) {
      if (((raw >> bit) & 1) != 0) {
        bytes[static_cast<std::size_t>((s.start + bit) / 8)] |=
            static_cast<std::uint8_t>(1u << ((s.start + bit) % 8));
      }
    }
  }
  return bytes;
}

const Message& message(const std::vector<Message>& all, unsigned id) {
  for (const Message& m : all) {
    if (m.id == id) return m;
  }
  ADD_FAILURE() << "mensagem " << id << " ausente do DBC";
  static const Message none;
  return none;
}

void expect_same_bytes(const Message& m, const std::map<std::string, double>& values,
                       const can_frame_t& from_c) {
  const std::vector<std::uint8_t> from_dbc = encode(m, values);
  EXPECT_EQ(from_c.id, m.id) << m.name;
  ASSERT_EQ(from_c.dlc, m.dlc) << m.name;
  for (std::size_t i = 0; i < from_dbc.size(); ++i) {
    EXPECT_EQ(from_c.data[i], from_dbc[i]) << m.name << " byte " << i;
  }
}

}  // namespace

TEST(CanDescription, DeclaresTheFiveFramesWithoutOverlappingSignals) {
  const std::vector<Message> all = load_dbc();
  ASSERT_EQ(all.size(), 5u);
  for (const Message& m : all) {
    std::bitset<64> used;
    for (const Signal& s : m.signals) {
      ASSERT_LE(s.start + s.length, m.dlc * 8) << m.name << "." << s.name << " passa do dlc";
      for (int bit = s.start; bit < s.start + s.length; ++bit) {
        EXPECT_FALSE(used[static_cast<std::size_t>(bit)]) << m.name << "." << s.name << " sobrepoe";
        used.set(static_cast<std::size_t>(bit));
      }
    }
    EXPECT_FALSE(m.signals.empty()) << m.name;
  }
}

TEST(CanDescription, SensorsMatchTheFirmware) {
  const std::vector<Message> all = load_dbc();
  can_frame_t frame;
  const can_sensors_t in{123.45, 9876.0, 5};
  can_sensors_pack(&in, &frame);
  expect_same_bytes(message(all, CAN_ID_SENSORS), {{"speed", 123.45}, {"rpm", 9876}, {"gear", 5}},
                    frame);
}

TEST(CanDescription, DriverAndActuatorsShareOneLayoutThatMatchesTheFirmware) {
  const std::vector<Message> all = load_dbc();
  const can_command_t in{0.6, 0.25, -0.4, 3};
  const std::map<std::string, double> values{
      {"throttle", 0.6}, {"brake", 0.25}, {"steer", -0.4}, {"flags", 3}};
  for (const unsigned id : {static_cast<unsigned>(CAN_ID_DRIVER), static_cast<unsigned>(CAN_ID_ACTUATORS)}) {
    can_frame_t frame;
    can_command_pack(id, &in, &frame);
    expect_same_bytes(message(all, id), values, frame);
  }
}

TEST(CanDescription, GearAndTelemetryMatchTheFirmware) {
  const std::vector<Message> all = load_dbc();
  can_frame_t frame;
  can_gear_pack(6, &frame);
  expect_same_bytes(message(all, CAN_ID_GEAR), {{"gear_request", 6}}, frame);

  const can_telemetry_t telemetry{88.88, 7000.0, 0.5, 0.2, 4, 2};
  can_telemetry_pack(&telemetry, &frame);
  expect_same_bytes(message(all, CAN_ID_TELEMETRY),
                    {{"speed", 88.88}, {"rpm", 7000}, {"throttle", 0.5}, {"brake", 0.2}, {"gear", 4}, {"flags", 2}},
                    frame);
}
