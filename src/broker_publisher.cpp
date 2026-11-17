#include "sim/broker_publisher.hpp"

#include <cstring>

namespace sim {
namespace {

constexpr unsigned kVarint = 0;
constexpr unsigned kFixed64 = 1;
constexpr unsigned kLengthDelimited = 2;

void put_varint(std::string& out, std::uint64_t value) {
  while (value >= 0x80) {
    out.push_back(static_cast<char>((value & 0x7F) | 0x80));
    value >>= 7;
  }
  out.push_back(static_cast<char>(value));
}

void put_tag(std::string& out, unsigned field, unsigned wire_type) {
  put_varint(out, (std::uint64_t{field} << 3) | wire_type);
}

void put_uint(std::string& out, unsigned field, std::uint64_t value) {
  if (value == 0) return;
  put_tag(out, field, kVarint);
  put_varint(out, value);
}

void put_double(std::string& out, unsigned field, double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof bits);
  if (bits == 0) return;  // so o +0,0 e o valor padrao; -0,0 vai ao fio, como no proto3
  put_tag(out, field, kFixed64);
  for (int i = 0; i < 8; ++i) out.push_back(static_cast<char>((bits >> (8 * i)) & 0xFF));
}

void put_bytes(std::string& out, unsigned field, const std::string& value) {
  if (value.empty()) return;
  put_tag(out, field, kLengthDelimited);
  put_varint(out, value.size());
  out += value;
}

}  // namespace

std::string encode_sample(std::uint64_t sequence, const can_telemetry_t& telemetry) {
  std::string out;
  put_uint(out, 1, sequence);
  put_double(out, 2, telemetry.speed_mps);
  put_double(out, 3, telemetry.rpm);
  put_double(out, 4, telemetry.throttle);
  put_double(out, 5, telemetry.brake);
  put_uint(out, 6, telemetry.gear < 0 ? 0u : static_cast<std::uint64_t>(telemetry.gear));
  put_uint(out, 7, telemetry.flags);
  return out;
}

std::string encode_session_start(const std::string& name, double track_length_m) {
  std::string out;
  put_bytes(out, 1, name);
  put_double(out, 2, track_length_m);
  return out;
}

std::string encode_publish(const std::string& topic, const std::string& payload) {
  std::string inner;
  put_bytes(inner, 1, topic);
  put_bytes(inner, 2, payload);
  std::string out;
  put_tag(out, 2, kLengthDelimited);  // ClientMessage.publish; o "oneof" sai mesmo se vazio
  put_varint(out, inner.size());
  out += inner;
  return out;
}

std::string frame_message(const std::string& message) {
  if (message.empty() || message.size() > kMaxBrokerFrame) return {};
  std::string frame;
  frame.reserve(4 + message.size());
  for (int shift = 24; shift >= 0; shift -= 8) {
    frame.push_back(static_cast<char>((message.size() >> shift) & 0xFF));
  }
  frame += message;
  return frame;
}

bool BrokerPublisher::publish(const std::string& topic, const std::string& payload) {
  const std::string frame = frame_message(encode_publish(topic, payload));
  if (frame.empty()) return false;
  return link_.send_bytes(reinterpret_cast<const std::uint8_t*>(frame.data()), frame.size());
}

}  // namespace sim
