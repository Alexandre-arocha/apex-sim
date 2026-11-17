#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "can/can_frames.h"
#include "sim/telemetry_client.hpp"

namespace sim {

// Topicos e tamanho maximo de quadro do broker (proto/broker.proto, server/SimTelemetry/Broker.cs).
inline constexpr const char* kTopicSessionStart = "session.start";
inline constexpr const char* kTopicSample = "telemetry.sample";
inline constexpr std::size_t kMaxBrokerFrame = std::size_t{1} << 20;

// As mensagens de proto/*.proto, codificadas na mao no formato de fio do Protobuf (varint, double
// de 64 bits little-endian, campo com tamanho; campos com valor padrao ficam de fora, como no
// proto3). Sao so tres mensagens pequenas, entao nao ha dependencia da biblioteca Protobuf; os
// mesmos bytes sao conferidos no codigo gerado do lado C# (BrokerTests.cs).
[[nodiscard]] std::string encode_sample(std::uint64_t sequence, const can_telemetry_t& telemetry);
[[nodiscard]] std::string encode_session_start(const std::string& name, double track_length_m);
// ClientMessage { publish = Publish { topic, payload } }.
[[nodiscard]] std::string encode_publish(const std::string& topic, const std::string& payload);
// 4 bytes de tamanho (big-endian) e a mensagem; vazio se ela e vazia ou passa de 1 MiB.
[[nodiscard]] std::string frame_message(const std::string& message);

// Publica no broker pub/sub do servidor de telemetria. So publica; nao assina nada.
class BrokerPublisher {
 public:
  [[nodiscard]] bool connect(const std::string& host, std::uint16_t port) {
    return link_.connect(host, port);
  }
  // false se nao ha conexao, a mensagem nao cabe num quadro ou a conexao caiu.
  [[nodiscard]] bool publish(const std::string& topic, const std::string& payload);
  void close() { link_.close(); }
  [[nodiscard]] bool connected() const noexcept { return link_.connected(); }

 private:
  TelemetryClient link_;
};

}  // namespace sim
