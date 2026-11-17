#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "can/can_frames.h"

namespace sim {

inline constexpr std::size_t kWireSize = 13;

// Registro de fio do servidor de telemetria (server/SimTelemetry/CanFrame.cs): id uint32
// little-endian, dlc e 8 bytes de dados (os que passam do dlc valem 0).
[[nodiscard]] std::array<std::uint8_t, kWireSize> to_wire(const can_frame_t& frame);

// Cliente TCP bloqueante que manda quadros CAN ao servidor de telemetria. Winsock no Windows,
// sockets POSIX nos demais.
class TelemetryClient {
 public:
  TelemetryClient() = default;
  ~TelemetryClient();
  TelemetryClient(const TelemetryClient&) = delete;
  TelemetryClient& operator=(const TelemetryClient&) = delete;

  // host pode ser nome ou endereco IP. Devolve false se nao conseguir conectar.
  [[nodiscard]] bool connect(const std::string& host, std::uint16_t port);
  // Manda um quadro inteiro; false se nao ha conexao ou ela caiu (e a fecha).
  [[nodiscard]] bool send(const can_frame_t& frame);
  // Manda bytes crus (todos ou nada na conexao); false se nao ha conexao ou ela caiu (e a fecha).
  [[nodiscard]] bool send_bytes(const std::uint8_t* data, std::size_t size);
  void close();
  [[nodiscard]] bool connected() const noexcept { return handle_ >= 0; }

 private:
  std::intptr_t handle_ = -1;
};

}  // namespace sim
