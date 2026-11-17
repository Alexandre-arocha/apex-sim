#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "can/can_frames.h"

namespace sim {

struct CanBusConfig {
  double bitrate_bps = 1.0e6;     // CAN classico a 1 Mbit/s
  double frame_error_rate = 0.0;  // chance de um quadro ser corrompido na transmissao
  std::uint32_t seed = 1;         // semente do sorteio dos erros (determinista)
};

struct CanDelivery {
  can_frame_t frame;
  double queued_us;     // quando entrou na fila
  double delivered_us;  // quando terminou de ser transmitido
  [[nodiscard]] double latency_us() const noexcept { return delivered_us - queued_us; }
};

struct CanBusStats {
  std::uint64_t delivered = 0;
  std::uint64_t retransmissions = 0;  // quadros refeitos por erro de CRC
  double busy_us = 0.0;               // tempo com o barramento ocupado
  double max_latency_us = 0.0;
};

// Um barramento CAN classico (quadros padrao de 11 bits) com varios nos. Quem quer falar poe o
// quadro na fila; a cada instante o quadro pronto de MENOR id ganha a arbitragem e ocupa o
// barramento pelo tempo real de transmissao (bits do quadro, com pior caso de bit stuffing, a
// bitrate). Um quadro corrompido e detectado pelo CRC de todos os nos: gasta o tempo da
// transmissao mais um quadro de erro e volta a disputar a arbitragem (retransmissao automatica).
class CanBus {
 public:
  explicit CanBus(CanBusConfig config = {});

  // Poe um quadro na fila; ele so disputa o barramento a partir de at_us.
  void submit(const can_frame_t& frame, double at_us);
  // Roda o barramento ate until_us e devolve o que foi entregue, em ordem. Um quadro iniciado
  // antes de until_us termina de ser transmitido mesmo que passe dele.
  [[nodiscard]] std::vector<CanDelivery> run_until(double until_us);

  [[nodiscard]] std::size_t pending() const noexcept { return queue_.size(); }
  [[nodiscard]] double now_us() const noexcept { return clock_us_; }
  [[nodiscard]] const CanBusStats& stats() const noexcept { return stats_; }
  // Fracao do tempo simulado com o barramento ocupado.
  [[nodiscard]] double utilization() const noexcept {
    return clock_us_ > 0.0 ? stats_.busy_us / clock_us_ : 0.0;
  }

  // Bits de um quadro padrao com dlc bytes: 47 de protocolo (com o intervalo entre quadros) + 8 por
  // byte + pior caso de bit stuffing. 135 para 8 bytes.
  [[nodiscard]] static double frame_bits(std::uint8_t dlc) noexcept;

 private:
  struct Pending {
    can_frame_t frame;
    double queued_us;
  };

  [[nodiscard]] bool corrupted();

  CanBusConfig config_;
  std::vector<Pending> queue_;
  double clock_us_ = 0.0;
  CanBusStats stats_;
  std::uint32_t rng_;
};

}  // namespace sim
