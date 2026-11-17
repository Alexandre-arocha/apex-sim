#include "sim/can_bus.hpp"

#include <algorithm>
#include <limits>

namespace sim {
namespace {

constexpr double kErrorFrameBits = 20.0;

}  // namespace

CanBus::CanBus(CanBusConfig config) : config_(config), rng_(config.seed) {}

double CanBus::frame_bits(std::uint8_t dlc) noexcept {
  const unsigned bytes = std::min<unsigned>(dlc, 8u);
  return 47.0 + 8.0 * bytes + static_cast<double>((34u + 8u * bytes - 1u) / 4u);
}

void CanBus::submit(const can_frame_t& frame, double at_us) { queue_.push_back({frame, at_us}); }

bool CanBus::corrupted() {
  if (!(config_.frame_error_rate > 0.0)) return false;
  rng_ = rng_ * 1664525u + 1013904223u;  // LCG: determinista entre plataformas
  return static_cast<double>(rng_) / 4294967296.0 < config_.frame_error_rate;
}

std::vector<CanDelivery> CanBus::run_until(double until_us) {
  std::vector<CanDelivery> delivered;
  const double bit_us = 1.0e6 / config_.bitrate_bps;

  while (clock_us_ < until_us) {
    // Arbitragem: entre os quadros ja prontos, o de menor id (empate: o que entrou antes).
    std::size_t winner = queue_.size();
    double next_ready_us = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < queue_.size(); ++i) {
      if (queue_[i].queued_us > clock_us_) {
        next_ready_us = std::min(next_ready_us, queue_[i].queued_us);
      } else if (winner == queue_.size() || queue_[i].frame.id < queue_[winner].frame.id) {
        winner = i;
      }
    }
    if (winner == queue_.size()) {  // barramento ocioso
      clock_us_ = std::min(until_us, next_ready_us);
      continue;
    }

    const Pending sending = queue_[winner];
    const double duration_us = frame_bits(sending.frame.dlc) * bit_us;
    if (corrupted()) {
      // Um bit trocado no caminho: o CRC de quem recebe nao confere, o erro e sinalizado e o
      // quadro continua na fila para ser retransmitido.
      can_frame_t garbled = sending.frame;
      if (garbled.dlc > 0) {
        garbled.data[0] ^= 1u;
      } else {
        garbled.id ^= 1u;
      }
      const double cost_us = duration_us + kErrorFrameBits * bit_us;
      clock_us_ += cost_us;
      stats_.busy_us += cost_us;
      if (!can_frame_crc_ok(&garbled)) stats_.retransmissions += 1;
      continue;
    }

    clock_us_ += duration_us;
    stats_.busy_us += duration_us;
    stats_.delivered += 1;
    const CanDelivery delivery{sending.frame, sending.queued_us, clock_us_};
    stats_.max_latency_us = std::max(stats_.max_latency_us, delivery.latency_us());
    delivered.push_back(delivery);
    queue_.erase(queue_.begin() + static_cast<std::ptrdiff_t>(winner));
  }
  return delivered;
}

}  // namespace sim
