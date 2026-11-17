#pragma once

#include <cstdint>
#include <vector>

#include "can/can_frames.h"
#include "ecu/ecu.h"
#include "sim/can_bus.hpp"
#include "sim/vehicle.hpp"

namespace sim {

// Poe a ECU entre o piloto e o carro, falando CAN de verdade por um barramento simulado
// (CanBus): a cada passo de 1 ms, fisica e piloto mandam seus quadros; na metade do passo a ECU
// recebe o que chegou, roda e manda atuadores, marcha e telemetria; na outra metade esses quadros
// chegam ao carro. Se o barramento atrasar um quadro alem do passo, a ECU ve o quadro como
// perdido (modo seguro, se for o do piloto) e o carro segue com o ultimo comando que recebeu.
// Guarda os quadros de telemetria que chegaram ao box (100 Hz).
class EcuLoop {
 public:
  EcuLoop();
  explicit EcuLoop(ecu_config_t config, CanBusConfig bus = {});

  // Um passo de 1 ms: comandos do piloto + estado do carro -> comandos para Vehicle::step.
  [[nodiscard]] Controls step(const VehicleState& state, const Controls& driver);

  [[nodiscard]] const std::vector<can_frame_t>& telemetry_frames() const noexcept {
    return telemetry_frames_;
  }
  [[nodiscard]] const CanBus& bus() const noexcept { return bus_; }

 private:
  static constexpr double kTickUs = 1000.0;

  ecu_t ecu_{};
  CanBus bus_;
  std::uint64_t tick_ = 0;
  can_command_t actuators_{};  // ultimo comando de atuadores recebido
  int gear_request_ = 0;       // ultima marcha pedida recebida
  std::vector<can_frame_t> telemetry_frames_;
};

}  // namespace sim
