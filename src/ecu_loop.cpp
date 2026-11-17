#include "sim/ecu_loop.hpp"

namespace sim {

EcuLoop::EcuLoop() : EcuLoop(ecu_default_config()) {}

EcuLoop::EcuLoop(ecu_config_t config, CanBusConfig bus) : bus_(bus) { ecu_init(&ecu_, config); }

Controls EcuLoop::step(const VehicleState& state, const Controls& driver) {
  const double start_us = static_cast<double>(tick_) * kTickUs;
  tick_ += 1;

  // Fisica e piloto falam no inicio do passo.
  const can_sensors_t sensors{state.speed_mps, state.engine_rpm, state.gear};
  const can_command_t wanted{driver.throttle, driver.brake, driver.steer, 0};
  can_frame_t frame;
  can_sensors_pack(&sensors, &frame);
  bus_.submit(frame, start_us);
  can_command_pack(CAN_ID_DRIVER, &wanted, &frame);
  bus_.submit(frame, start_us);

  // O que chega ao carro e ao box vale a qualquer momento, mesmo um quadro atrasado do passo
  // anterior que so agora conseguiu o barramento.
  const auto absorb = [this](const can_frame_t& received) {
    if (received.id == CAN_ID_ACTUATORS) {
      static_cast<void>(can_command_unpack(CAN_ID_ACTUATORS, &received, &actuators_));
    } else if (received.id == CAN_ID_GEAR) {
      static_cast<void>(can_gear_unpack(&received, &gear_request_));
    } else if (received.id == CAN_ID_TELEMETRY) {
      telemetry_frames_.push_back(received);
    }
  };

  // Primeira metade: o que chegou a ECU. Quadro que nao chegou a tempo conta como perdido.
  can_frame_t sensors_frame;
  can_frame_t driver_frame;
  bool have_sensors = false;
  bool have_driver = false;
  for (const CanDelivery& d : bus_.run_until(start_us + 0.5 * kTickUs)) {
    if (d.frame.id == CAN_ID_SENSORS) {
      sensors_frame = d.frame;
      have_sensors = true;
    } else if (d.frame.id == CAN_ID_DRIVER) {
      driver_frame = d.frame;
      have_driver = true;
    } else {
      absorb(d.frame);
    }
  }

  can_frame_t actuators_frame;
  can_frame_t telemetry_frame;
  const bool has_telemetry =
      ecu_step(&ecu_, have_driver ? &driver_frame : nullptr, have_sensors ? &sensors_frame : nullptr,
               &actuators_frame, &telemetry_frame) != 0;
  const double ecu_us = start_us + 0.5 * kTickUs;
  bus_.submit(actuators_frame, ecu_us);
  can_frame_t gear_frame;
  ecu_gear_frame(&ecu_, &gear_frame);
  bus_.submit(gear_frame, ecu_us);
  if (has_telemetry) bus_.submit(telemetry_frame, ecu_us);

  // Segunda metade: o que chegou ao carro e ao box.
  for (const CanDelivery& d : bus_.run_until(start_us + kTickUs)) absorb(d.frame);
  return Controls{actuators_.throttle, actuators_.brake, actuators_.steer, gear_request_};
}

}  // namespace sim
