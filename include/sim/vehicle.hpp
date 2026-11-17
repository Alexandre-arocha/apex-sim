#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "sim/engine.hpp"
#include "sim/gearbox.hpp"
#include "sim/tyre.hpp"

namespace sim {

// Indices das rodas nos vetores por roda de VehicleState.
enum Wheel : std::size_t { kFrontLeft, kFrontRight, kRearLeft, kRearRight };

struct VehicleParams {
  double mass_kg = 798.0;
  double yaw_inertia_kgm2 = 1100.0;
  double wheelbase_m = 3.6;
  double track_width_m = 1.6;
  double cg_height_m = 0.25;
  double weight_distribution_front = 0.46;  // fracao do peso estatico no eixo dianteiro
  double wheel_radius_m = 0.36;
  double drag_area_m2 = 1.3;         // Cd * A
  double downforce_area_m2 = 5.2;    // Cl * A (sustentacao/arrasto = 4)
  double aero_balance_front = 0.44;  // fracao do downforce no eixo dianteiro
  double air_density_kgm3 = 1.225;
  double rolling_coeff = 0.015;      // multiplica a carga normal (peso + downforce)
  double roll_stiffness_front = 0.55;  // fracao da transferencia lateral de carga na dianteira
  double max_brake_force_n = 38000.0;
  double brake_bias_front = 0.58;      // fracao da forca de freio no eixo dianteiro
  double max_steer_angle_rad = 0.35;   // angulo das rodas dianteiras com steer = +-1
  double driveline_efficiency = 0.93;
  double clutch_launch_rpm = 8000.0;  // rpm do motor patinando a embreagem na largada
  double shift_time_s = 0.03;         // torque cortado durante a troca
  double upshift_rpm = 14000.0;
  double downshift_rpm = 8500.0;
  Engine engine = Engine::reference();
  Gearbox gearbox = Gearbox::reference();
  Tyre front_tyre = Tyre::reference_front();
  Tyre rear_tyre = Tyre::reference_rear();
};

struct Controls {
  double throttle = 0.0;  // [0,1]; valores fora da faixa sao saturados, nao finitos viram 0
  double brake = 0.0;     // [0,1]
  double steer = 0.0;     // [-1,1], positivo = esquerda; angulo = steer * max_steer_angle_rad
  int gear_request = 0;   // 0 = cambio automatico do carro; 1..N pede essa marcha (satura em N)
};

struct VehicleState {
  std::uint64_t step_count = 0;
  double time_s = 0.0;
  // Mundo: x, y (m) e yaw anti-horario (rad, nao normalizado). O carro parte da origem
  // apontando para +x.
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double position_m = 0.0;  // distancia percorrida
  double speed_mps = 0.0;   // modulo da velocidade
  // Chassi (ISO 8855): x para frente, y para a esquerda.
  double vx_mps = 0.0;  // negativo so quando o carro roda e desliza de re
  double vy_mps = 0.0;
  double yaw_rate_radps = 0.0;
  double accel_long_mps2 = 0.0;  // o que um acelerometro no CG mediria
  double accel_lat_mps2 = 0.0;
  double steer_angle_rad = 0.0;  // angulo das rodas dianteiras
  double engine_rpm = 0.0;
  int gear = 1;
  bool shifting = false;
  // Forcas do ultimo passo (N), todas >= 0.
  double drive_force_n = 0.0;  // tracao entregue pelas rodas traseiras
  double brake_force_n = 0.0;  // freio entregue pelas quatro rodas
  double drag_force_n = 0.0;
  double rolling_force_n = 0.0;
  double downforce_n = 0.0;
  // Por roda (indices de Wheel); fx e fy no referencial da roda.
  std::array<double, 4> wheel_load_n{};
  std::array<double, 4> wheel_slip_angle_rad{};
  std::array<double, 4> wheel_fx_n{};
  std::array<double, 4> wheel_fy_n{};
};

// Modelo plano de quatro rodas em passo fixo de 1 ms: motor, cambio automatico, aerodinamica,
// transferencia de carga quase-estatica e pneus de Pacejka limitados pelo circulo de atrito.
// Simplificacoes: sem marcha a re (o carro so anda para tras se rodar), sem freio-motor, sem
// dinamica de giro das rodas (o pneu entrega a forca longitudinal pedida ate o limite de
// aderencia) e chassi rigido (a carga das rodas responde a aceleracao do passo anterior).
class Vehicle {
 public:
  static constexpr double kDt = 0.001;  // s

  // Lanca std::invalid_argument se algum parametro for invalido.
  // initial_speed_mps < 0 ou nao finito e tratado como 0; a marcha inicial
  // e a mais baixa cujo rpm nao excede upshift_rpm.
  explicit Vehicle(VehicleParams params, double initial_speed_mps = 0.0);

  void step(Controls controls);
  [[nodiscard]] const VehicleState& state() const noexcept { return state_; }
  [[nodiscard]] const VehicleParams& params() const noexcept { return params_; }

 private:
  [[nodiscard]] double rpm_in_gear(int gear, double speed_mps) const noexcept;
  [[nodiscard]] std::array<double, 4> wheel_loads(double downforce_n) const noexcept;

  VehicleParams params_;
  VehicleState state_;
  int target_gear_ = 1;
  int shift_steps_left_ = 0;
};

}  // namespace sim
