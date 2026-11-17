#include "sim/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kRadPerSToRpm = 60.0 / (2.0 * std::numbers::pi);
// Abaixo desta velocidade de rolagem o angulo de deriva e regularizado: o pneu vira um
// amortecedor viscoso, o que mantem o Euler explicito estavel ate o carro parar.
constexpr double kSlipRegSpeed = 3.0;  // m/s

double unit(double x) noexcept { return std::isfinite(x) ? std::clamp(x, 0.0, 1.0) : 0.0; }

double signed_unit(double x) noexcept {
  return std::isfinite(x) ? std::clamp(x, -1.0, 1.0) : 0.0;
}

bool positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }

bool non_negative(double x) noexcept { return std::isfinite(x) && x >= 0.0; }

bool fraction(double x) noexcept { return x >= 0.0 && x <= 1.0; }  // falso para NaN

double sign(double x) noexcept { return static_cast<double>((x > 0.0) - (x < 0.0)); }

// Zera o que e pequeno demais para ser movimento (1 pm/s ou 1 prad/s). O amortecimento
// viscoso dos pneus so se aproxima do zero; sem este corte um carro parado depois de rodar
// carregaria restos denormais para sempre e speed_mps nunca voltaria a ser 0.
double settle(double rate) noexcept { return std::abs(rate) < 1e-12 ? 0.0 : rate; }

void validate(const VehicleParams& p) {
  const bool ok =
      positive(p.mass_kg) && positive(p.yaw_inertia_kgm2) && positive(p.wheelbase_m) &&
      positive(p.track_width_m) && non_negative(p.cg_height_m) &&
      p.weight_distribution_front > 0.0 && p.weight_distribution_front < 1.0 &&
      positive(p.wheel_radius_m) && non_negative(p.drag_area_m2) &&
      non_negative(p.downforce_area_m2) && fraction(p.aero_balance_front) &&
      positive(p.air_density_kgm3) && non_negative(p.rolling_coeff) &&
      fraction(p.roll_stiffness_front) && non_negative(p.max_brake_force_n) &&
      fraction(p.brake_bias_front) && positive(p.max_steer_angle_rad) &&
      p.max_steer_angle_rad < 0.5 * std::numbers::pi && positive(p.driveline_efficiency) &&
      p.driveline_efficiency <= 1.0 && positive(p.clutch_launch_rpm) &&
      non_negative(p.shift_time_s) && positive(p.downshift_rpm) &&
      p.downshift_rpm < p.upshift_rpm && p.upshift_rpm < p.engine.redline_rpm();
  if (!ok) throw std::invalid_argument("VehicleParams invalidos");

  // Em baixa velocidade os pneus amortecem a deriva e a guinada com taxa
  // (rigidez de deriva) / kSlipRegSpeed, que precisa caber no passo de 1 ms.
  const double cg_to_front = p.wheelbase_m * (1.0 - p.weight_distribution_front);
  const double cg_to_rear = p.wheelbase_m - cg_to_front;
  const double weight = p.mass_kg * kGravity;
  const double front =
      2.0 * p.front_tyre.cornering_stiffness(0.5 * weight * p.weight_distribution_front);
  const double rear =
      2.0 * p.rear_tyre.cornering_stiffness(0.5 * weight * (1.0 - p.weight_distribution_front));
  const double damping_rate =
      std::max((front + rear) / p.mass_kg,
               (cg_to_front * cg_to_front * front + cg_to_rear * cg_to_rear * rear) /
                   p.yaw_inertia_kgm2) /
      kSlipRegSpeed;
  if (damping_rate * Vehicle::kDt > 0.5) {
    throw std::invalid_argument("VehicleParams: pneus rigidos demais para o passo de 1 ms");
  }
}

}  // namespace

Vehicle::Vehicle(VehicleParams params, double initial_speed_mps) : params_(std::move(params)) {
  validate(params_);
  const double v0 = positive(initial_speed_mps) ? initial_speed_mps : 0.0;
  int gear = 1;
  while (gear < params_.gearbox.gear_count() && rpm_in_gear(gear, v0) > params_.upshift_rpm) ++gear;
  state_.speed_mps = v0;
  state_.vx_mps = v0;
  state_.gear = gear;
  state_.engine_rpm = std::max(rpm_in_gear(gear, v0), params_.engine.idle_rpm());
  state_.wheel_load_n =
      wheel_loads(0.5 * params_.air_density_kgm3 * v0 * v0 * params_.downforce_area_m2);
  target_gear_ = gear;
}

double Vehicle::rpm_in_gear(int gear, double speed_mps) const noexcept {
  return speed_mps / params_.wheel_radius_m * kRadPerSToRpm * params_.gearbox.overall_ratio(gear);
}

std::array<double, 4> Vehicle::wheel_loads(double downforce_n) const noexcept {
  const VehicleParams& p = params_;
  const double weight = p.mass_kg * kGravity;
  const double total = weight + downforce_n;
  // Eixos: peso estatico + downforce + transferencia longitudinal m*ax*h/L.
  const double pitch = p.mass_kg * state_.accel_long_mps2 * p.cg_height_m / p.wheelbase_m;
  const double front = std::clamp(
      weight * p.weight_distribution_front + downforce_n * p.aero_balance_front - pitch, 0.0,
      total);
  const double rear = total - front;
  // Lado a lado: m*ay*h/bitola, repartida pela rigidez de rolagem; a roda interna nao fica
  // com carga negativa.
  const double roll = p.mass_kg * state_.accel_lat_mps2 * p.cg_height_m / p.track_width_m;
  const double front_shift =
      std::clamp(roll * p.roll_stiffness_front, -0.5 * front, 0.5 * front);
  const double rear_shift =
      std::clamp(roll * (1.0 - p.roll_stiffness_front), -0.5 * rear, 0.5 * rear);
  return {{0.5 * front - front_shift, 0.5 * front + front_shift, 0.5 * rear - rear_shift,
           0.5 * rear + rear_shift}};
}

void Vehicle::step(Controls controls) {
  const VehicleParams& p = params_;
  const double throttle = unit(controls.throttle);
  const double brake = unit(controls.brake);
  const double steer_angle = signed_unit(controls.steer) * p.max_steer_angle_rad;
  const double vx = state_.vx_mps;
  const double vy = state_.vy_mps;
  const double yaw_rate = state_.yaw_rate_radps;

  // Cambio automatico: troca em curso corta o torque ate completar.
  if (shift_steps_left_ > 0) {
    if (--shift_steps_left_ == 0) state_.gear = target_gear_;
  } else {
    const double rpm = rpm_in_gear(state_.gear, vx);
    if (controls.gear_request > 0) {  // a ECU decide a marcha
      target_gear_ = std::min(controls.gear_request, p.gearbox.gear_count());
    } else if (rpm > p.upshift_rpm && state_.gear < p.gearbox.gear_count()) {
      target_gear_ = state_.gear + 1;
    } else if (rpm < p.downshift_rpm && state_.gear > 1) {
      target_gear_ = state_.gear - 1;
    }
    if (target_gear_ != state_.gear) {
      shift_steps_left_ = std::max(1, static_cast<int>(std::lround(p.shift_time_s / kDt)));
    }
  }
  const bool shifting = shift_steps_left_ > 0;

  // Aerodinamica pela velocidade total; cargas por roda pela aceleracao do passo anterior.
  const double speed = std::hypot(vx, vy);
  const double q = 0.5 * p.air_density_kgm3 * speed * speed;
  const double downforce = q * p.downforce_area_m2;
  const double drag = q * p.drag_area_m2;
  const std::array<double, 4> load = wheel_loads(downforce);
  // Rolamento: oposto ao sentido em que o carro rola; nulo com ele parado.
  const double rolling = sign(vx) * p.rolling_coeff * (p.mass_kg * kGravity + downforce);

  // Motor: na largada a embreagem patina e mantem o motor em clutch_launch_rpm.
  const double engine_rpm =
      std::max(rpm_in_gear(state_.gear, vx),
               throttle > 0.0 ? p.clutch_launch_rpm : p.engine.idle_rpm());
  const double drive_demand =
      shifting ? 0.0
               : p.engine.torque(engine_rpm, throttle) * p.gearbox.overall_ratio(state_.gear) *
                     p.driveline_efficiency / p.wheel_radius_m;
  const double brake_demand = brake * p.max_brake_force_n;

  // Pneus. Cada roda recebe a demanda longitudinal (tracao nas traseiras, freio repartido
  // pelo balanco) e a forca lateral de Pacejka; a resultante e limitada ao circulo de atrito.
  const double cg_to_front = p.wheelbase_m * (1.0 - p.weight_distribution_front);
  const double cg_to_rear = p.wheelbase_m - cg_to_front;
  // O freio no maximo detem a roda dentro do passo, sem inverter seu movimento. O ganho que
  // garante isso vem da inercia que uma roda enxerga (1/m + braco^2/Iz), repartida entre as
  // quatro; sem esse teto o carro parado ficaria tremendo em torno do zero.
  const double lever = std::hypot(std::max(cg_to_front, cg_to_rear), 0.5 * p.track_width_m);
  const double brake_stop_gain =
      0.25 / (kDt * (1.0 / p.mass_kg + lever * lever / p.yaw_inertia_kgm2));
  const double cos_steer = std::cos(steer_angle);
  const double sin_steer = std::sin(steer_angle);
  double force_x = 0.0;
  double force_y = 0.0;
  double yaw_moment = 0.0;
  double drive = 0.0;
  double braking = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const bool front = i < kRearLeft;
    const double px = front ? cg_to_front : -cg_to_rear;
    const double py = (i % 2 == 0 ? 0.5 : -0.5) * p.track_width_m;  // esquerda positiva
    const double c = front ? cos_steer : 1.0;
    const double s = front ? sin_steer : 0.0;
    // Velocidade do ponto de contato no referencial da roda.
    const double contact_vx = vx - yaw_rate * py;
    const double contact_vy = vy + yaw_rate * px;
    const double along = c * contact_vx + s * contact_vy;
    const double across = c * contact_vy - s * contact_vx;

    const Tyre& tyre = front ? p.front_tyre : p.rear_tyre;
    const double slip_angle = -std::atan(across / std::max(std::abs(along), kSlipRegSpeed));
    const double drive_fx = front ? 0.0 : 0.5 * drive_demand;
    const double brake_share =
        0.5 * brake_demand * (front ? p.brake_bias_front : 1.0 - p.brake_bias_front);
    const double brake_fx = std::clamp(brake_stop_gain * along, -brake_share, brake_share);
    const double demand_fx = drive_fx - brake_fx;
    const double demand_fy = tyre.lateral_force(slip_angle, load[i]);
    const double demand = std::hypot(demand_fx, demand_fy);
    const double grip = tyre.peak_force(load[i]);
    const double scale = demand > grip ? grip / demand : 1.0;
    const double fx = scale * demand_fx;
    const double fy = scale * demand_fy;

    const double body_fx = c * fx - s * fy;
    const double body_fy = s * fx + c * fy;
    force_x += body_fx;
    force_y += body_fy;
    yaw_moment += px * body_fy - py * body_fx;
    drive += scale * drive_fx;
    braking += scale * std::abs(brake_fx);
    state_.wheel_load_n[i] = load[i];
    state_.wheel_slip_angle_rad[i] = slip_angle;
    state_.wheel_fx_n[i] = fx;
    state_.wheel_fy_n[i] = fy;
  }
  if (speed > 0.0) {  // arrasto oposto ao vetor velocidade
    force_x -= drag * (vx / speed);
    force_y -= drag * (vy / speed);
  }
  force_x -= rolling;

  // Integracao (Euler semi-implicito). As forcas dao a aceleracao inercial nos eixos do
  // chassi; como ele gira yaw_rate*dt no passo, a velocidade e reexpressa nos eixos girados.
  double accel_long = force_x / p.mass_kg;
  const double accel_lat = force_y / p.mass_kg;
  double vx_free = vx + accel_long * kDt;
  if (vx * vx_free < 0.0) {
    // Ao cruzar vx = 0 o carro para nesse eixo por um passo: assim freio e rolamento o
    // detem em vez de inverter seu sentido.
    vx_free = 0.0;
    accel_long = -vx / kDt;
  }
  const double vy_free = vy + accel_lat * kDt;
  const double yaw_rate_new = settle(yaw_rate + yaw_moment / p.yaw_inertia_kgm2 * kDt);
  const double yaw_step = yaw_rate_new * kDt;
  const double vx_new = settle(std::cos(yaw_step) * vx_free + std::sin(yaw_step) * vy_free);
  const double vy_new = settle(std::cos(yaw_step) * vy_free - std::sin(yaw_step) * vx_free);
  const double yaw_new = state_.yaw_rad + yaw_step;
  const double speed_new = std::hypot(vx_new, vy_new);

  state_.step_count += 1;
  state_.time_s = static_cast<double>(state_.step_count) * kDt;
  state_.x_m += (vx_new * std::cos(yaw_new) - vy_new * std::sin(yaw_new)) * kDt;
  state_.y_m += (vx_new * std::sin(yaw_new) + vy_new * std::cos(yaw_new)) * kDt;
  state_.yaw_rad = yaw_new;
  state_.position_m += speed_new * kDt;
  state_.speed_mps = speed_new;
  state_.vx_mps = vx_new;
  state_.vy_mps = vy_new;
  state_.yaw_rate_radps = yaw_rate_new;
  state_.accel_long_mps2 = accel_long;
  state_.accel_lat_mps2 = accel_lat;
  state_.steer_angle_rad = steer_angle;
  state_.engine_rpm = engine_rpm;
  state_.shifting = shifting;
  state_.drive_force_n = drive;
  state_.brake_force_n = braking;
  state_.drag_force_n = drag;
  state_.rolling_force_n = std::abs(rolling);
  state_.downforce_n = downforce;
}

}  // namespace sim
