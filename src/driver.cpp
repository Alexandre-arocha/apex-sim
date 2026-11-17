#include "sim/driver.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kRadPerSToRpm = 60.0 / (2.0 * std::numbers::pi);
constexpr double kSpeedCap = 150.0;  // m/s: teto do limite, acima do que o carro alcanca
constexpr int kBisections = 30;
constexpr double kSlopeStep = 0.01;  // m: passo para medir a inclinacao do limite de velocidade

bool positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }

double downforce_n(const VehicleParams& p, double speed) noexcept {
  return 0.5 * p.air_density_kgm3 * speed * speed * p.downforce_area_m2;
}

// Arrasto mais rolamento (N).
double resistance_n(const VehicleParams& p, double speed) noexcept {
  return 0.5 * p.air_density_kgm3 * speed * speed * p.drag_area_m2 +
         p.rolling_coeff * (p.mass_kg * kGravity + downforce_n(p, speed));
}

// Aderencia (N) das duas rodas de um eixo: em curva a de dentro perde carga para a de fora.
struct AxleGrip {
  double inside_n;
  double outside_n;
  [[nodiscard]] double total_n() const noexcept { return inside_n + outside_n; }
};

struct CarGrip {
  AxleGrip front;
  AxleGrip rear;
};

// Aderencia de cada roda com o carro a `speed`, acelerando accel_long e fazendo lat_accel de
// aceleracao lateral, com as mesmas cargas quase-estaticas que Vehicle usa: peso, downforce e
// transferencia longitudinal e lateral.
CarGrip wheel_grip(const VehicleParams& p, double speed, double accel_long,
                   double lat_accel) noexcept {
  const double weight = p.mass_kg * kGravity;
  const double downforce = downforce_n(p, speed);
  const double front =
      std::clamp(weight * p.weight_distribution_front + downforce * p.aero_balance_front -
                     p.mass_kg * accel_long * p.cg_height_m / p.wheelbase_m,
                 0.0, weight + downforce);
  const double rear = weight + downforce - front;
  const double roll = p.mass_kg * std::abs(lat_accel) * p.cg_height_m / p.track_width_m;
  const double front_shift = std::min(roll * p.roll_stiffness_front, 0.5 * front);
  const double rear_shift = std::min(roll * (1.0 - p.roll_stiffness_front), 0.5 * rear);
  return {{p.front_tyre.peak_force(0.5 * front - front_shift),
           p.front_tyre.peak_force(0.5 * front + front_shift)},
          {p.rear_tyre.peak_force(0.5 * rear - rear_shift),
           p.rear_tyre.peak_force(0.5 * rear + rear_shift)}};
}

// Forca longitudinal (N) que cada roda de um eixo ainda transmite enquanto o eixo faz a forca
// lateral lateral_n sem passar da fracao `usage` da aderencia. As duas rodas trabalham com a
// mesma deriva e dividem a forca lateral na proporcao da aderencia; a de dentro e a que limita.
double wheel_reserve(const AxleGrip& grip, double lateral_n, double usage) noexcept {
  if (!(grip.total_n() > 0.0)) return 0.0;
  const double lateral_usage = lateral_n / grip.total_n();
  return grip.inside_n * std::sqrt(std::max(0.0, usage * usage - lateral_usage * lateral_usage));
}

// Angulo de deriva (rad) com que um eixo de aderencia grip_n faz a forca lateral force_n: o
// inverso da formula de Pacejka, saturado no pico.
double slip_angle(const Tyre& tyre, double grip_n, double force_n) noexcept {
  if (!(grip_n > 0.0)) return 0.0;
  const double usage = std::clamp(force_n / grip_n, -1.0, 1.0);
  return std::tan(std::asin(usage) / tyre.params().shape_c) / tyre.params().stiffness_b;
}

// Maior valor em [0, upper] que `fits` aceita, por bissecao. Supoe que `fits` aceite um
// intervalo que comeca em 0.
template <typename Fits>
double largest_fitting(double upper, Fits fits) noexcept {
  if (fits(upper)) return upper;
  double low = 0.0;
  double high = upper;
  for (int i = 0; i < kBisections; ++i) {
    const double mid = 0.5 * (low + high);
    if (fits(mid)) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return low;
}

// Forca de tracao do motor a fundo (N nas rodas), na marcha que o cambio usa acelerando.
double engine_force_n(const VehicleParams& p, double speed) noexcept {
  const double wheel_rpm = speed / p.wheel_radius_m * kRadPerSToRpm;
  int gear = 1;
  while (gear < p.gearbox.gear_count() &&
         wheel_rpm * p.gearbox.overall_ratio(gear) > p.upshift_rpm) {
    ++gear;
  }
  const double ratio = p.gearbox.overall_ratio(gear);
  const double rpm = std::max(wheel_rpm * ratio, p.clutch_launch_rpm);
  return p.engine.torque(rpm, 1.0) * ratio * p.driveline_efficiency / p.wheel_radius_m;
}

}  // namespace

Driver::Driver(Track track, VehicleParams car, DriverParams params)
    : track_(std::move(track)), car_(std::move(car)), params_(params) {
  static_cast<void>(Vehicle{car_});  // valida car pelos criterios de Vehicle
  if (!(positive(params_.grip_usage) && positive(params_.steer_preview_s) &&
        positive(params_.min_preview_m) && positive(params_.speed_response_s) &&
        car_.max_brake_force_n > 0.0)) {
    throw std::invalid_argument("DriverParams invalidos");
  }
  plan();
}

Controls Driver::drive(const VehicleState& state) {
  position_ = located_ ? track_.locate(state.x_m, state.y_m, position_.s_m)
                       : track_.locate(state.x_m, state.y_m);
  located_ = true;
  const TrackPoint line = track_.at(position_.s_m);
  const double speed = state.speed_mps;
  const double front_mass = car_.mass_kg * car_.weight_distribution_front;

  // Direcao. Curvatura pedida: a da linha mais a correcao que traz o carro de volta a ela na
  // distancia de mira, medindo o desvio lateral e o erro de rumo da velocidade (nao do chassi,
  // que derrapa um pouco em curva). Angulo das rodas: o geometrico para essa curvatura mais a
  // diferenca entre as derivas de que os pneus dianteiros e traseiros precisam para faze-la.
  const double drift = speed > 1.0 ? std::atan2(state.vy_mps, state.vx_mps) : 0.0;
  const double course_error =
      std::remainder(state.yaw_rad + drift - line.heading_rad, 2.0 * std::numbers::pi);
  const double preview = std::max(params_.min_preview_m, params_.steer_preview_s * speed);
  const double curvature =
      line.curvature_1pm -
      2.0 * (position_.lateral_m + preview * std::sin(course_error)) / (preview * preview);
  const double lat_accel_wanted = speed * speed * curvature;
  const CarGrip grip = wheel_grip(car_, speed, state.accel_long_mps2, lat_accel_wanted);
  const double steer_angle =
      std::atan(car_.wheelbase_m * curvature) +
      slip_angle(car_.front_tyre, grip.front.total_n(), front_mass * lat_accel_wanted) -
      slip_angle(car_.rear_tyre, grip.rear.total_n(),
                 (car_.mass_kg - front_mass) * lat_accel_wanted);

  // Pedais. A aceleracao pedida e a que acompanha o limite de velocidade (a derivada dele no
  // tempo, do jeito que o carro o percorre) mais a que tira o erro de velocidade em
  // speed_response_s. Abaixo do limite o carro se aproxima dele por baixo.
  const double limit = speed_limit_mps(position_.s_m);
  const double limit_slope =
      (speed_limit_mps(position_.s_m + kSlopeStep) - limit) / kSlopeStep;  // d(limite)/ds
  const double force =
      car_.mass_kg * (speed * limit_slope + (limit - speed) / params_.speed_response_s) +
      resistance_n(car_, speed);
  const double lat_accel = std::abs(state.accel_lat_mps2);

  Controls controls;
  controls.steer = std::clamp(steer_angle / car_.max_steer_angle_rad, -1.0, 1.0);
  if (force > 0.0) {
    const double rpm = std::max(state.engine_rpm, car_.clutch_launch_rpm);
    const double full = car_.engine.full_load_torque(rpm) *
                        car_.gearbox.overall_ratio(state.gear) * car_.driveline_efficiency /
                        car_.wheel_radius_m;
    if (full > 0.0) {
      controls.throttle = std::min(force, drive_force_limit(speed, lat_accel, full)) / full;
    }
  } else {
    controls.brake =
        std::min(-force, brake_force_limit(speed, lat_accel)) / car_.max_brake_force_n;
  }
  return controls;
}

double Driver::speed_limit_mps(double s_m) const noexcept {
  const double u = track_.wrap(s_m) * static_cast<double>(speed_limit_mps_.size() - 1) /
                   track_.length_m();
  const std::size_t i = std::min(static_cast<std::size_t>(u), speed_limit_mps_.size() - 2);
  return speed_limit_mps_[i] +
         (u - static_cast<double>(i)) * (speed_limit_mps_[i + 1] - speed_limit_mps_[i]);
}

void Driver::plan() {
  const std::vector<TrackPoint>& line = track_.points();
  const std::size_t count = line.size() - 1;
  const double spacing = track_.length_m() / static_cast<double>(count);
  const auto lat_accel = [&line](std::size_t i, double speed) {
    return speed * speed * std::abs(line[i].curvature_1pm);
  };

  // Limite de curva em cada ponto.
  speed_limit_mps_.resize(count + 1);
  for (std::size_t i = 0; i < count; ++i) {
    speed_limit_mps_[i] = corner_speed(std::abs(line[i].curvature_1pm));
  }
  // Frenagem, de tras para a frente: em cada ponto, nao mais rapido do que da para frear ate
  // o limite do ponto seguinte. Duas voltas, para o limite fechar em torno da largada.
  for (std::size_t n = 2 * count; n > 0; --n) {
    const std::size_t here = n % count;
    const std::size_t before = (n - 1) % count;
    const double v = speed_limit_mps_[here];
    const double decel =
        (brake_force_limit(v, lat_accel(here, v)) + resistance_n(car_, v)) / car_.mass_kg;
    speed_limit_mps_[before] =
        std::min(speed_limit_mps_[before], std::sqrt(v * v + 2.0 * decel * spacing));
  }
  speed_limit_mps_[count] = speed_limit_mps_[0];

  // Previsao da volta lancada: o mesmo limite, mais a aceleracao que motor e tracao permitem.
  std::vector<double> speed = speed_limit_mps_;
  for (std::size_t n = 0; n < 2 * count; ++n) {
    const std::size_t here = n % count;
    const std::size_t after = (n + 1) % count;
    const double v = speed[here];
    const double accel = (drive_force_limit(v, lat_accel(here, v), engine_force_n(car_, v)) -
                          resistance_n(car_, v)) /
                         car_.mass_kg;
    speed[after] =
        std::min(speed[after], std::sqrt(std::max(0.0, v * v + 2.0 * accel * spacing)));
  }
  speed[count] = speed[0];
  predicted_lap_time_s_ = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    predicted_lap_time_s_ += 2.0 * spacing / (speed[i] + speed[i + 1]);
  }
}

// Maior velocidade em que uma curva com esta curvatura (em modulo) cabe na aderencia.
double Driver::corner_speed(double curvature) const noexcept {
  const double front_mass = car_.mass_kg * car_.weight_distribution_front;
  return largest_fitting(kSpeedCap, [&](double speed) {
    const double lat_accel = speed * speed * curvature;
    const CarGrip grip = wheel_grip(car_, speed, 0.0, lat_accel);
    return front_mass * lat_accel <= params_.grip_usage * grip.front.total_n() &&
           (car_.mass_kg - front_mass) * lat_accel <= params_.grip_usage * grip.rear.total_n();
  });
}

// Maior forca de freio (N, soma das quatro rodas) que deixa cada uma dentro do circulo
// de atrito enquanto o carro faz lat_accel de aceleracao lateral. Frear muda a carga dos eixos
// e a carga muda o quanto da para frear, dai a bissecao.
double Driver::brake_force_limit(double speed, double lat_accel) const noexcept {
  const double front_mass = car_.mass_kg * car_.weight_distribution_front;
  const double rear_mass = car_.mass_kg - front_mass;
  const double resistance = resistance_n(car_, speed);
  return largest_fitting(car_.max_brake_force_n, [&](double force) {
    const CarGrip grip =
        wheel_grip(car_, speed, -(force + resistance) / car_.mass_kg, lat_accel);
    return 0.5 * car_.brake_bias_front * force <=
               wheel_reserve(grip.front, front_mass * lat_accel, params_.grip_usage) &&
           0.5 * (1.0 - car_.brake_bias_front) * force <=
               wheel_reserve(grip.rear, rear_mass * lat_accel, params_.grip_usage);
  });
}

// Maior forca de tracao (N) que o eixo traseiro transmite junto com a forca lateral, sem
// passar do que o motor entrega (engine_force).
double Driver::drive_force_limit(double speed, double lat_accel,
                                 double engine_force) const noexcept {
  const double rear_mass = car_.mass_kg * (1.0 - car_.weight_distribution_front);
  const double resistance = resistance_n(car_, speed);
  return largest_fitting(engine_force, [&](double force) {
    const CarGrip grip = wheel_grip(car_, speed, (force - resistance) / car_.mass_kg, lat_accel);
    return 0.5 * force <= wheel_reserve(grip.rear, rear_mass * lat_accel, params_.grip_usage);
  });
}

}  // namespace sim
