#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "sim/vehicle.hpp"

using sim::Controls;
using sim::kFrontLeft;
using sim::kFrontRight;
using sim::kRearLeft;
using sim::kRearRight;
using sim::Vehicle;
using sim::VehicleParams;
using sim::VehicleState;

namespace {

constexpr double kG = 9.80665;

struct Vec2 {
  double x;
  double y;
};

Vec2 position(const Vehicle& car) { return {car.state().x_m, car.state().y_m}; }

// Velocidade do CG nos eixos do mundo.
Vec2 world_velocity(const VehicleState& s) {
  return {s.vx_mps * std::cos(s.yaw_rad) - s.vy_mps * std::sin(s.yaw_rad),
          s.vx_mps * std::sin(s.yaw_rad) + s.vy_mps * std::cos(s.yaw_rad)};
}

// Raio do circulo que passa por tres pontos da trajetoria: medida puramente geometrica.
double circumradius(Vec2 a, Vec2 b, Vec2 c) {
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(a.x - c.x, a.y - c.y);
  const double twice_area = std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
  return ab * bc * ca / (2.0 * twice_area);
}

// O que os quatro pneus conseguem transmitir agora, por kg de carro (m/s^2).
double grip_limit(const Vehicle& car) {
  const VehicleParams& p = car.params();
  const auto& load = car.state().wheel_load_n;
  return (p.front_tyre.peak_force(load[kFrontLeft]) + p.front_tyre.peak_force(load[kFrontRight]) +
          p.rear_tyre.peak_force(load[kRearLeft]) + p.rear_tyre.peak_force(load[kRearRight])) /
         p.mass_kg;
}

// Piloto de skidpad (curva a esquerda): mantem o raio pela taxa de guinada, com esterco de
// Ackermann mais um PI no erro em relacao a v/R, e a velocidade por um PI suave no acelerador.
class SkidpadDriver {
 public:
  SkidpadDriver(const VehicleParams& p, double radius_m)
      : radius_m_(radius_m),
        ackermann_rad_(p.wheelbase_m / radius_m),
        max_steer_rad_(p.max_steer_angle_rad) {}

  // target_speed = 0 deixa o carro sem acelerador.
  Controls operator()(const VehicleState& s, double target_speed) {
    const double speed_error = target_speed - s.speed_mps;
    speed_integral_ += speed_error * Vehicle::kDt;
    const double yaw_error = s.speed_mps / radius_m_ - s.yaw_rate_radps;
    yaw_integral_ += yaw_error * Vehicle::kDt;
    const double steer_rad = ackermann_rad_ + 0.1 * yaw_error + 1.0 * yaw_integral_;
    return {std::clamp(0.05 * speed_error + 0.05 * speed_integral_, 0.0, 1.0), 0.0,
            std::clamp(steer_rad / max_steer_rad_, -1.0, 1.0)};
  }

 private:
  double radius_m_;
  double ackermann_rad_;
  double max_steer_rad_;
  double speed_integral_ = 0.0;
  double yaw_integral_ = 0.0;
};

struct SkidpadLimit {
  double lat_accel = 0.0;  // maior aceleracao lateral atingida (m/s^2)
  double speed = 0.0;      // velocidade nesse instante (m/s)
  double grip = 0.0;       // grip_limit() nesse instante (m/s^2)
};

// Metodo do raio constante: acelera devagar no circulo ate o carro nao segurar mais o raio.
SkidpadLimit find_limit(const VehicleParams& p, double radius_m, double start_speed) {
  Vehicle car{p, start_speed};
  SkidpadDriver driver{p, radius_m};
  SkidpadLimit limit;
  for (int i = 0; i < 150000; ++i) {
    const double t = i * Vehicle::kDt;
    car.step(driver(car.state(), start_speed + 0.5 * t));  // 0,5 m/s^2: quase-estatico
    const VehicleState& s = car.state();
    if (s.accel_lat_mps2 > limit.lat_accel) {
      limit = {s.accel_lat_mps2, s.speed_mps, grip_limit(car)};
    }
    // Raio real (v / taxa de guinada) 10% acima do pedido: passou do limite.
    if (t > 2.0 && s.speed_mps > 1.1 * radius_m * s.yaw_rate_radps) break;
  }
  return limit;
}

constexpr double kRadius = 40.0;  // m; o limite fica perto de 105 km/h, longe das trocas de marcha

const SkidpadLimit& limit_at_40m() {
  static const SkidpadLimit limit = find_limit(VehicleParams{}, kRadius, 15.0);
  return limit;
}

}  // namespace

TEST(Handling, StraightLineStaysStraight) {
  Vehicle car{VehicleParams{}};
  for (int i = 0; i < 20000; ++i) car.step({1.0, 0.0, 0.0});
  const VehicleState& s = car.state();
  EXPECT_DOUBLE_EQ(s.y_m, 0.0);
  EXPECT_DOUBLE_EQ(s.yaw_rad, 0.0);
  EXPECT_DOUBLE_EQ(s.vy_mps, 0.0);
  EXPECT_DOUBLE_EQ(s.x_m, s.position_m);
  EXPECT_DOUBLE_EQ(s.wheel_load_n[kFrontLeft], s.wheel_load_n[kFrontRight]);
  EXPECT_DOUBLE_EQ(s.wheel_load_n[kRearLeft], s.wheel_load_n[kRearRight]);
}

TEST(Handling, SteeringAtStandstillDoesNotMoveTheCar) {
  const VehicleParams p;
  Vehicle car{p};
  for (int i = 0; i < 1000; ++i) car.step({0.0, 0.0, 1.0});
  EXPECT_DOUBLE_EQ(car.state().x_m, 0.0);
  EXPECT_DOUBLE_EQ(car.state().y_m, 0.0);
  EXPECT_DOUBLE_EQ(car.state().yaw_rad, 0.0);
  EXPECT_DOUBLE_EQ(car.state().steer_angle_rad, p.max_steer_angle_rad);
}

TEST(Handling, LowSpeedTurnFollowsAckermannGeometry) {
  const VehicleParams p;
  Vehicle car{p, 6.0};
  const double steer = 0.3;
  std::array<Vec2, 3> sample{};
  for (int i = 1; i <= 4000; ++i) {
    car.step({0.0, 0.0, steer});
    if (i % 1000 == 0 && i >= 2000) sample[static_cast<std::size_t>(i / 1000 - 2)] = position(car);
  }
  // Devagar, o eixo traseiro descreve o raio L / tan(delta); o CG fica um pouco por fora.
  const double rear_radius = p.wheelbase_m / std::tan(steer * p.max_steer_angle_rad);
  const double cg_to_rear = p.wheelbase_m * p.weight_distribution_front;
  const double expected = std::hypot(rear_radius, cg_to_rear);
  EXPECT_NEAR(circumradius(sample[0], sample[1], sample[2]), expected, 0.02 * expected);
  EXPECT_GT(car.state().yaw_rad, 0.0);  // esterco positivo = curva a esquerda
  EXPECT_GT(car.state().y_m, 0.0);
}

TEST(Handling, LeftAndRightTurnsAreMirrorImages) {
  Vehicle left{VehicleParams{}, 40.0};
  Vehicle right{VehicleParams{}, 40.0};
  for (int i = 0; i < 3000; ++i) {
    left.step({0.2, 0.0, 0.1});
    right.step({0.2, 0.0, -0.1});
  }
  const VehicleState& l = left.state();
  const VehicleState& r = right.state();
  EXPECT_GT(l.y_m, 1.0);
  EXPECT_NEAR(l.x_m, r.x_m, 1e-6);
  EXPECT_NEAR(l.y_m, -r.y_m, 1e-6);
  EXPECT_NEAR(l.yaw_rad, -r.yaw_rad, 1e-9);
  EXPECT_NEAR(l.accel_lat_mps2, -r.accel_lat_mps2, 1e-6);
  EXPECT_NEAR(l.wheel_load_n[kFrontLeft], r.wheel_load_n[kFrontRight], 1e-6);
  EXPECT_NEAR(l.wheel_load_n[kRearRight], r.wheel_load_n[kRearLeft], 1e-6);
}

TEST(Handling, WheelLoadsAlwaysSumToWeightPlusDownforce) {
  const VehicleParams p;
  Vehicle car{p, 50.0};
  for (int i = 0; i < 6000; ++i) {
    // Acelera, vira e freia em fases defasadas.
    car.step({i < 2000 ? 1.0 : 0.0, i >= 4000 ? 0.7 : 0.0, i >= 1000 ? 0.15 : 0.0});
    const VehicleState& s = car.state();
    double total = 0.0;
    for (const double load : s.wheel_load_n) {
      ASSERT_GE(load, 0.0);
      total += load;
    }
    ASSERT_NEAR(total, p.mass_kg * kG + s.downforce_n, 1e-6);
  }
}

TEST(Handling, BrakingMovesLoadToTheFrontAndAcceleratingToTheRear) {
  const VehicleParams p;
  const auto front_axle = [](const VehicleState& s) {
    return s.wheel_load_n[kFrontLeft] + s.wheel_load_n[kFrontRight];
  };
  // Carga dianteira prevista pelo momento de arfagem m*ax*h/L.
  const auto expected_front = [&p](const VehicleState& s) {
    return p.mass_kg * kG * p.weight_distribution_front + s.downforce_n * p.aero_balance_front -
           p.mass_kg * s.accel_long_mps2 * p.cg_height_m / p.wheelbase_m;
  };

  Vehicle braking{p, 60.0};
  for (int i = 0; i < 500; ++i) braking.step({0.0, 0.6, 0.0});
  ASSERT_LT(braking.state().accel_long_mps2, -2.0 * kG);
  EXPECT_NEAR(front_axle(braking.state()), expected_front(braking.state()),
              0.01 * expected_front(braking.state()));

  Vehicle launching{p};
  for (int i = 0; i < 500; ++i) launching.step({1.0, 0.0, 0.0});
  ASSERT_GT(launching.state().accel_long_mps2, 1.0 * kG);
  EXPECT_NEAR(front_axle(launching.state()), expected_front(launching.state()),
              0.01 * expected_front(launching.state()));
  EXPECT_LT(front_axle(launching.state()), p.mass_kg * kG * p.weight_distribution_front);
}

TEST(Handling, CorneringMovesLoadToTheOuterWheels) {
  const VehicleParams p;
  Vehicle car{p, 40.0};
  for (int i = 0; i < 3000; ++i) car.step({0.15, 0.0, 0.1});  // curva a esquerda em regime
  const VehicleState& s = car.state();
  ASSERT_GT(s.accel_lat_mps2, 1.0 * kG);
  const auto& load = s.wheel_load_n;
  const double outer_minus_inner =
      (load[kFrontRight] + load[kRearRight]) - (load[kFrontLeft] + load[kRearLeft]);
  const double expected = 2.0 * p.mass_kg * s.accel_lat_mps2 * p.cg_height_m / p.track_width_m;
  EXPECT_NEAR(outer_minus_inner, expected, 0.01 * expected);
  // A dianteira absorve a fracao dada pela rigidez de rolagem.
  EXPECT_NEAR((load[kFrontRight] - load[kFrontLeft]) / outer_minus_inner, p.roll_stiffness_front,
              1e-6);
}

TEST(Handling, TyreForcesStayInsideTheFrictionCircle) {
  const VehicleParams p;
  Vehicle car{p, 70.0};
  for (int i = 0; i < 4000; ++i) {
    // Virando: potencia total e depois freio total.
    car.step({i < 1500 ? 1.0 : 0.0, i >= 1500 ? 1.0 : 0.0, 0.3});
    const VehicleState& s = car.state();
    for (std::size_t w = 0; w < 4; ++w) {
      const sim::Tyre& tyre = w < kRearLeft ? p.front_tyre : p.rear_tyre;
      ASSERT_LE(std::hypot(s.wheel_fx_n[w], s.wheel_fy_n[w]),
                tyre.peak_force(s.wheel_load_n[w]) + 1e-6);
    }
  }
}

TEST(Handling, HardBrakingInACornerCostsLateralGrip) {
  Vehicle car{VehicleParams{}, 45.0};
  for (int i = 0; i < 1500; ++i) car.step({0.2, 0.0, 0.3});  // curva no limite
  const double cornering = car.state().accel_lat_mps2;
  ASSERT_GT(cornering, 2.0 * kG);
  for (int i = 0; i < 100; ++i) car.step({0.0, 1.0, 0.3});  // 0,1 s de freio total
  EXPECT_LT(car.state().accel_lat_mps2, 0.8 * cornering);
  EXPECT_LT(car.state().accel_long_mps2, -1.5 * kG);
}

TEST(Handling, FixedRadiusCornerReachesTheGripLimit) {
  const SkidpadLimit& limit = limit_at_40m();
  // Nao da para passar do que os pneus transmitem...
  EXPECT_LE(limit.lat_accel, limit.grip);
  // ...e o chassi aproveita quase tudo: a dianteira satura um pouco antes da traseira.
  EXPECT_GT(limit.lat_accel, 0.85 * limit.grip);
  // Plausibilidade: curva de ~105 km/h; um F1 faz algo entre 2 e 3 g.
  EXPECT_GT(limit.lat_accel, 1.8 * kG);
  EXPECT_LT(limit.lat_accel, 3.0 * kG);
}

TEST(Handling, HoldsTheFixedRadiusJustBelowTheLimit) {
  const VehicleParams p;
  const double target = 0.95 * limit_at_40m().speed;
  Vehicle car{p, 15.0};
  SkidpadDriver driver{p, kRadius};
  std::array<Vec2, 3> sample{};
  for (int i = 1; i <= 30000; ++i) {
    car.step(driver(car.state(), std::min(15.0 + 1.0 * i * Vehicle::kDt, target)));
    if (i == 27000) sample[0] = position(car);
    if (i == 28500) sample[1] = position(car);
    if (i == 30000) sample[2] = position(car);
  }
  const VehicleState& s = car.state();
  EXPECT_NEAR(circumradius(sample[0], sample[1], sample[2]), kRadius, 0.01 * kRadius);
  EXPECT_NEAR(s.speed_mps, target, 0.1);
  EXPECT_NEAR(s.accel_lat_mps2, target * target / kRadius, 0.03 * target * target / kRadius);
  EXPECT_GT(s.accel_lat_mps2, 0.85 * limit_at_40m().lat_accel);
}

TEST(Handling, AboveTheLimitSpeedTheCarRunsWide) {
  const VehicleParams p;
  Vehicle car{p, 1.15 * limit_at_40m().speed};
  SkidpadDriver driver{p, kRadius};
  std::array<Vec2, 3> sample{};
  for (int i = 1; i <= 900; ++i) {
    car.step(driver(car.state(), 0.0));  // sem acelerador: so tenta fazer a curva
    if (i % 300 == 0) sample[static_cast<std::size_t>(i / 300 - 1)] = position(car);
    // Sai de frente (subesterco), sem rodar.
    ASSERT_LT(std::abs(car.state().vy_mps), 0.3 * car.state().vx_mps);
  }
  EXPECT_GT(circumradius(sample[0], sample[1], sample[2]), 1.1 * kRadius);
  EXPECT_GT(car.state().yaw_rate_radps, 0.0);
}

TEST(Handling, DownforceRaisesTheCorneringLimitWithSpeed) {
  const SkidpadLimit fast = find_limit(VehicleParams{}, 80.0, 25.0);
  EXPECT_GT(fast.speed, 1.3 * limit_at_40m().speed);
  EXPECT_GT(fast.lat_accel, 1.2 * limit_at_40m().lat_accel);
}

TEST(Handling, LowerCentreOfGravityCornersHarder) {
  VehicleParams low;
  low.cg_height_m = 0.05;
  VehicleParams high;
  high.cg_height_m = 0.40;
  // Menos transferencia lateral de carga = pneus mais iguais = mais aderencia total.
  EXPECT_GT(find_limit(low, kRadius, 15.0).lat_accel,
            1.02 * find_limit(high, kRadius, 15.0).lat_accel);
}

// Potencia total com esterco total: a traseira gasta a aderencia tracionando, perde a forca
// lateral e o carro roda. Solto, ele segue de re ate o arrasto e o rolamento o pararem.
TEST(Handling, SpunCarSlidesBackwardsObeyingNewtonAndCoastsToRest) {
  Vehicle car{VehicleParams{}, 40.0};
  double min_vx = 0.0;
  for (int i = 0; i < 6000; ++i) {
    const Vec2 before = world_velocity(car.state());
    const double cos_yaw = std::cos(car.state().yaw_rad);
    const double sin_yaw = std::sin(car.state().yaw_rad);
    car.step(i < 2000 ? Controls{1.0, 0.0, 1.0} : Controls{});  // roda 2 s, depois desliza solto
    const VehicleState& s = car.state();
    const Vec2 after = world_velocity(s);
    min_vx = std::min(min_vx, s.vx_mps);
    // No mundo, a velocidade so muda pela forca dos pneus e do ar: e a aceleracao informada,
    // girada pelo rumo que o carro tinha no inicio do passo.
    ASSERT_NEAR((after.x - before.x) / Vehicle::kDt,
                cos_yaw * s.accel_long_mps2 - sin_yaw * s.accel_lat_mps2, 1e-6);
    ASSERT_NEAR((after.y - before.y) / Vehicle::kDt,
                sin_yaw * s.accel_long_mps2 + cos_yaw * s.accel_lat_mps2, 1e-6);
  }
  EXPECT_LT(min_vx, -20.0);  // passou de re: a quantidade de movimento nao some no piao
  for (int i = 0; i < 150000; ++i) car.step({});
  EXPECT_DOUBLE_EQ(car.state().speed_mps, 0.0);
}

TEST(Handling, FullThrottleAtFullLockStaysBounded) {
  Vehicle car{VehicleParams{}, 30.0};
  for (int i = 0; i < 8000; ++i) {
    car.step({1.0, 0.0, 1.0});
    const VehicleState& s = car.state();
    ASSERT_TRUE(std::isfinite(s.x_m) && std::isfinite(s.y_m) && std::isfinite(s.yaw_rad));
    ASSERT_LT(s.speed_mps, 120.0);
    ASSERT_LT(std::abs(s.yaw_rate_radps), 15.0);
  }
}

TEST(Handling, BrakesBringASpinningCarToACompleteRest) {
  Vehicle car{VehicleParams{}, 40.0};
  for (int i = 0; i < 2000; ++i) car.step({1.0, 0.0, 1.0});  // potencia total com esterco total
  for (int i = 0; i < 10000; ++i) car.step({0.0, 1.0, 0.0});
  // Repouso de verdade: o freio nao deixa o carro tremendo em torno do zero.
  EXPECT_DOUBLE_EQ(car.state().speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(car.state().yaw_rate_radps, 0.0);
}
