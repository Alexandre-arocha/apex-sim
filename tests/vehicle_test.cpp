#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "sim/vehicle.hpp"

using sim::Controls;
using sim::Vehicle;
using sim::VehicleParams;

namespace {

constexpr double kG = 9.80665;
constexpr double kFullThrottle = 1.0;

double kmh(double mps) { return mps * 3.6; }

// Acelera a fundo ate atingir target_mps; devolve o tempo ou -1 no timeout.
double time_to_speed(Vehicle& car, double target_mps, double timeout_s) {
  while (car.state().time_s < timeout_s) {
    car.step({kFullThrottle, 0.0});
    if (car.state().speed_mps >= target_mps) return car.state().time_s;
  }
  return -1.0;
}

}  // namespace

TEST(Vehicle, FixedStepAdvancesExactlyOneMillisecondPerStep) {
  Vehicle car{VehicleParams{}};
  for (int i = 0; i < 2500; ++i) car.step({});
  EXPECT_EQ(car.state().step_count, 2500u);
  EXPECT_NEAR(car.state().time_s, 2.5, 1e-9);
  EXPECT_DOUBLE_EQ(Vehicle::kDt, 0.001);
}

TEST(Vehicle, IsDeterministic) {
  Vehicle a{VehicleParams{}}, b{VehicleParams{}};
  for (int i = 0; i < 20000; ++i) {
    const Controls c{i < 12000 ? 1.0 : 0.0, i >= 15000 ? 0.6 : 0.0};
    a.step(c);
    b.step(c);
  }
  EXPECT_EQ(a.state().speed_mps, b.state().speed_mps);
  EXPECT_EQ(a.state().position_m, b.state().position_m);
  EXPECT_EQ(a.state().gear, b.state().gear);
}

TEST(Vehicle, ZeroTo100KmhIsPlausibleForAnF1Car) {
  Vehicle car{VehicleParams{}};
  const double t = time_to_speed(car, 100.0 / 3.6, 10.0);
  ASSERT_GT(t, 0.0) << "nao chegou a 100 km/h";
  EXPECT_GT(t, 2.0);
  EXPECT_LT(t, 3.5);
}

TEST(Vehicle, LaunchAccelerationIsTractionLimited) {
  const VehicleParams p;
  Vehicle car{p};
  double peak = 0;
  while (car.state().speed_mps < 100.0 / 3.6) {
    car.step({kFullThrottle, 0.0});
    const auto& s = car.state();
    const double rear_grip = p.rear_tyre.peak_force(s.wheel_load_n[sim::kRearLeft]) +
                             p.rear_tyre.peak_force(s.wheel_load_n[sim::kRearRight]);
    EXPECT_LE(s.drive_force_n, rear_grip + 1e-6);
    if (s.time_s < 0.5) {  // em primeira o motor sobra: quem limita e o pneu
      EXPECT_NEAR(s.drive_force_n, rear_grip, 1e-6);
    }
    peak = std::max(peak, s.accel_long_mps2);
  }
  EXPECT_LT(peak, 2.0 * kG);  // nenhum carro de F1 passa de ~2 g na largada
}

TEST(Vehicle, TopSpeedIsPlausibleAndStable) {
  const VehicleParams p;
  Vehicle car{p};
  for (int i = 0; i < 140000; ++i) car.step({kFullThrottle, 0.0});
  const double v140 = car.state().speed_mps;
  for (int i = 0; i < 10000; ++i) {
    car.step({kFullThrottle, 0.0});
    EXPECT_LE(car.state().engine_rpm, p.engine.redline_rpm());
  }
  EXPECT_GT(kmh(v140), 320.0);
  EXPECT_LT(kmh(v140), 380.0);
  EXPECT_NEAR(car.state().speed_mps, v140, 0.5);  // terminal: aceleracao ~ 0
  EXPECT_EQ(car.state().gear, p.gearbox.gear_count());
}

TEST(Vehicle, NewtonSecondLawHoldsEveryStep) {
  const VehicleParams p;
  Vehicle car{p};
  for (int i = 0; i < 30000; ++i) {
    car.step({kFullThrottle, i > 20000 ? 0.4 : 0.0});
    const auto& s = car.state();
    const double net = s.drive_force_n - s.brake_force_n - s.drag_force_n - s.rolling_force_n;
    EXPECT_NEAR(s.accel_long_mps2, net / p.mass_kg, 1e-6);
  }
}

TEST(Vehicle, GearsOnlyGoUpUnderFullThrottleAndTorqueIsCutWhileShifting) {
  Vehicle car{VehicleParams{}};
  int last_gear = car.state().gear;
  int shifts_seen = 0;
  for (int i = 0; i < 40000; ++i) {
    car.step({kFullThrottle, 0.0});
    const auto& s = car.state();
    EXPECT_GE(s.gear, last_gear);
    if (s.shifting) {
      EXPECT_DOUBLE_EQ(s.drive_force_n, 0.0);
    }
    if (s.gear != last_gear) ++shifts_seen;
    last_gear = s.gear;
  }
  EXPECT_EQ(shifts_seen, 7);  // 1 -> 8
}

TEST(Vehicle, FullBrakeStopsWithPlausibleDecelerationAndNeverReverses) {
  const VehicleParams p;
  Vehicle car{p, 100.0};  // 360 km/h
  double peak_decel = 0;
  int steps = 0;
  while (car.state().speed_mps > 0.0 && steps++ < 30000) {
    car.step({0.0, 1.0});
    peak_decel = std::max(peak_decel, -car.state().accel_long_mps2);
    ASSERT_GE(car.state().speed_mps, 0.0);
  }
  EXPECT_DOUBLE_EQ(car.state().speed_mps, 0.0);
  EXPECT_GT(peak_decel, 3.0 * kG);
  EXPECT_LT(peak_decel, 6.5 * kG);
  EXPECT_LT(car.state().time_s, 10.0);
  EXPECT_LT(car.state().position_m, 1000.0);
}

TEST(Vehicle, CoastingSlowsDownFromDragAndRollingOnly) {
  Vehicle car{VehicleParams{}, 80.0};
  double last = car.state().speed_mps;
  for (int i = 0; i < 5000; ++i) {
    car.step({});
    EXPECT_LE(car.state().speed_mps, last);
    EXPECT_DOUBLE_EQ(car.state().drive_force_n, 0.0);
    last = car.state().speed_mps;
  }
  EXPECT_LT(last, 80.0);
  EXPECT_GT(last, 0.0);
}

TEST(Vehicle, StandstillStaysPutWithoutInput) {
  Vehicle car{VehicleParams{}};
  for (int i = 0; i < 1000; ++i) car.step({});
  EXPECT_DOUBLE_EQ(car.state().speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(car.state().position_m, 0.0);
}

TEST(Vehicle, InvalidControlsAreSanitized) {
  Vehicle car{VehicleParams{}};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  car.step({nan, nan});
  EXPECT_DOUBLE_EQ(car.state().drive_force_n, 0.0);
  EXPECT_DOUBLE_EQ(car.state().brake_force_n, 0.0);
  car.step({50.0, -3.0});  // saturados em 1 e 0
  EXPECT_GT(car.state().drive_force_n, 0.0);
  EXPECT_DOUBLE_EQ(car.state().brake_force_n, 0.0);
}

TEST(Vehicle, InitialSpeedPicksSensibleGear) {
  EXPECT_EQ(Vehicle(VehicleParams{}, 0.0).state().gear, 1);
  const Vehicle fast(VehicleParams{}, 90.0);
  EXPECT_GT(fast.state().gear, 5);
  EXPECT_DOUBLE_EQ(Vehicle(VehicleParams{}, -5.0).state().speed_mps, 0.0);
}

TEST(Vehicle, HonoursAGearRequestAndStaysAutomaticWithoutOne) {
  Vehicle car{VehicleParams{}};
  for (int i = 0; i < 100; ++i) car.step({0.0, 0.0, 0.0, 3});
  EXPECT_EQ(car.state().gear, 3);
  for (int i = 0; i < 100; ++i) car.step({0.0, 0.0, 0.0, 99});  // satura na ultima marcha
  EXPECT_EQ(car.state().gear, car.params().gearbox.gear_count());
  Vehicle automatic{VehicleParams{}};
  for (int i = 0; i < 100; ++i) automatic.step({1.0, 0.0, 0.0, 0});
  EXPECT_EQ(automatic.state().gear, 1);  // sem pedido, o cambio automatico segue em 1a
}

TEST(Vehicle, RejectsInvalidParams) {
  VehicleParams p;
  p.mass_kg = 0;
  EXPECT_THROW(Vehicle{p}, std::invalid_argument);
  p = VehicleParams{};
  p.weight_distribution_front = 1.5;
  EXPECT_THROW(Vehicle{p}, std::invalid_argument);
  p = VehicleParams{};
  p.upshift_rpm = 16000;  // acima da zona vermelha
  EXPECT_THROW(Vehicle{p}, std::invalid_argument);
  p = VehicleParams{};
  p.yaw_inertia_kgm2 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Vehicle{p}, std::invalid_argument);
  p = VehicleParams{};
  p.front_tyre = sim::Tyre({.stiffness_b = 500.0});  // rigido demais para o passo de 1 ms
  EXPECT_THROW(Vehicle{p}, std::invalid_argument);
}
