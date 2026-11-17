#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "sim/driver.hpp"
#include "sim/track.hpp"
#include "sim/vehicle.hpp"

using sim::Controls;
using sim::CurvaturePoint;
using sim::Driver;
using sim::DriverParams;
using sim::LapTimer;
using sim::Track;
using sim::TrackPoint;
using sim::Vehicle;
using sim::VehicleParams;
using sim::VehicleState;

namespace {

constexpr double kG = 9.80665;

// O que aconteceu numa sequencia de voltas.
struct LapRun {
  std::vector<double> lap_times_s;
  double predicted_lap_time_s = 0.0;
  double max_offset_m = 0.0;          // maior distancia a linha
  double max_speed_excess_mps = 0.0;  // maior velocidade acima do limite do piloto no ponto
  double top_speed_mps = 0.0;
  double max_lat_accel_mps2 = 0.0;
  double max_braking_mps2 = 0.0;
};

// Poe o piloto para dar `laps` voltas a partir da largada parada, ou ate acabar o tempo.
LapRun run_laps(const Track& track, const VehicleParams& car, const DriverParams& params, int laps,
             double time_limit_s = 400.0) {
  Vehicle vehicle{car};
  Driver driver{track, car, params};
  LapTimer timer{track.length_m()};
  LapRun run;
  run.predicted_lap_time_s = driver.predicted_lap_time_s();
  while (timer.lap_times_s().size() < static_cast<std::size_t>(laps) &&
         vehicle.state().time_s < time_limit_s) {
    const VehicleState& state = vehicle.state();
    const Controls controls = driver.drive(state);
    const double s = driver.position().s_m;
    timer.update(state.time_s, s);
    run.max_offset_m = std::max(run.max_offset_m, std::abs(driver.position().lateral_m));
    run.max_speed_excess_mps =
        std::max(run.max_speed_excess_mps, state.speed_mps - driver.speed_limit_mps(s));
    run.top_speed_mps = std::max(run.top_speed_mps, state.speed_mps);
    run.max_lat_accel_mps2 = std::max(run.max_lat_accel_mps2, std::abs(state.accel_lat_mps2));
    run.max_braking_mps2 = std::max(run.max_braking_mps2, -state.accel_long_mps2);
    vehicle.step(controls);
  }
  run.lap_times_s = timer.lap_times_s();
  return run;
}

// Tres voltas no circuito de referencia com o carro e o piloto padrao.
const LapRun& reference_run() {
  static const LapRun run = run_laps(Track::reference(), VehicleParams{}, DriverParams{}, 3);
  return run;
}

// Oval: duas retas de 300 m e duas curvas de 180 graus com 60 m de raio, com entrada e saida
// em clotoide. direction = +1 anti-horario, -1 horario.
Track oval(double direction) {
  const double radius = 60.0;
  const double ramp = 40.0;
  std::vector<CurvaturePoint> profile{{0.0, 0.0}};
  for (int half = 0; half < 2; ++half) {
    double s = profile.back().s_m;
    profile.push_back({s += 300.0, 0.0});
    profile.push_back({s += ramp, direction / radius});
    profile.push_back({s += std::numbers::pi * radius - ramp, direction / radius});
    profile.push_back({s += ramp, 0.0});
  }
  return Track(profile);
}

}  // namespace

TEST(Driver, SpeedLimitFollowsTheCorners) {
  const Track track = Track::reference();
  const Driver driver{track, VehicleParams{}};
  double slowest = std::numeric_limits<double>::infinity();
  double slowest_at = 0.0;
  for (const TrackPoint& p : track.points()) {
    const double limit = driver.speed_limit_mps(p.s_m);
    if (limit < slowest) {
      slowest = limit;
      slowest_at = p.s_m;
    }
  }
  // O ponto mais lento fica na curva mais fechada (40 m de raio), perto de 100 km/h.
  EXPECT_NEAR(std::abs(track.at(slowest_at).curvature_1pm), 1.0 / 40.0, 1e-9);
  EXPECT_GT(slowest * 3.6, 90.0);
  EXPECT_LT(slowest * 3.6, 110.0);
  // No meio da reta o limite e so um teto: ali quem segura o carro e o motor.
  EXPECT_GT(driver.speed_limit_mps(0.0) * 3.6, 400.0);
}

// Guarda de regressao do limite estatico de aderencia do carro de referencia numa curva de 40 m
// (2,17 g); que a estimativa vale na pratica quem confere sao os testes de volta abaixo, em que o
// piloto usa 90% dela e fica a menos de 1 m da linha.
TEST(Driver, GripEstimateMatchesTheReferenceCorneringLimit) {
  const double radius = 40.0;
  const Track skidpad(std::vector<CurvaturePoint>{
      {0.0, 1.0 / radius}, {2.0 * std::numbers::pi * radius, 1.0 / radius}});
  DriverParams all_the_grip;
  all_the_grip.grip_usage = 1.0;
  const Driver driver{skidpad, VehicleParams{}, all_the_grip};
  const double speed = driver.speed_limit_mps(0.0);
  EXPECT_NEAR(speed * speed / radius, 2.17 * kG, 0.15 * kG);
}

TEST(Driver, CompletesConsistentLaps) {
  const LapRun& run = reference_run();
  ASSERT_EQ(run.lap_times_s.size(), 3u);
  EXPECT_GT(run.lap_times_s[1], 40.0);  // 2,2 km a uma media perto de 180 km/h
  EXPECT_LT(run.lap_times_s[1], 50.0);
  EXPECT_GT(run.lap_times_s[0], run.lap_times_s[1] + 1.0);     // a primeira larga parada
  EXPECT_NEAR(run.lap_times_s[2], run.lap_times_s[1], 0.01);  // voltas lancadas iguais
}

TEST(Driver, StaysOnTheLineAndWithinItsSpeedLimit) {
  const LapRun& run = reference_run();
  EXPECT_LT(run.max_offset_m, 1.0);
  EXPECT_LT(run.max_speed_excess_mps, 0.5);
}

TEST(Driver, DrivesTheCarNearItsLimits) {
  const LapRun& run = reference_run();
  ASSERT_EQ(run.lap_times_s.size(), 3u);
  EXPECT_GT(run.top_speed_mps * 3.6, 280.0);
  EXPECT_GT(run.max_lat_accel_mps2, 1.8 * kG);
  EXPECT_LT(run.max_lat_accel_mps2, 3.5 * kG);
  EXPECT_GT(run.max_braking_mps2, 3.8 * kG);
  EXPECT_LT(run.max_braking_mps2, 6.5 * kG);
  // A volta simulada sai perto da prevista com o carro tratado como um ponto.
  EXPECT_NEAR(run.lap_times_s[1], run.predicted_lap_time_s, 0.02 * run.predicted_lap_time_s);
}

TEST(Driver, IsDeterministic) {
  const LapRun again = run_laps(Track::reference(), VehicleParams{}, DriverParams{}, 2);
  ASSERT_EQ(again.lap_times_s.size(), 2u);
  EXPECT_EQ(again.lap_times_s[0], reference_run().lap_times_s[0]);
  EXPECT_EQ(again.lap_times_s[1], reference_run().lap_times_s[1]);
}

TEST(Driver, CautiousDriverIsSlower) {
  DriverParams cautious;
  cautious.grip_usage = 0.6;
  const LapRun run = run_laps(Track::reference(), VehicleParams{}, cautious, 1);
  ASSERT_EQ(run.lap_times_s.size(), 1u);
  EXPECT_GT(run.lap_times_s[0], reference_run().lap_times_s[0] + 2.0);
  EXPECT_LT(run.max_lat_accel_mps2, 0.8 * reference_run().max_lat_accel_mps2);
  EXPECT_LT(run.max_offset_m, 0.5);
}

// Controle negativo: pedindo mais do que os pneus dao, o carro nao segura a linha.
TEST(Driver, AskingForMoreGripThanTheTyresHaveRunsWide) {
  DriverParams bold;
  bold.grip_usage = 1.3;
  const LapRun run = run_laps(Track::reference(), VehicleParams{}, bold, 1, 30.0);
  EXPECT_GT(run.max_offset_m, 3.0);
}

TEST(Driver, AdaptsToTheCar) {
  VehicleParams low_downforce;
  low_downforce.downforce_area_m2 = 2.0;
  const LapRun run = run_laps(Track::reference(), low_downforce, DriverParams{}, 1);
  ASSERT_EQ(run.lap_times_s.size(), 1u);
  EXPECT_GT(run.lap_times_s[0], reference_run().lap_times_s[0] + 0.5);
  EXPECT_LT(run.max_offset_m, 1.0);  // com menos downforce segue a linha um pouco pior
  EXPECT_LT(run.max_speed_excess_mps, 1.0);
}

TEST(Driver, ClockwiseAndAnticlockwiseLapsAreMirrorImages) {
  const LapRun left = run_laps(oval(1.0), VehicleParams{}, DriverParams{}, 1);
  const LapRun right = run_laps(oval(-1.0), VehicleParams{}, DriverParams{}, 1);
  ASSERT_EQ(left.lap_times_s.size(), 1u);
  ASSERT_EQ(right.lap_times_s.size(), 1u);
  EXPECT_NEAR(left.lap_times_s[0], right.lap_times_s[0], 1e-6);
  EXPECT_NEAR(left.max_offset_m, right.max_offset_m, 1e-6);
  EXPECT_LT(left.max_offset_m, 1.0);
}

TEST(Driver, ReturnsToTheLineAfterBeingPushedOffIt) {
  const VehicleParams car;
  Vehicle vehicle{car, 30.0};
  Driver driver{Track::reference(), car};
  for (int i = 0; i < 500; ++i) {
    vehicle.step({0.3, 0.0, 0.15});  // meio segundo virando a esquerda, sem piloto
  }
  double worst = 0.0;    // maior desvio depois que o piloto assume
  double settled = 0.0;  // maior desvio no ultimo meio segundo
  for (int i = 0; i < 3000; ++i) {
    vehicle.step(driver.drive(vehicle.state()));
    const double offset = std::abs(driver.position().lateral_m);
    worst = std::max(worst, offset);
    if (i >= 2500) {
      settled = std::max(settled, offset);
    }
  }
  EXPECT_GT(worst, 0.5);
  EXPECT_LT(settled, 0.05);
}

TEST(Driver, RejectsInvalidParams) {
  const Track track = Track::reference();
  DriverParams params;
  params.grip_usage = 0.0;
  EXPECT_THROW(Driver(track, VehicleParams{}, params), std::invalid_argument);
  params = DriverParams{};
  params.steer_preview_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Driver(track, VehicleParams{}, params), std::invalid_argument);
  params = DriverParams{};
  params.min_preview_m = -1.0;
  EXPECT_THROW(Driver(track, VehicleParams{}, params), std::invalid_argument);
  params = DriverParams{};
  params.speed_response_s = 0.0;
  EXPECT_THROW(Driver(track, VehicleParams{}, params), std::invalid_argument);
  VehicleParams no_brakes;
  no_brakes.max_brake_force_n = 0.0;
  EXPECT_THROW(Driver(track, no_brakes), std::invalid_argument);
  VehicleParams massless;
  massless.mass_kg = 0.0;
  EXPECT_THROW(Driver(track, massless), std::invalid_argument);
}
