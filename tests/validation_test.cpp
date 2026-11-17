// Validacao do modelo contra numeros publicos de F1. As faixas sao as que varias fontes
// publicas citam para um carro moderno; sao ordens de grandeza, nao dados de uma equipe (ver o
// README, secao "Validacao"). O objetivo e o carro de referencia cair dentro delas, nao
// reproduzir um carro especifico.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "sim/driver.hpp"
#include "sim/track.hpp"
#include "sim/vehicle.hpp"

using sim::CurvaturePoint;
using sim::Driver;
using sim::DriverParams;
using sim::Track;
using sim::Vehicle;
using sim::VehicleParams;

namespace {

constexpr double kG = 9.80665;
constexpr double kKmh = 1.0 / 3.6;

struct FullThrottle {
  double t100 = -1;
  double t200 = -1;
  double t300 = -1;
  double top_speed_mps = 0;
};

// Largada parada, acelerador a fundo por 200 s: tempos de 0 a 100/200/300 km/h e velocidade final.
FullThrottle full_throttle_run(const VehicleParams& params) {
  Vehicle car{params};
  FullThrottle r;
  for (int i = 0; i < 200000; ++i) {
    car.step({1.0, 0.0, 0.0});
    const double v = car.state().speed_mps;
    const double t = car.state().time_s;
    if (r.t100 < 0 && v >= 100 * kKmh) r.t100 = t;
    if (r.t200 < 0 && v >= 200 * kKmh) r.t200 = t;
    if (r.t300 < 0 && v >= 300 * kKmh) r.t300 = t;
  }
  r.top_speed_mps = car.state().speed_mps;
  return r;
}

struct Braking {
  double peak_g = 0;
  double distance_m = 0;
  double time_s = 0;
};

// Freio a fundo em reta a partir de from_kmh ate parar.
Braking brake_from(const VehicleParams& params, double from_kmh) {
  Vehicle car{params, from_kmh * kKmh};
  Braking r;
  while (car.state().speed_mps > 0.0 && car.state().step_count < 30000) {
    car.step({0.0, 1.0, 0.0});
    r.peak_g = std::max(r.peak_g, -car.state().accel_long_mps2 / kG);
  }
  r.distance_m = car.state().position_m;
  r.time_s = car.state().time_s;
  return r;
}

// Aceleracao lateral maxima (g) numa curva de raio constante, pelo limite de velocidade do piloto
// com 100% da aderencia (o piloto foi conferido contra o skidpad simulado nos testes dele).
double lateral_g_at_radius(const VehicleParams& params, double radius_m) {
  const Track circle(std::vector<CurvaturePoint>{{0.0, 1.0 / radius_m},
                                                 {2.0 * std::numbers::pi * radius_m, 1.0 / radius_m}});
  DriverParams all_the_grip;
  all_the_grip.grip_usage = 1.0;
  const double v = Driver{circle, params, all_the_grip}.speed_limit_mps(0.0);
  return v * v / radius_m / kG;
}

double downforce_over_weight(const VehicleParams& p, double speed_kmh) {
  const double v = speed_kmh * kKmh;
  return 0.5 * p.air_density_kgm3 * v * v * p.downforce_area_m2 / (p.mass_kg * kG);
}

}  // namespace

// Fontes citam 0-100 km/h entre ~2,4 e 2,8 s, 0-200 entre ~4,5 e 5,5 s e 0-300 entre ~8,4 e 10,6 s.
TEST(Validation, AccelerationTimesAreInThePublicRange) {
  const FullThrottle r = full_throttle_run(VehicleParams{});
  EXPECT_GE(r.t100, 2.4);
  EXPECT_LE(r.t100, 2.8);
  EXPECT_GE(r.t200, 4.5);
  EXPECT_LE(r.t200, 5.5);
  EXPECT_GE(r.t300, 8.4);
  EXPECT_LE(r.t300, 10.6);
}

// Velocidades maximas de carros de F1 modernos ficam entre ~320 e ~375 km/h conforme a pista.
TEST(Validation, TopSpeedIsInThePublicRange) {
  const double top_kmh = full_throttle_run(VehicleParams{}).top_speed_mps * 3.6;
  EXPECT_GE(top_kmh, 320.0);
  EXPECT_LE(top_kmh, 375.0);
}

// A frenagem de pico de um F1 e citada como 5-6 g a partir de ~300 km/h.
TEST(Validation, PeakBrakingIsInThePublicRange) {
  const Braking b = brake_from(VehicleParams{}, 300.0);
  EXPECT_GE(b.peak_g, 5.0);
  EXPECT_LE(b.peak_g, 6.5);
}

// Fontes dizem que o downforce iguala o peso a ~150 km/h e chega a 3-4 vezes o peso perto da
// velocidade maxima; a relacao sustentacao/arrasto do conceito de 2022 era ~4.
TEST(Validation, AerodynamicsMatchThePublicFigures) {
  const VehicleParams p;
  EXPECT_GE(downforce_over_weight(p, 150.0), 0.6);
  EXPECT_LE(downforce_over_weight(p, 150.0), 1.2);
  EXPECT_GE(downforce_over_weight(p, 300.0), 2.5);
  EXPECT_LE(downforce_over_weight(p, 300.0), 4.5);
  EXPECT_GE(p.downforce_area_m2 / p.drag_area_m2, 3.5);
  EXPECT_LE(p.downforce_area_m2 / p.drag_area_m2, 4.5);
}

// Curvas rapidas de F1 sao citadas com 5-6 g laterais; as lentas ficam bem abaixo disso, entre
// ~2 e ~3,5 g.
TEST(Validation, LateralGripGrowsWithSpeedAndReachesFastCornerLevels) {
  const VehicleParams p;
  const double slow = lateral_g_at_radius(p, 40.0);
  const double fast = lateral_g_at_radius(p, 135.0);  // limite a ~300 km/h
  EXPECT_GE(slow, 2.0);
  EXPECT_LE(slow, 3.5);
  EXPECT_GE(fast, 4.5);
  EXPECT_LE(fast, 6.5);
  EXPECT_GT(fast, slow);
}
