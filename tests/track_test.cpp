#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "sim/track.hpp"

using sim::CurvaturePoint;
using sim::LapTimer;
using sim::Track;
using sim::TrackPoint;
using sim::TrackPosition;

namespace {

constexpr double kPi = std::numbers::pi;

using Profile = std::vector<CurvaturePoint>;

// Circulo de raio |radius_m|: anti-horario se radius_m > 0, horario se < 0.
Track circle(double radius_m) {
  return Track(Profile{{0.0, 1.0 / radius_m}, {2.0 * kPi * std::abs(radius_m), 1.0 / radius_m}});
}

}  // namespace

TEST(Track, CircleMatchesTheAnalyticShape) {
  const double radius = 50.0;
  const Track track = circle(radius);
  EXPECT_DOUBLE_EQ(track.length_m(), 2.0 * kPi * radius);
  for (const TrackPoint& p : track.points()) {
    const double angle = p.s_m / radius;
    EXPECT_NEAR(p.x_m, radius * std::sin(angle), 1e-9);
    EXPECT_NEAR(p.y_m, radius * (1.0 - std::cos(angle)), 1e-9);
    EXPECT_NEAR(p.heading_rad, angle, 1e-12);
    EXPECT_DOUBLE_EQ(p.curvature_1pm, 1.0 / radius);
  }
}

TEST(Track, NegativeCurvatureTurnsRight) {
  const Track track = circle(-30.0);
  EXPECT_NEAR(track.points().back().heading_rad, -2.0 * kPi, 1e-9);
  const TrackPoint quarter = track.at(track.length_m() / 4.0);
  EXPECT_NEAR(quarter.x_m, 30.0, 0.01);
  EXPECT_NEAR(quarter.y_m, -30.0, 0.01);
}

// Quadrado de cantos arredondados: quatro curvas de 90 graus feitas so de clotoide (a
// curvatura sobe e desce), com retas entre elas.
TEST(Track, ClothoidMatchesAnIndependentQuadrature) {
  const double ramp = 40.0;               // comprimento de cada clotoide
  const double peak = (kPi / 2.0) / ramp;  // as duas rampas de uma curva somam 90 graus
  Profile profile{{0.0, 0.0}};
  for (int corner = 0; corner < 4; ++corner) {
    const double s = profile.back().s_m;
    profile.push_back({s + ramp, peak});
    profile.push_back({s + 2.0 * ramp, 0.0});
    profile.push_back({s + 2.0 * ramp + 20.0, 0.0});
  }
  const Track track{profile};

  const TrackPoint& end_of_ramp = track.points()[40];
  ASSERT_DOUBLE_EQ(end_of_ramp.s_m, ramp);
  // Na primeira clotoide o rumo e (peak / ramp) s^2 / 2; integra a posicao pelo ponto medio.
  const int steps = 200000;
  const double ds = ramp / steps;
  double x = 0.0;
  double y = 0.0;
  for (int i = 0; i < steps; ++i) {
    const double s = (i + 0.5) * ds;
    const double heading = 0.5 * (peak / ramp) * s * s;
    x += std::cos(heading) * ds;
    y += std::sin(heading) * ds;
  }
  EXPECT_NEAR(end_of_ramp.x_m, x, 1e-8);
  EXPECT_NEAR(end_of_ramp.y_m, y, 1e-8);
  EXPECT_NEAR(end_of_ramp.heading_rad, kPi / 4.0, 1e-12);

  const TrackPoint& end_of_corner = track.points()[80];
  EXPECT_NEAR(end_of_corner.heading_rad, kPi / 2.0, 1e-12);
  EXPECT_NEAR(end_of_corner.x_m, end_of_corner.y_m, 1e-9);  // curva simetrica: corda a 45 graus
}

TEST(Track, ReferenceIsAClosedCircuit) {
  const Track track = Track::reference();
  EXPECT_NEAR(track.length_m(), 2215.0, 0.1);
  const TrackPoint& start = track.points().front();
  const TrackPoint& finish = track.points().back();
  EXPECT_DOUBLE_EQ(start.x_m, 0.0);
  EXPECT_DOUBLE_EQ(start.y_m, 0.0);
  EXPECT_DOUBLE_EQ(start.heading_rad, 0.0);
  EXPECT_DOUBLE_EQ(finish.x_m, 0.0);
  EXPECT_DOUBLE_EQ(finish.y_m, 0.0);
  EXPECT_NEAR(finish.s_m, track.length_m(), 1e-9);
  EXPECT_NEAR(finish.heading_rad, 2.0 * kPi, 1e-9);  // uma volta anti-horaria
}

TEST(Track, SamplesAreEvenlySpacedAndConsistent) {
  const Track track = Track::reference();
  const std::vector<TrackPoint>& points = track.points();
  const double spacing = track.length_m() / static_cast<double>(points.size() - 1);
  EXPECT_GT(spacing, 0.9);
  EXPECT_LE(spacing, 1.0);
  double tightest = 0.0;
  double most_to_the_right = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const TrackPoint& a = points[i];
    const TrackPoint& b = points[i + 1];
    EXPECT_NEAR(b.s_m - a.s_m, spacing, 1e-9);
    const double chord = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    EXPECT_LE(chord, spacing + 1e-12);  // a corda nao e maior que o arco
    EXPECT_GT(chord, 0.999 * spacing);
    // A corda aponta no rumo medio do trecho, e o rumo muda pela curvatura media.
    const double direction = std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
    EXPECT_NEAR(std::remainder(direction - 0.5 * (a.heading_rad + b.heading_rad), 2.0 * kPi), 0.0,
                1e-4);
    EXPECT_NEAR(b.heading_rad - a.heading_rad,
                0.5 * (a.curvature_1pm + b.curvature_1pm) * spacing, 2e-4);
    tightest = std::max(tightest, std::abs(a.curvature_1pm));
    most_to_the_right = std::min(most_to_the_right, a.curvature_1pm);
  }
  EXPECT_DOUBLE_EQ(tightest, 1.0 / 40.0);
  EXPECT_DOUBLE_EQ(most_to_the_right, -1.0 / 80.0);
}

TEST(Track, WrapsDistanceAroundTheLap) {
  const Track track = circle(50.0);
  const double length = track.length_m();
  EXPECT_DOUBLE_EQ(track.wrap(10.0), 10.0);
  EXPECT_DOUBLE_EQ(track.wrap(length), 0.0);
  EXPECT_NEAR(track.wrap(2.5 * length), 0.5 * length, 1e-9);
  EXPECT_NEAR(track.wrap(-1.0), length - 1.0, 1e-9);
  EXPECT_LT(track.wrap(-1e-20), length);
  EXPECT_DOUBLE_EQ(track.wrap(std::numeric_limits<double>::quiet_NaN()), 0.0);
  EXPECT_DOUBLE_EQ(track.wrap(std::numeric_limits<double>::infinity()), 0.0);
}

TEST(Track, InterpolatesTheLineBetweenSamples) {
  const double radius = 50.0;
  const Track track = circle(radius);
  for (double s = 0.0; s < track.length_m(); s += 7.3) {
    const TrackPoint p = track.at(s);
    EXPECT_DOUBLE_EQ(p.s_m, s);
    // Entre amostras a linha e a corda: fica ate ~2,5 mm para dentro do arco.
    EXPECT_NEAR(p.x_m, radius * std::sin(s / radius), 4e-3);
    EXPECT_NEAR(p.y_m, radius * (1.0 - std::cos(s / radius)), 4e-3);
    EXPECT_NEAR(p.heading_rad, s / radius, 1e-12);
    const TrackPoint laps_later = track.at(s + 3.0 * track.length_m());
    EXPECT_NEAR(laps_later.x_m, p.x_m, 1e-9);
    EXPECT_NEAR(laps_later.y_m, p.y_m, 1e-9);
  }
}

TEST(Track, LocatesPointsBesideTheLine) {
  const Track track = Track::reference();
  for (double s = 0.5; s < track.length_m(); s += 13.7) {
    const TrackPoint line = track.at(s);
    for (const double offset : {-3.0, 0.0, 1.5}) {  // positivo = a esquerda da linha
      const double x = line.x_m - offset * std::sin(line.heading_rad);
      const double y = line.y_m + offset * std::cos(line.heading_rad);
      const TrackPosition found = track.locate(x, y);
      EXPECT_NEAR(found.s_m, s, 0.05);
      EXPECT_NEAR(found.lateral_m, offset, 0.01);
      // Com uma posicao anterior como ponto de partida, de tras ou da frente, acha o mesmo.
      for (const double hint : {s - 15.0, s + 15.0}) {
        const TrackPosition followed = track.locate(x, y, hint);
        EXPECT_NEAR(followed.s_m, found.s_m, 1e-9);
        EXPECT_NEAR(followed.lateral_m, found.lateral_m, 1e-9);
      }
    }
  }
}

TEST(Track, RejectsInvalidProfiles) {
  const double k = 1.0 / 50.0;
  const double lap = 2.0 * kPi * 50.0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Track(Profile{{0.0, k}}), std::invalid_argument);            // um ponto so
  EXPECT_THROW(Track(Profile{{1.0, k}, {lap, k}}), std::invalid_argument);  // nao comeca em 0
  EXPECT_THROW(Track(Profile{{0.0, k}, {lap, k}, {lap, k}}), std::invalid_argument);  // s parado
  EXPECT_THROW(Track(Profile{{0.0, nan}, {lap, k}}), std::invalid_argument);
  EXPECT_THROW(Track(Profile{{0.0, 0.0}, {2.0e6, 0.0}}), std::invalid_argument);  // longa demais
  EXPECT_THROW(Track(Profile{{0.0, 0.0}, {100.0, 0.0}}), std::invalid_argument);  // reta
  EXPECT_THROW(Track(Profile{{0.0, k}, {0.9 * lap, k}}), std::invalid_argument);  // volta incompleta
  EXPECT_THROW(Track(Profile{{0.0, k}, {2.0 * lap, k}}), std::invalid_argument);  // duas voltas
  // Dois semicirculos de raios 50 e 30 m: o rumo da uma volta, mas a linha termina a 40 m
  // da largada.
  const double half = kPi * 50.0;
  const double joint_turn = 0.5 * (1.0 / 50.0 + 1.0 / 30.0) * 0.01;
  EXPECT_THROW(Track(Profile{{0.0, 1.0 / 50.0},
                             {half, 1.0 / 50.0},
                             {half + 0.01, 1.0 / 30.0},
                             {half + 0.01 + 30.0 * (kPi - joint_turn), 1.0 / 30.0}}),
               std::invalid_argument);
}

TEST(LapTimer, TimesLapsToTheInstantOfCrossingTheLine) {
  const double length = 1000.0;
  const double speed = 50.0;
  const double dt = 0.07;  // 3,5 m por passo: a linha fica no meio de um passo
  LapTimer timer{length};
  for (int i = 0; i * dt < 65.0; ++i) {
    timer.update(i * dt, std::fmod(300.0 + speed * i * dt, length));
  }
  ASSERT_EQ(timer.lap_times_s().size(), 3u);
  EXPECT_NEAR(timer.lap_times_s()[0], 14.0, 1e-9);  // largou a 700 m da linha
  EXPECT_NEAR(timer.lap_times_s()[1], 20.0, 1e-9);
  EXPECT_NEAR(timer.lap_times_s()[2], 20.0, 1e-9);
}

TEST(LapTimer, CrossingBackwardsDoesNotCountALap) {
  LapTimer timer{1000.0};
  timer.update(0.0, 990.0);
  timer.update(1.0, 10.0);   // cruzou a linha: uma volta
  timer.update(2.0, 990.0);  // voltou de re
  timer.update(3.0, 10.0);   // refez o trecho: nao e volta nova
  ASSERT_EQ(timer.lap_times_s().size(), 1u);
  EXPECT_DOUBLE_EQ(timer.lap_times_s()[0], 0.5);
  timer.update(10.0, 400.0);
  timer.update(20.0, 800.0);
  timer.update(25.0, 990.0);
  timer.update(26.0, 10.0);
  ASSERT_EQ(timer.lap_times_s().size(), 2u);
  EXPECT_DOUBLE_EQ(timer.lap_times_s()[1], 25.0);  // de 0,5 s a 25,5 s
}

TEST(LapTimer, RejectsInvalidLength) {
  EXPECT_THROW(LapTimer{0.0}, std::invalid_argument);
  EXPECT_THROW(LapTimer{-5.0}, std::invalid_argument);
  EXPECT_THROW(LapTimer{std::numeric_limits<double>::infinity()}, std::invalid_argument);
  EXPECT_THROW(LapTimer{std::numeric_limits<double>::quiet_NaN()}, std::invalid_argument);
}
