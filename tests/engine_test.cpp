#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "sim/engine.hpp"
#include "sim/gearbox.hpp"

using sim::Engine;
using sim::Gearbox;

TEST(Engine, TorqueCurveHitsPointsAndInterpolates) {
  const Engine e({{1000, 100}, {2000, 300}}, 500, 3000);
  EXPECT_DOUBLE_EQ(e.full_load_torque(1000), 100);
  EXPECT_DOUBLE_EQ(e.full_load_torque(2000), 300);
  EXPECT_DOUBLE_EQ(e.full_load_torque(1500), 200);
}

TEST(Engine, ExtrapolationIsFlatAndNanIsSafe) {
  const Engine e({{1000, 100}, {2000, 300}}, 500, 3000);
  EXPECT_DOUBLE_EQ(e.full_load_torque(0), 100);
  EXPECT_DOUBLE_EQ(e.full_load_torque(2500), 300);
  EXPECT_DOUBLE_EQ(e.full_load_torque(std::numeric_limits<double>::quiet_NaN()), 100);
}

TEST(Engine, ThrottleScalesAndIsClamped) {
  const Engine e({{1000, 100}, {2000, 300}}, 500, 3000);
  EXPECT_DOUBLE_EQ(e.torque(1500, 0.5), 100);
  EXPECT_DOUBLE_EQ(e.torque(1500, 2.0), 200);
  EXPECT_DOUBLE_EQ(e.torque(1500, -1.0), 0);
}

TEST(Engine, RevLimiterCutsTorqueAtRedline) {
  const Engine e({{1000, 100}, {2000, 300}}, 500, 3000);
  EXPECT_GT(e.torque(2999, 1.0), 0);
  EXPECT_DOUBLE_EQ(e.torque(3000, 1.0), 0);
}

TEST(Engine, ReferencePeakPowerIsPlausible) {
  const Engine e = Engine::reference();
  double peak_kw = 0;
  for (double rpm = 4000; rpm < 15000; rpm += 100) {
    peak_kw = std::max(peak_kw, e.full_load_torque(rpm) * rpm * 2 * 3.14159265 / 60 / 1000);
  }
  EXPECT_GT(peak_kw, 700);
  EXPECT_LT(peak_kw, 820);
}

TEST(Engine, RejectsInvalidConfig) {
  EXPECT_THROW(Engine({{1000, 100}}, 500, 3000), std::invalid_argument);
  EXPECT_THROW(Engine({{2000, 100}, {1000, 200}}, 500, 3000), std::invalid_argument);
  EXPECT_THROW(Engine({{1000, 100}, {1000, 200}}, 500, 3000), std::invalid_argument);
  EXPECT_THROW(Engine({{1000, -1}, {2000, 200}}, 500, 3000), std::invalid_argument);
  EXPECT_THROW(Engine({{1000, 100}, {2000, 200}}, 3000, 500), std::invalid_argument);
  EXPECT_THROW(Engine({{1000, std::numeric_limits<double>::infinity()}, {2000, 200}}, 500, 3000),
               std::invalid_argument);
}

TEST(Gearbox, OverallRatioAndSaturation) {
  const Gearbox g({3.0, 2.0, 1.0}, 4.0);
  EXPECT_EQ(g.gear_count(), 3);
  EXPECT_DOUBLE_EQ(g.overall_ratio(1), 12.0);
  EXPECT_DOUBLE_EQ(g.overall_ratio(3), 4.0);
  EXPECT_DOUBLE_EQ(g.overall_ratio(0), 12.0);
  EXPECT_DOUBLE_EQ(g.overall_ratio(99), 4.0);
}

TEST(Gearbox, RejectsInvalidConfig) {
  EXPECT_THROW(Gearbox({}, 4.0), std::invalid_argument);
  EXPECT_THROW(Gearbox({1.0, 2.0}, 4.0), std::invalid_argument);
  EXPECT_THROW(Gearbox({2.0, 2.0}, 4.0), std::invalid_argument);
  EXPECT_THROW(Gearbox({2.0, 0.0}, 4.0), std::invalid_argument);
  EXPECT_THROW(Gearbox({2.0, 1.0}, 0.0), std::invalid_argument);
}
