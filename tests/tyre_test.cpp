#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "sim/tyre.hpp"

using sim::Tyre;
using sim::TyreParams;

namespace {

constexpr double kLoad = 4000.0;  // carga nominal do pneu padrao

}  // namespace

TEST(Tyre, NoSlipNoForceAndCurveIsOdd) {
  const Tyre tyre{TyreParams{}};
  EXPECT_DOUBLE_EQ(tyre.lateral_force(0.0, kLoad), 0.0);
  for (double slip = 0.01; slip < 0.5; slip += 0.01) {
    EXPECT_DOUBLE_EQ(tyre.lateral_force(-slip, kLoad), -tyre.lateral_force(slip, kLoad));
  }
}

TEST(Tyre, PeakIsMuTimesLoadAtThePeakSlipAngle) {
  const TyreParams p;
  const Tyre tyre{p};
  const double peak_slip = tyre.peak_slip_angle();
  EXPECT_GT(peak_slip, 0.05);  // ~3 graus
  EXPECT_LT(peak_slip, 0.20);  // ~11 graus
  EXPECT_DOUBLE_EQ(tyre.peak_force(kLoad), p.mu * kLoad);
  EXPECT_NEAR(tyre.lateral_force(peak_slip, kLoad), p.mu * kLoad, 1e-6);
  for (double slip = 0.0; slip < 1.5; slip += 0.005) {
    EXPECT_LE(tyre.lateral_force(slip, kLoad), tyre.peak_force(kLoad) + 1e-9);
  }
}

TEST(Tyre, ForceRisesToThePeakThenFallsOff) {
  const Tyre tyre{TyreParams{}};
  double last = 0.0;
  for (double slip = 0.002; slip < tyre.peak_slip_angle(); slip += 0.002) {
    const double force = tyre.lateral_force(slip, kLoad);
    EXPECT_GT(force, last);
    last = force;
  }
  const double sliding = tyre.lateral_force(1.0, kLoad);  // ~57 graus: pneu deslizando
  EXPECT_LT(sliding, tyre.peak_force(kLoad));
  EXPECT_GT(sliding, 0.6 * tyre.peak_force(kLoad));
}

TEST(Tyre, CorneringStiffnessIsTheSlopeAtZeroSlip) {
  const Tyre tyre{TyreParams{}};
  const double h = 1e-6;
  const double slope = (tyre.lateral_force(h, kLoad) - tyre.lateral_force(-h, kLoad)) / (2.0 * h);
  EXPECT_NEAR(slope, tyre.cornering_stiffness(kLoad), 1e-3 * slope);
  // Ordem de grandeza de pneu de corrida: alguns kN por grau.
  const double per_degree = tyre.cornering_stiffness(kLoad) * 3.14159265 / 180.0;
  EXPECT_GT(per_degree, 1500.0);
  EXPECT_LT(per_degree, 5000.0);
}

TEST(Tyre, GripCoefficientDropsWithLoadButForceStillGrows) {
  const Tyre tyre{TyreParams{}};
  double last_mu = std::numeric_limits<double>::infinity();
  double last_force = 0.0;
  for (double load = 500.0; load <= 12000.0; load += 500.0) {
    const double force = tyre.peak_force(load);
    EXPECT_LT(force / load, last_mu);
    EXPECT_GT(force, last_force);
    last_mu = force / load;
    last_force = force;
  }
}

TEST(Tyre, WithoutLoadSensitivityForceIsProportionalToLoad) {
  const Tyre tyre{TyreParams{.load_sensitivity = 0.0}};
  EXPECT_DOUBLE_EQ(tyre.peak_force(8000.0), 2.0 * tyre.peak_force(4000.0));
}

TEST(Tyre, NoLoadNoForce) {
  const Tyre tyre{TyreParams{}};
  EXPECT_DOUBLE_EQ(tyre.peak_force(0.0), 0.0);
  EXPECT_DOUBLE_EQ(tyre.peak_force(-100.0), 0.0);
  EXPECT_DOUBLE_EQ(tyre.peak_force(std::numeric_limits<double>::quiet_NaN()), 0.0);
  EXPECT_DOUBLE_EQ(tyre.lateral_force(0.1, 0.0), 0.0);
}

TEST(Tyre, RejectsInvalidParams) {
  EXPECT_THROW(Tyre(TyreParams{.mu = 0.0}), std::invalid_argument);
  EXPECT_THROW(Tyre(TyreParams{.mu = std::numeric_limits<double>::quiet_NaN()}),
               std::invalid_argument);
  EXPECT_THROW(Tyre(TyreParams{.nominal_load_n = -1.0}), std::invalid_argument);
  EXPECT_THROW(Tyre(TyreParams{.load_sensitivity = 1.0}), std::invalid_argument);
  EXPECT_THROW(Tyre(TyreParams{.stiffness_b = 0.0}), std::invalid_argument);
  EXPECT_THROW(Tyre(TyreParams{.shape_c = 0.9}), std::invalid_argument);
  EXPECT_THROW(Tyre(TyreParams{.shape_c = 2.0}), std::invalid_argument);
}
