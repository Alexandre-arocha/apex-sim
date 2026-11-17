#include "sim/tyre.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace sim {
namespace {

// mu nunca cai abaixo desta fracao do nominal, por maior que seja a carga.
constexpr double kMinMuFraction = 0.5;

}  // namespace

Tyre::Tyre(TyreParams params) : params_(params) {
  const TyreParams& p = params_;
  const bool ok = std::isfinite(p.mu) && p.mu > 0.0 && std::isfinite(p.nominal_load_n) &&
                  p.nominal_load_n > 0.0 && p.load_sensitivity >= 0.0 &&
                  p.load_sensitivity < 1.0 && std::isfinite(p.stiffness_b) &&
                  p.stiffness_b > 0.0 && p.shape_c >= 1.0 && p.shape_c < 2.0;
  if (!ok) throw std::invalid_argument("TyreParams invalidos");
}

Tyre Tyre::reference_front() { return Tyre({.mu = 1.6}); }

Tyre Tyre::reference_rear() { return Tyre({.mu = 1.7}); }

double Tyre::peak_force(double load_n) const noexcept {
  if (!(load_n > 0.0)) return 0.0;  // inclui NaN
  const double relative_mu =
      1.0 - params_.load_sensitivity * (load_n / params_.nominal_load_n - 1.0);
  return params_.mu * std::max(kMinMuFraction, relative_mu) * load_n;
}

double Tyre::lateral_force(double slip_angle_rad, double load_n) const noexcept {
  return peak_force(load_n) *
         std::sin(params_.shape_c * std::atan(params_.stiffness_b * slip_angle_rad));
}

double Tyre::cornering_stiffness(double load_n) const noexcept {
  return params_.stiffness_b * params_.shape_c * peak_force(load_n);
}

double Tyre::peak_slip_angle() const noexcept {
  return std::tan(std::numbers::pi / (2.0 * params_.shape_c)) / params_.stiffness_b;
}

}  // namespace sim
