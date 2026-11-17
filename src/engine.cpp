#include "sim/engine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim {

Engine::Engine(std::vector<TorquePoint> curve, double idle_rpm, double redline_rpm)
    : curve_(std::move(curve)), idle_rpm_(idle_rpm), redline_rpm_(redline_rpm) {
  if (curve_.size() < 2) throw std::invalid_argument("Engine: curva precisa de >= 2 pontos");
  if (!(std::isfinite(idle_rpm_) && std::isfinite(redline_rpm_) && idle_rpm_ > 0.0 &&
        idle_rpm_ < redline_rpm_)) {
    throw std::invalid_argument("Engine: exige 0 < idle < redline");
  }
  for (std::size_t i = 0; i < curve_.size(); ++i) {
    const auto& p = curve_[i];
    if (!std::isfinite(p.rpm) || !std::isfinite(p.torque_nm) || p.torque_nm < 0.0) {
      throw std::invalid_argument("Engine: ponto de curva invalido");
    }
    if (i > 0 && p.rpm <= curve_[i - 1].rpm) {
      throw std::invalid_argument("Engine: rpm deve ser estritamente crescente");
    }
  }
}

Engine Engine::reference() {
  return Engine({{4000, 380},
                 {6000, 470},
                 {8000, 540},
                 {10000, 590},
                 {11000, 600},
                 {12000, 590},
                 {13000, 560},
                 {14000, 520},
                 {15000, 470}},
                /*idle_rpm=*/4000.0, /*redline_rpm=*/15000.0);
}

double Engine::full_load_torque(double rpm) const noexcept {
  if (!(rpm > curve_.front().rpm)) return curve_.front().torque_nm;  // inclui NaN
  if (rpm >= curve_.back().rpm) return curve_.back().torque_nm;
  const auto hi = std::upper_bound(curve_.begin(), curve_.end(), rpm,
                                   [](double r, const TorquePoint& p) { return r < p.rpm; });
  const auto lo = hi - 1;
  const double f = (rpm - lo->rpm) / (hi->rpm - lo->rpm);
  return lo->torque_nm + f * (hi->torque_nm - lo->torque_nm);
}

double Engine::torque(double rpm, double throttle) const noexcept {
  if (rpm >= redline_rpm_) return 0.0;
  return std::clamp(throttle, 0.0, 1.0) * full_load_torque(rpm);
}

}  // namespace sim
