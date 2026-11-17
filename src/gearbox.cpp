#include "sim/gearbox.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim {

Gearbox::Gearbox(std::vector<double> ratios, double final_drive)
    : ratios_(std::move(ratios)), final_drive_(final_drive) {
  if (ratios_.empty()) throw std::invalid_argument("Gearbox: sem marchas");
  if (!(std::isfinite(final_drive_) && final_drive_ > 0.0)) {
    throw std::invalid_argument("Gearbox: relacao final invalida");
  }
  for (std::size_t i = 0; i < ratios_.size(); ++i) {
    if (!(std::isfinite(ratios_[i]) && ratios_[i] > 0.0)) {
      throw std::invalid_argument("Gearbox: relacao invalida");
    }
    if (i > 0 && ratios_[i] >= ratios_[i - 1]) {
      throw std::invalid_argument("Gearbox: relacoes devem ser estritamente decrescentes");
    }
  }
}

Gearbox Gearbox::reference() {
  return Gearbox({2.6, 2.0, 1.65, 1.4, 1.2, 1.05, 0.95, 0.88}, 6.0);
}

double Gearbox::overall_ratio(int gear) const noexcept {
  const int g = std::clamp(gear, 1, gear_count());
  return ratios_[static_cast<std::size_t>(g - 1)] * final_drive_;
}

}  // namespace sim
