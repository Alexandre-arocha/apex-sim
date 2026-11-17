#pragma once

#include <vector>

namespace sim {

// Caixa de marchas: relacoes por marcha (1-based) e relacao final.
class Gearbox {
 public:
  // Lanca std::invalid_argument se vazia, ou relacao <= 0 / nao finita,
  // ou relacoes nao estritamente decrescentes.
  Gearbox(std::vector<double> ratios, double final_drive);

  // 8 marchas, relacao final 6.0.
  [[nodiscard]] static Gearbox reference();

  [[nodiscard]] int gear_count() const noexcept { return static_cast<int>(ratios_.size()); }
  // Relacao total motor->roda (marcha * final). gear fora de [1, gear_count] e satura.
  [[nodiscard]] double overall_ratio(int gear) const noexcept;

 private:
  std::vector<double> ratios_;
  double final_drive_;
};

}  // namespace sim
