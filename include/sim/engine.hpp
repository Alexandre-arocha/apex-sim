#pragma once

#include <vector>

namespace sim {

struct TorquePoint {
  double rpm;
  double torque_nm;
};

// Motor com curva de torque a plena carga (interpolacao linear, extrapolacao plana).
class Engine {
 public:
  // Lanca std::invalid_argument se a curva for invalida (menos de 2 pontos,
  // rpm nao estritamente crescente, valores negativos/nao finitos) ou idle >= redline.
  Engine(std::vector<TorquePoint> curve, double idle_rpm, double redline_rpm);

  // Motor de referencia ~760 kW, 15000 rpm (ordem de grandeza de F1).
  [[nodiscard]] static Engine reference();

  [[nodiscard]] double full_load_torque(double rpm) const noexcept;
  // throttle em [0,1]. Corte de combustivel (0) em rpm >= redline.
  [[nodiscard]] double torque(double rpm, double throttle) const noexcept;

  [[nodiscard]] double idle_rpm() const noexcept { return idle_rpm_; }
  [[nodiscard]] double redline_rpm() const noexcept { return redline_rpm_; }

 private:
  std::vector<TorquePoint> curve_;
  double idle_rpm_;
  double redline_rpm_;
};

}  // namespace sim
