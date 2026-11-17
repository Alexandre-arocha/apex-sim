#pragma once

namespace sim {

struct TyreParams {
  double mu = 1.8;                // atrito de pico na carga nominal
  double nominal_load_n = 4000.0;
  double load_sensitivity = 0.1;  // fracao de mu perdida a cada carga nominal adicional, em [0, 1)
  double stiffness_b = 18.0;      // Pacejka B (1/rad)
  double shape_c = 1.4;           // Pacejka C, em [1, 2)
};

// Pneu com a formula magica de Pacejka simplificada (sem E e sem offsets):
//   Fy = D sin(C atan(B alpha)),  D = mu(Fz) Fz,  com mu caindo linearmente com a carga.
class Tyre {
 public:
  // Lanca std::invalid_argument se algum parametro for invalido.
  explicit Tyre(TyreParams params);

  // Pneus de referencia (ordem de grandeza de F1); o traseiro, mais largo, tem mais aderencia.
  [[nodiscard]] static Tyre reference_front();
  [[nodiscard]] static Tyre reference_rear();

  // Maior forca (N) que o pneu transmite com a carga vertical load_n; 0 se load_n <= 0.
  [[nodiscard]] double peak_force(double load_n) const noexcept;
  // Forca lateral em deriva pura (N), com o sinal de slip_angle_rad.
  [[nodiscard]] double lateral_force(double slip_angle_rad, double load_n) const noexcept;
  // Rigidez de deriva dFy/dalpha em alpha = 0 (N/rad) = B C D.
  [[nodiscard]] double cornering_stiffness(double load_n) const noexcept;
  // Angulo de deriva em que a forca lateral e maxima (rad).
  [[nodiscard]] double peak_slip_angle() const noexcept;

  [[nodiscard]] const TyreParams& params() const noexcept { return params_; }

 private:
  TyreParams params_;
};

}  // namespace sim
