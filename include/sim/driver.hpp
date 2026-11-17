#pragma once

#include <vector>

#include "sim/track.hpp"
#include "sim/vehicle.hpp"

namespace sim {

struct DriverParams {
  // Fracao da aderencia de cada pneu que o piloto se permite usar. A folga cobre o que a
  // estimativa quase-estatica nao ve, como os transitorios de entrada e saida de curva.
  double grip_usage = 0.9;
  double steer_preview_s = 0.4;  // a direcao mira o ponto onde o carro estara neste tempo
  double min_preview_m = 5.0;    // distancia minima de mira, para baixa velocidade
  double speed_response_s = 0.1;  // tempo em que os pedais tiram um erro de velocidade
};

// Piloto virtual: segue a linha da pista o mais rapido que a aderencia deixa.
// Ao ser criado calcula o limite de velocidade em cada ponto da pista: o da curva naquele
// ponto e o que ainda permite frear para as curvas seguintes. Ao guiar acelera a fundo abaixo
// do limite, freia ao chegar nele e esterca o que os pneus pedem para fazer a curva, corrigindo
// pelo desvio em relacao a linha.
// A aderencia e estimada roda a roda (circulo de atrito com peso, downforce e transferencia
// longitudinal e lateral de carga), e o piloto nunca pede a um pneu mais do que grip_usage dela.
class Driver {
 public:
  // Lanca std::invalid_argument se car for invalido para Vehicle ou nao tiver freio, ou se
  // algum parametro do piloto nao for positivo e finito.
  Driver(Track track, VehicleParams car, DriverParams params = {});

  // Comandos para o proximo passo de Vehicle, a partir do estado atual do carro.
  [[nodiscard]] Controls drive(const VehicleState& state);

  // Limite de velocidade em s (m/s). Nas retas longe de curvas e so um teto alto: ali quem
  // limita o carro e o motor.
  [[nodiscard]] double speed_limit_mps(double s_m) const noexcept;
  // Tempo previsto da volta lancada (s): o limite acima mais a aceleracao que motor e tracao
  // permitem, com o carro tratado como um ponto. Serve para conferir a simulacao.
  [[nodiscard]] double predicted_lap_time_s() const noexcept { return predicted_lap_time_s_; }
  // Posicao do carro na pista no ultimo drive().
  [[nodiscard]] const TrackPosition& position() const noexcept { return position_; }

 private:
  void plan();
  [[nodiscard]] double corner_speed(double curvature) const noexcept;
  [[nodiscard]] double brake_force_limit(double speed, double lat_accel) const noexcept;
  [[nodiscard]] double drive_force_limit(double speed, double lat_accel,
                                         double engine_force) const noexcept;

  Track track_;
  VehicleParams car_;
  DriverParams params_;
  std::vector<double> speed_limit_mps_;  // nos pontos da pista
  double predicted_lap_time_s_ = 0.0;
  TrackPosition position_{};
  bool located_ = false;
};

}  // namespace sim
