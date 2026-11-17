#pragma once

#include <cstddef>
#include <vector>

namespace sim {

// Ponto do perfil de curvatura que define a pista. Entre dois pontos a curvatura varia
// linearmente: constante em 0 e reta, constante em 1/raio e arco, em rampa e clotoide.
struct CurvaturePoint {
  double s_m;            // distancia ao longo da linha, a partir da largada
  double curvature_1pm;  // 1/raio; positiva = curva a esquerda
};

// Amostra da linha da pista.
struct TrackPoint {
  double s_m;
  double x_m;
  double y_m;
  double heading_rad;  // direcao da linha, anti-horaria a partir de +x, sem normalizar
  double curvature_1pm;
};

// Onde um ponto do plano esta em relacao a pista.
struct TrackPosition {
  double s_m;        // distancia ao longo da linha ate o ponto mais proximo, em [0, comprimento)
  double lateral_m;  // distancia a linha; positiva = a esquerda dela
};

// Circuito fechado: uma linha (a linha ideal que o piloto segue) descrita por um perfil de
// curvatura e amostrada a cada ~1 m. A linha parte da origem apontando para +x, que e a pose
// inicial de Vehicle, e pode girar para qualquer um dos lados.
class Track {
 public:
  // Lanca std::invalid_argument se o perfil for invalido (menos de 2 pontos, s que nao comeca
  // em 0 ou nao cresce, valores nao finitos, mais de 1000 km) ou se a linha nao fechar: ela
  // tem de dar exatamente uma volta e terminar onde comecou.
  explicit Track(const std::vector<CurvaturePoint>& profile);

  // Circuito de referencia de 2,2 km: tres curvas, cada uma feita duas vezes por volta.
  // Esquerda de 90 graus com 40 m de raio, direita de 60 graus com 80 m e esquerda de
  // 150 graus com 50 m. A largada fica no meio da reta de 500 m.
  [[nodiscard]] static Track reference();

  [[nodiscard]] double length_m() const noexcept { return length_m_; }
  // Amostras igualmente espacadas. A ultima fecha a volta: mesma posicao da primeira, com
  // s = comprimento.
  [[nodiscard]] const std::vector<TrackPoint>& points() const noexcept { return points_; }
  // s levado para [0, comprimento); nao finito vira 0.
  [[nodiscard]] double wrap(double s_m) const noexcept;
  // A linha em s (qualquer valor: da a volta), interpolada entre amostras.
  [[nodiscard]] TrackPoint at(double s_m) const noexcept;
  // Ponto da linha mais proximo de (x, y), procurando na pista inteira.
  [[nodiscard]] TrackPosition locate(double x_m, double y_m) const noexcept;
  // Idem, procurando a partir de s_hint_m (a posicao do passo anterior): acompanha o carro ao
  // longo da linha sem saltar para outro trecho que passe perto.
  [[nodiscard]] TrackPosition locate(double x_m, double y_m, double s_hint_m) const noexcept;

 private:
  [[nodiscard]] TrackPosition project(double x_m, double y_m, std::size_t nearest) const noexcept;

  std::vector<TrackPoint> points_;
  double length_m_ = 0.0;
  double spacing_m_ = 0.0;
};

// Cronometro de voltas: detecta a passagem pela linha de largada (s = 0) a partir da posicao
// do carro na pista e interpola o instante da passagem dentro do passo.
class LapTimer {
 public:
  // Lanca std::invalid_argument se o comprimento nao for positivo e finito.
  explicit LapTimer(double track_length_m);

  // Chame a cada passo com o tempo e a posicao do carro na pista (TrackPosition::s_m).
  void update(double time_s, double s_m);
  // Tempos das voltas completas (s). A primeira conta desde a primeira chamada de update.
  // Cruzar a linha de re nao conta, nem a passagem para a frente que so desfaz esse recuo.
  [[nodiscard]] const std::vector<double>& lap_times_s() const noexcept { return lap_times_s_; }

 private:
  double track_length_m_;
  bool started_ = false;
  double last_time_s_ = 0.0;
  double last_s_m_ = 0.0;
  double lap_start_time_s_ = 0.0;
  int reversed_ = 0;  // passagens de re ainda nao desfeitas
  std::vector<double> lap_times_s_;
};

}  // namespace sim
