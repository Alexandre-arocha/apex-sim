#include "sim/track.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

constexpr double kSampleSpacing = 1.0;   // m entre amostras, aproximado
constexpr int kSubsteps = 8;             // passos de Simpson entre duas amostras
constexpr double kMaxLength = 1.0e6;     // m
constexpr double kMaxGap = 0.01;         // m entre o fim e o comeco da linha
constexpr double kMaxHeadingGap = 1e-4;  // rad a mais ou a menos do que uma volta

double squared_distance(const TrackPoint& point, double x, double y) noexcept {
  const double dx = x - point.x_m;
  const double dy = y - point.y_m;
  return dx * dx + dy * dy;
}

}  // namespace

Track::Track(const std::vector<CurvaturePoint>& profile) {
  if (profile.size() < 2) throw std::invalid_argument("Track: perfil precisa de >= 2 pontos");
  for (std::size_t i = 0; i < profile.size(); ++i) {
    const CurvaturePoint& p = profile[i];
    if (!std::isfinite(p.s_m) || !std::isfinite(p.curvature_1pm)) {
      throw std::invalid_argument("Track: ponto de perfil invalido");
    }
    if (i == 0 ? p.s_m != 0.0 : p.s_m <= profile[i - 1].s_m) {
      throw std::invalid_argument("Track: s deve comecar em 0 e crescer");
    }
  }
  length_m_ = profile.back().s_m;
  if (length_m_ > kMaxLength) throw std::invalid_argument("Track: pista longa demais");
  // Numero par de amostras: as duas metades de um circuito simetrico caem nos mesmos pontos.
  const std::size_t count =
      2 * static_cast<std::size_t>(std::ceil(length_m_ / (2.0 * kSampleSpacing)));
  spacing_m_ = length_m_ / static_cast<double>(count);

  // Rumo: integral exata da curvatura, que e linear em cada trecho do perfil.
  std::size_t knot = 0;
  double knot_heading = 0.0;  // rumo no ponto `knot` do perfil
  const auto line_at = [&](double s) {  // {rumo, curvatura}; s nao pode recuar entre chamadas
    while (knot + 2 < profile.size() && s > profile[knot + 1].s_m) {
      knot_heading += 0.5 * (profile[knot].curvature_1pm + profile[knot + 1].curvature_1pm) *
                      (profile[knot + 1].s_m - profile[knot].s_m);
      ++knot;
    }
    const CurvaturePoint& a = profile[knot];
    const CurvaturePoint& b = profile[knot + 1];
    const double u = s - a.s_m;
    const double rate = (b.curvature_1pm - a.curvature_1pm) / (b.s_m - a.s_m);
    return std::pair{knot_heading + (a.curvature_1pm + 0.5 * rate * u) * u,
                     a.curvature_1pm + rate * u};
  };

  // Posicao: integral de (cos, sin) do rumo, por Simpson.
  points_.reserve(count + 1);
  const double h = spacing_m_ / kSubsteps;
  double x = 0.0;
  double y = 0.0;
  for (std::size_t i = 0;; ++i) {
    const double s = static_cast<double>(i) * spacing_m_;
    const auto [heading, curvature] = line_at(s);
    points_.push_back({s, x, y, heading, curvature});
    if (i == count) break;
    for (int j = 0; j < kSubsteps; ++j) {
      const double from = s + j * h;
      const double h0 = line_at(from).first;
      const double h1 = line_at(from + 0.5 * h).first;
      const double h2 = line_at(from + h).first;
      x += h / 6.0 * (std::cos(h0) + 4.0 * std::cos(h1) + std::cos(h2));
      y += h / 6.0 * (std::sin(h0) + 4.0 * std::sin(h1) + std::sin(h2));
    }
  }

  TrackPoint& end = points_.back();
  if (std::abs(std::abs(end.heading_rad) - 2.0 * std::numbers::pi) > kMaxHeadingGap ||
      std::hypot(end.x_m, end.y_m) > kMaxGap) {
    throw std::invalid_argument("Track: a linha nao fecha uma volta");
  }
  end.x_m = 0.0;
  end.y_m = 0.0;
}

Track Track::reference() {
  constexpr double kDegree = std::numbers::pi / 180.0;
  std::vector<CurvaturePoint> profile{{0.0, 0.0}};
  const auto straight = [&profile](double length) {
    profile.push_back({profile.back().s_m + length, 0.0});
  };
  // Curva com entrada e saida em clotoide: a curvatura sobe ate 1/radius em `transition`
  // metros, fica constante e desce igual. angle_rad > 0 vira a esquerda.
  const auto corner = [&profile](double angle_rad, double radius, double transition) {
    const double curvature = std::copysign(1.0 / radius, angle_rad);
    double s = profile.back().s_m;
    profile.push_back({s += transition, curvature});
    profile.push_back({s += std::abs(angle_rad) * radius - transition, curvature});
    profile.push_back({s += transition, 0.0});
  };
  // Duas metades iguais, cada uma virando 180 graus: a volta fecha por simetria.
  for (int half = 0; half < 2; ++half) {
    straight(250.0);
    corner(90.0 * kDegree, 40.0, 30.0);
    straight(150.0);
    corner(-60.0 * kDegree, 80.0, 40.0);
    straight(80.0);
    corner(150.0 * kDegree, 50.0, 30.0);
    straight(250.0);
  }
  return Track(profile);
}

double Track::wrap(double s_m) const noexcept {
  if (!std::isfinite(s_m)) return 0.0;
  const double s = std::fmod(s_m, length_m_);
  const double wrapped = s < 0.0 ? s + length_m_ : s;
  return wrapped < length_m_ ? wrapped : 0.0;  // -1e-20 + comprimento arredonda para o comprimento
}

TrackPoint Track::at(double s_m) const noexcept {
  const double s = wrap(s_m);
  const double u = s / spacing_m_;
  const std::size_t i = std::min(static_cast<std::size_t>(u), points_.size() - 2);
  const double f = u - static_cast<double>(i);
  const TrackPoint& a = points_[i];
  const TrackPoint& b = points_[i + 1];
  return {s, a.x_m + f * (b.x_m - a.x_m), a.y_m + f * (b.y_m - a.y_m),
          a.heading_rad + f * (b.heading_rad - a.heading_rad),
          a.curvature_1pm + f * (b.curvature_1pm - a.curvature_1pm)};
}

TrackPosition Track::locate(double x_m, double y_m) const noexcept {
  std::size_t nearest = 0;
  for (std::size_t i = 1; i + 1 < points_.size(); ++i) {
    if (squared_distance(points_[i], x_m, y_m) < squared_distance(points_[nearest], x_m, y_m)) {
      nearest = i;
    }
  }
  return project(x_m, y_m, nearest);
}

TrackPosition Track::locate(double x_m, double y_m, double s_hint_m) const noexcept {
  const std::size_t count = points_.size() - 1;
  std::size_t nearest = static_cast<std::size_t>(wrap(s_hint_m) / spacing_m_) % count;
  // Desce a distancia de amostra em amostra, primeiro para a frente e depois para tras.
  for (const std::size_t step : {std::size_t{1}, count - 1}) {
    for (;;) {
      const std::size_t next = (nearest + step) % count;
      if (!(squared_distance(points_[next], x_m, y_m) <
            squared_distance(points_[nearest], x_m, y_m))) {
        break;
      }
      nearest = next;
    }
  }
  return project(x_m, y_m, nearest);
}

// Projeta (x, y) nos dois segmentos que saem da amostra `nearest` e fica com o mais proximo.
TrackPosition Track::project(double x_m, double y_m, std::size_t nearest) const noexcept {
  const std::size_t count = points_.size() - 1;
  TrackPosition best{};
  double best_distance = std::numeric_limits<double>::infinity();
  for (const std::size_t i : {nearest, (nearest + count - 1) % count}) {
    const TrackPoint& a = points_[i];
    const TrackPoint& b = points_[i + 1];
    const double dx = b.x_m - a.x_m;
    const double dy = b.y_m - a.y_m;
    const double px = x_m - a.x_m;
    const double py = y_m - a.y_m;
    const double t = std::clamp((px * dx + py * dy) / (dx * dx + dy * dy), 0.0, 1.0);
    const double distance = std::hypot(px - t * dx, py - t * dy);
    if (distance < best_distance) {
      best_distance = distance;
      best = {a.s_m + t * spacing_m_, std::copysign(distance, dx * py - dy * px)};
    }
  }
  best.s_m = wrap(best.s_m);  // o fim do ultimo segmento e a largada
  return best;
}

LapTimer::LapTimer(double track_length_m) : track_length_m_(track_length_m) {
  if (!(std::isfinite(track_length_m_) && track_length_m_ > 0.0)) {
    throw std::invalid_argument("LapTimer: comprimento invalido");
  }
}

void LapTimer::update(double time_s, double s_m) {
  if (!started_) {
    started_ = true;
    lap_start_time_s_ = time_s;
  } else if (last_s_m_ - s_m > 0.5 * track_length_m_) {  // s deu a volta: cruzou para a frente
    const double before = track_length_m_ - last_s_m_;
    const double crossing = last_time_s_ + (time_s - last_time_s_) * before / (before + s_m);
    if (reversed_ > 0) {
      --reversed_;
    } else {
      lap_times_s_.push_back(crossing - lap_start_time_s_);
      lap_start_time_s_ = crossing;
    }
  } else if (s_m - last_s_m_ > 0.5 * track_length_m_) {  // cruzou de re
    ++reversed_;
  }
  last_time_s_ = time_s;
  last_s_m_ = s_m;
}

}  // namespace sim
