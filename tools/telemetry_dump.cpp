// Roda o piloto virtual com a ECU no laco e grava a telemetria que ela emite no formato de fio do
// servidor C# (ver server/SimTelemetry/CanFrame.cs): registros de 13 bytes, id uint32
// little-endian, dlc, 8 bytes de dados.
//   uso: sim_telemetry_dump <arquivo-de-saida|-> [voltas] [host:porta|broker:host:porta]
// Com host:porta, manda tambem cada quadro ao servidor de telemetria por TCP, a medida que a ECU o
// emite. Com broker:host:porta, publica no broker pub/sub (Protobuf): uma SessionStart e depois uma
// TelemetrySample por quadro, numerada a partir de 0. "-" no lugar do arquivo nao grava arquivo.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "sim/broker_publisher.hpp"
#include "sim/driver.hpp"
#include "sim/ecu_loop.hpp"
#include "sim/telemetry_client.hpp"
#include "sim/track.hpp"
#include "sim/vehicle.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "uso: %s <arquivo-de-saida|-> [voltas] [host:porta|broker:host:porta]\n", argv[0]);
    return 2;
  }
  const long laps = argc > 2 ? std::strtol(argv[2], nullptr, 10) : 2;
  if (laps < 1 || laps > 100) {
    std::fprintf(stderr, "voltas deve estar entre 1 e 100\n");
    return 2;
  }

  const sim::VehicleParams params;
  const sim::Track track = sim::Track::reference();

  sim::TelemetryClient client;
  sim::BrokerPublisher broker;
  if (argc > 3) {
    std::string target = argv[3];
    const bool via_broker = target.rfind("broker:", 0) == 0;
    if (via_broker) target.erase(0, 7);
    const std::size_t colon = target.rfind(':');
    const long port = colon == std::string::npos ? 0 : std::strtol(target.c_str() + colon + 1, nullptr, 10);
    const bool valid = colon != std::string::npos && port >= 1 && port <= 65535;
    const std::string host = valid ? target.substr(0, colon) : std::string();
    const auto port16 = static_cast<std::uint16_t>(port);
    if (!valid || !(via_broker ? broker.connect(host, port16) : client.connect(host, port16))) {
      std::fprintf(stderr, "nao consegui conectar a %s\n", argv[3]);
      return 1;
    }
    if (via_broker && !broker.publish(sim::kTopicSessionStart,
                                      sim::encode_session_start("sim_telemetry_dump", track.length_m()))) {
      std::fprintf(stderr, "a conexao com o broker caiu\n");
      return 1;
    }
  }

  sim::Vehicle car{params};
  sim::Driver driver{track, params};
  sim::LapTimer timer{track.length_m()};
  sim::EcuLoop ecu;
  double top_speed = 0.0;
  std::size_t sent = 0;
  std::size_t published = 0;
  while (timer.lap_times_s().size() < static_cast<std::size_t>(laps) &&
         car.state().time_s < 60.0 * static_cast<double>(laps) + 60.0) {
    const sim::Controls wanted = driver.drive(car.state());
    timer.update(car.state().time_s, driver.position().s_m);
    top_speed = std::max(top_speed, car.state().speed_mps);
    car.step(ecu.step(car.state(), wanted));
    for (; client.connected() && sent < ecu.telemetry_frames().size(); ++sent) {
      if (!client.send(ecu.telemetry_frames()[sent])) {
        std::fprintf(stderr, "a conexao com o servidor caiu\n");
        return 1;
      }
    }
    for (; broker.connected() && published < ecu.telemetry_frames().size(); ++published) {
      can_telemetry_t telemetry;
      if (can_telemetry_unpack(&ecu.telemetry_frames()[published], &telemetry) != 0 ||
          !broker.publish(sim::kTopicSample, sim::encode_sample(published, telemetry))) {
        std::fprintf(stderr, "a conexao com o broker caiu\n");
        return 1;
      }
    }
  }

  const bool to_file = std::string(argv[1]) != "-";
  std::ofstream out;
  if (to_file) out.open(argv[1], std::ios::binary);
  if (to_file && !out) {
    std::fprintf(stderr, "nao consegui abrir %s\n", argv[1]);
    return 1;
  }
  for (const can_frame_t& frame : ecu.telemetry_frames()) {
    if (!to_file) break;
    std::array<char, 13> record{};
    for (std::size_t i = 0; i < 4; ++i) {
      record[i] = static_cast<char>((frame.id >> (8 * i)) & 0xFFu);
    }
    record[4] = static_cast<char>(frame.dlc);
    for (std::size_t i = 0; i < 8; ++i) record[5 + i] = static_cast<char>(frame.data[i]);
    out.write(record.data(), static_cast<std::streamsize>(record.size()));
  }
  out.close();
  if (to_file && !out) {
    std::fprintf(stderr, "erro ao gravar %s\n", argv[1]);
    return 1;
  }

  std::printf("pista_m=%.4f quadros=%zu passos=%llu vmax_kmh=%.2f voltas_s=", track.length_m(),
              ecu.telemetry_frames().size(),
              static_cast<unsigned long long>(car.state().step_count), top_speed * 3.6);
  for (const double t : timer.lap_times_s()) std::printf("%.3f ", t);
  std::printf("\n");
  return 0;
}
