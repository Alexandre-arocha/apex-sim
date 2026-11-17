#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "can/can_frames.h"
#include "ecu/ecu.h"
#include "sim/driver.hpp"
#include "sim/ecu_loop.hpp"
#include "sim/track.hpp"
#include "sim/vehicle.hpp"

using sim::Controls;
using sim::Driver;
using sim::EcuLoop;
using sim::LapTimer;
using sim::Track;
using sim::Vehicle;
using sim::VehicleParams;

namespace {

// Um passo da ECU com quadros montados a partir de valores; nullptr = quadro perdido.
struct EcuResult {
  can_command_t actuators{};
  can_telemetry_t telemetry{};
  bool has_telemetry = false;
};

EcuResult step_ecu(ecu_t& ecu, const can_command_t* driver, const can_sensors_t* sensors) {
  can_frame_t driver_frame;
  can_frame_t sensors_frame;
  can_frame_t actuators_frame;
  can_frame_t telemetry_frame;
  if (driver != nullptr) can_command_pack(CAN_ID_DRIVER, driver, &driver_frame);
  if (sensors != nullptr) can_sensors_pack(sensors, &sensors_frame);
  EcuResult result;
  result.has_telemetry =
      ecu_step(&ecu, driver != nullptr ? &driver_frame : nullptr,
               sensors != nullptr ? &sensors_frame : nullptr, &actuators_frame, &telemetry_frame) !=
      0;
  EXPECT_EQ(can_command_unpack(CAN_ID_ACTUATORS, &actuators_frame, &result.actuators), 0);
  if (result.has_telemetry) {
    EXPECT_EQ(can_telemetry_unpack(&telemetry_frame, &result.telemetry), 0);
  }
  return result;
}

ecu_t make_ecu() {
  ecu_t ecu;
  ecu_init(&ecu, ecu_default_config());
  return ecu;
}

struct LapResult {
  std::vector<double> lap_times_s;
  std::vector<can_frame_t> frames;
  std::uint64_t steps = 0;
  double top_speed_mps = 0.0;
  double max_rpm = 0.0;
  double bus_utilization = 0.0;
  std::uint64_t retransmissions = 0;
};

LapResult run_laps(bool with_ecu, int laps, sim::CanBusConfig bus = {}) {
  const VehicleParams params;
  const Track track = Track::reference();
  Vehicle car{params};
  Driver driver{track, params};
  LapTimer timer{track.length_m()};
  EcuLoop ecu{ecu_default_config(), bus};
  LapResult result;
  while (timer.lap_times_s().size() < static_cast<std::size_t>(laps) &&
         car.state().time_s < 200.0) {
    const Controls wanted = driver.drive(car.state());
    timer.update(car.state().time_s, driver.position().s_m);
    result.top_speed_mps = std::max(result.top_speed_mps, car.state().speed_mps);
    result.max_rpm = std::max(result.max_rpm, car.state().engine_rpm);
    car.step(with_ecu ? ecu.step(car.state(), wanted) : wanted);
  }
  result.lap_times_s = timer.lap_times_s();
  result.steps = car.state().step_count;
  result.frames = ecu.telemetry_frames();
  result.bus_utilization = ecu.bus().utilization();
  result.retransmissions = ecu.bus().stats().retransmissions;
  return result;
}

}  // namespace

TEST(CanFrames, SensorsLayoutAndRoundTrip) {
  const can_sensors_t in{123.456, 14321.4, 7};
  can_frame_t frame;
  can_sensors_pack(&in, &frame);
  EXPECT_EQ(frame.id, static_cast<std::uint32_t>(CAN_ID_SENSORS));
  EXPECT_EQ(frame.dlc, 5);
  EXPECT_EQ(frame.data[0], 12346 & 0xFF);  // little-endian, 0,01 m/s
  EXPECT_EQ(frame.data[1], 12346 >> 8);
  EXPECT_EQ(frame.data[4], 7);
  can_sensors_t out;
  ASSERT_EQ(can_sensors_unpack(&frame, &out), 0);
  EXPECT_NEAR(out.speed_mps, 123.456, 0.005);
  EXPECT_NEAR(out.rpm, 14321.4, 0.5);
  EXPECT_EQ(out.gear, 7);
}

TEST(CanFrames, CommandRoundTripSharedByDriverAndActuators) {
  const can_command_t in{0.75, 0.25, -0.5, 3};
  for (const std::uint32_t id : {static_cast<std::uint32_t>(CAN_ID_DRIVER),
                                 static_cast<std::uint32_t>(CAN_ID_ACTUATORS)}) {
    can_frame_t frame;
    can_command_pack(id, &in, &frame);
    can_command_t out;
    ASSERT_EQ(can_command_unpack(id, &frame, &out), 0);
    EXPECT_NEAR(out.throttle, 0.75, 1e-4);
    EXPECT_NEAR(out.brake, 0.25, 1e-4);
    EXPECT_NEAR(out.steer, -0.5, 1e-4);
    EXPECT_EQ(out.flags, 3);
  }
}

TEST(CanFrames, TelemetryRoundTrip) {
  const can_telemetry_t in{88.88, 9876.0, 0.5, 0.2, 4, 2};
  can_frame_t frame;
  can_telemetry_pack(&in, &frame);
  EXPECT_EQ(frame.id, static_cast<std::uint32_t>(CAN_ID_TELEMETRY));
  EXPECT_EQ(frame.dlc, 8);
  can_telemetry_t out;
  ASSERT_EQ(can_telemetry_unpack(&frame, &out), 0);
  EXPECT_NEAR(out.speed_mps, 88.88, 0.005);
  EXPECT_NEAR(out.rpm, 9876.0, 0.5);
  EXPECT_NEAR(out.throttle, 0.5, 0.002);
  EXPECT_NEAR(out.brake, 0.2, 0.002);
  EXPECT_EQ(out.gear, 4);
  EXPECT_EQ(out.flags, 2);
}

TEST(CanFrames, OutOfRangeValuesSaturateAndNanBecomesZero) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  can_frame_t frame;
  can_command_t out;
  const can_command_t huge{9.0, -1.0, -9.0, 0};
  can_command_pack(CAN_ID_DRIVER, &huge, &frame);
  ASSERT_EQ(can_command_unpack(CAN_ID_DRIVER, &frame, &out), 0);
  EXPECT_NEAR(out.throttle, 6.5535, 1e-9);  // satura em 65535 * 1e-4
  EXPECT_DOUBLE_EQ(out.brake, 0.0);
  EXPECT_NEAR(out.steer, -3.2768, 1e-9);
  const can_command_t broken{nan, nan, nan, 0};
  can_command_pack(CAN_ID_DRIVER, &broken, &frame);
  ASSERT_EQ(can_command_unpack(CAN_ID_DRIVER, &frame, &out), 0);
  EXPECT_DOUBLE_EQ(out.throttle, 0.0);
  EXPECT_DOUBLE_EQ(out.steer, 0.0);
}

TEST(CanFrames, UnpackRejectsWrongIdOrLength) {
  can_frame_t frame;
  const can_command_t cmd{0.5, 0.0, 0.0, 0};
  can_command_pack(CAN_ID_DRIVER, &cmd, &frame);
  can_command_t out;
  EXPECT_EQ(can_command_unpack(CAN_ID_ACTUATORS, &frame, &out), -1);
  frame.dlc = 6;
  EXPECT_EQ(can_command_unpack(CAN_ID_DRIVER, &frame, &out), -1);
  can_sensors_t sensors;
  EXPECT_EQ(can_sensors_unpack(&frame, &sensors), -1);
  can_telemetry_t telemetry;
  EXPECT_EQ(can_telemetry_unpack(&frame, &telemetry), -1);
}

TEST(Ecu, PassesPlausibleCommandsThrough) {
  ecu_t ecu = make_ecu();
  const can_command_t driver{0.6, 0.0, 0.2, 0};
  const can_sensors_t sensors{50.0, 8000.0, 3};
  const EcuResult r = step_ecu(ecu, &driver, &sensors);
  EXPECT_NEAR(r.actuators.throttle, 0.6, 1e-4);
  EXPECT_NEAR(r.actuators.steer, 0.2, 1e-4);
  EXPECT_EQ(r.actuators.flags, 0);
}

TEST(Ecu, BrakeWinsOverThrottle) {
  ecu_t ecu = make_ecu();
  const can_sensors_t sensors{50.0, 8000.0, 3};
  const can_command_t both{0.6, 0.5, 0.0, 0};
  const EcuResult r = step_ecu(ecu, &both, &sensors);
  EXPECT_DOUBLE_EQ(r.actuators.throttle, 0.0);
  EXPECT_NEAR(r.actuators.brake, 0.5, 1e-4);
  EXPECT_EQ(r.actuators.flags, ECU_FLAG_BRAKE_OVERRIDE);
  const can_command_t touching{0.6, 0.02, 0.0, 0};  // abaixo do limiar: nao e incoerente
  EXPECT_NEAR(step_ecu(ecu, &touching, &sensors).actuators.throttle, 0.6, 1e-4);
}

TEST(Ecu, CutsThrottleSmoothlyNearTheRevLimit) {
  ecu_t ecu = make_ecu();
  const can_command_t full{1.0, 0.0, 0.0, 0};
  const auto throttle_at = [&](double rpm) {
    const can_sensors_t sensors{80.0, rpm, 8};
    return step_ecu(ecu, &full, &sensors).actuators;
  };
  EXPECT_NEAR(throttle_at(14000.0).throttle, 1.0, 1e-4);
  EXPECT_EQ(throttle_at(14000.0).flags, 0);
  const can_command_t mid = throttle_at(14750.0);  // meio da rampa de 14500 a 15000
  EXPECT_NEAR(mid.throttle, 0.5, 1e-4);
  EXPECT_EQ(mid.flags, ECU_FLAG_REV_LIMIT);
  EXPECT_DOUBLE_EQ(throttle_at(15000.0).throttle, 0.0);
  EXPECT_DOUBLE_EQ(throttle_at(16000.0).throttle, 0.0);
}

TEST(Ecu, LimitUsesTheLastSensorFrameWhenOneIsLost) {
  ecu_t ecu = make_ecu();
  const can_command_t full{1.0, 0.0, 0.0, 0};
  const can_sensors_t high{80.0, 14750.0, 8};
  static_cast<void>(step_ecu(ecu, &full, &high));
  EXPECT_NEAR(step_ecu(ecu, &full, nullptr).actuators.throttle, 0.5, 1e-4);
}

TEST(Ecu, GoesSafeWhenTheDriverFrameIsMissingOrInvalid) {
  ecu_t ecu = make_ecu();
  const can_sensors_t sensors{50.0, 8000.0, 3};
  const EcuResult missing = step_ecu(ecu, nullptr, &sensors);
  EXPECT_DOUBLE_EQ(missing.actuators.throttle, 0.0);
  EXPECT_DOUBLE_EQ(missing.actuators.brake, 0.0);
  EXPECT_DOUBLE_EQ(missing.actuators.steer, 0.0);
  EXPECT_EQ(missing.actuators.flags, ECU_FLAG_DRIVER_LOST);

  // Quadro com id de outro emissor (o dos atuadores) tambem nao vale como piloto.
  const can_command_t cmd{1.0, 0.0, 0.5, 0};
  can_frame_t wrong;
  can_command_pack(CAN_ID_ACTUATORS, &cmd, &wrong);
  can_frame_t sensors_frame;
  can_sensors_pack(&sensors, &sensors_frame);
  can_frame_t actuators_frame;
  static_cast<void>(ecu_step(&ecu, &wrong, &sensors_frame, &actuators_frame, nullptr));
  can_command_t out;
  ASSERT_EQ(can_command_unpack(CAN_ID_ACTUATORS, &actuators_frame, &out), 0);
  EXPECT_DOUBLE_EQ(out.throttle, 0.0);
  EXPECT_EQ(out.flags, ECU_FLAG_DRIVER_LOST);
}

TEST(Ecu, ClampsOutOfRangeDriverCommands) {
  ecu_t ecu = make_ecu();
  const can_sensors_t sensors{50.0, 8000.0, 3};
  const can_command_t wild{9.0, 0.0, -9.0, 0};
  const EcuResult r = step_ecu(ecu, &wild, &sensors);
  EXPECT_NEAR(r.actuators.throttle, 1.0, 1e-4);
  EXPECT_NEAR(r.actuators.steer, -1.0, 1e-4);
}

TEST(Ecu, EmitsTelemetryEveryTenthStep) {
  ecu_t ecu = make_ecu();
  const can_command_t driver{0.4, 0.0, 0.0, 0};
  const can_sensors_t sensors{12.34, 9000.0, 3};
  int frames = 0;
  for (int i = 1; i <= 1000; ++i) {
    const EcuResult r = step_ecu(ecu, &driver, &sensors);
    EXPECT_EQ(r.has_telemetry, i % 10 == 0);
    if (r.has_telemetry) {
      ++frames;
      EXPECT_NEAR(r.telemetry.speed_mps, 12.34, 0.005);
      EXPECT_NEAR(r.telemetry.rpm, 9000.0, 0.5);
      EXPECT_NEAR(r.telemetry.throttle, 0.4, 0.002);
      EXPECT_EQ(r.telemetry.gear, 3);
    }
  }
  EXPECT_EQ(frames, 100);
}

TEST(Ecu, InvalidConfigFallsBackToDefaults) {
  // Faixa de giro invertida, limiar, periodo e cambio invalidos.
  ecu_config_t bad{15000.0, 14000.0, -1.0, 0u, 9000.0, 12000.0, 0};
  ecu_t ecu;
  ecu_init(&ecu, bad);
  const ecu_config_t defaults = ecu_default_config();
  EXPECT_DOUBLE_EQ(ecu.config.rpm_limit_start, defaults.rpm_limit_start);
  EXPECT_DOUBLE_EQ(ecu.config.rpm_limit_end, defaults.rpm_limit_end);
  EXPECT_DOUBLE_EQ(ecu.config.brake_override_threshold, defaults.brake_override_threshold);
  EXPECT_EQ(ecu.config.telemetry_period_steps, defaults.telemetry_period_steps);
  EXPECT_DOUBLE_EQ(ecu.config.upshift_rpm, defaults.upshift_rpm);
  EXPECT_EQ(ecu.config.gear_count, defaults.gear_count);
}

TEST(CanFrames, GearFrameRoundTripAndSaturation) {
  can_frame_t frame;
  can_gear_pack(5, &frame);
  EXPECT_EQ(frame.id, static_cast<std::uint32_t>(CAN_ID_GEAR));
  EXPECT_EQ(frame.dlc, 1);
  EXPECT_EQ(frame.data[0], 5);
  int gear = -1;
  ASSERT_EQ(can_gear_unpack(&frame, &gear), 0);
  EXPECT_EQ(gear, 5);
  can_gear_pack(300, &frame);
  ASSERT_EQ(can_gear_unpack(&frame, &gear), 0);
  EXPECT_EQ(gear, 255);
  frame.id = CAN_ID_SENSORS;
  EXPECT_EQ(can_gear_unpack(&frame, &gear), -1);
}

TEST(Ecu, ChoosesTheGear) {
  ecu_t ecu = make_ecu();
  const can_command_t driver{1.0, 0.0, 0.0, 0};
  const auto request_at = [&](int gear, double rpm) {
    const can_sensors_t sensors{80.0, rpm, gear};
    static_cast<void>(step_ecu(ecu, &driver, &sensors));
    can_frame_t frame;
    ecu_gear_frame(&ecu, &frame);
    int requested = -1;
    EXPECT_EQ(can_gear_unpack(&frame, &requested), 0);
    return requested;
  };
  EXPECT_EQ(request_at(3, 14100.0), 4);   // sobe
  EXPECT_EQ(request_at(5, 8000.0), 4);    // reduz
  EXPECT_EQ(request_at(4, 11000.0), 4);   // mantem
  EXPECT_EQ(request_at(8, 14900.0), 8);   // ultima marcha: nao passa dela
  EXPECT_EQ(request_at(1, 5000.0), 1);    // primeira: nao reduz
}

TEST(EcuLoop, LapsWithTheEcuInTheLoopMatchLapsWithout) {
  const LapResult plain = run_laps(false, 2);
  const LapResult with_ecu = run_laps(true, 2);
  ASSERT_EQ(plain.lap_times_s.size(), 2u);
  ASSERT_EQ(with_ecu.lap_times_s.size(), 2u);
  EXPECT_NEAR(with_ecu.lap_times_s[1], plain.lap_times_s[1], 0.01 * plain.lap_times_s[1]);
  // A ECU decide a marcha com um passo de atraso, o que acrescenta alguns rpm; o limite que
  // importa e o do motor.
  EXPECT_LE(with_ecu.max_rpm, 15000.0);
  EXPECT_NEAR(with_ecu.max_rpm, plain.max_rpm, 50.0);
}

TEST(EcuLoop, TheBusCarriesTheOneKilohertzLoadWithRoomToSpare) {
  const LapResult r = run_laps(true, 1);
  // Por passo: sensores 105 + piloto 125 + atuadores 125 + marcha 65 bits, e telemetria (135) a cada
  // 10 passos: ~433 bits em cada 1000 (1 Mbit/s).
  EXPECT_GT(r.bus_utilization, 0.40);
  EXPECT_LT(r.bus_utilization, 0.47);
  EXPECT_EQ(r.retransmissions, 0u);
}

TEST(EcuLoop, LapsSurviveACorruptingBus) {
  sim::CanBusConfig noisy_bus;
  noisy_bus.frame_error_rate = 0.02;
  noisy_bus.seed = 3;
  const LapResult clean = run_laps(true, 2);
  const LapResult noisy = run_laps(true, 2, noisy_bus);
  ASSERT_EQ(noisy.lap_times_s.size(), 2u);
  EXPECT_GT(noisy.retransmissions, 1000u);
  EXPECT_NEAR(noisy.lap_times_s[1], clean.lap_times_s[1], 0.02 * clean.lap_times_s[1]);
  EXPECT_GE(noisy.frames.size() + 1, noisy.steps / 10);
  for (const can_frame_t& frame : noisy.frames) {  // o CRC barrou todo quadro corrompido
    can_telemetry_t t;
    ASSERT_EQ(can_telemetry_unpack(&frame, &t), 0);
  }
}

TEST(EcuLoop, EmitsTelemetryAt100HzThatTracksTheCar) {
  const LapResult r = run_laps(true, 1);
  ASSERT_EQ(r.frames.size(), r.steps / 10);
  double top = 0.0;
  for (const can_frame_t& frame : r.frames) {
    can_telemetry_t t;
    ASSERT_EQ(can_telemetry_unpack(&frame, &t), 0);
    EXPECT_EQ(t.flags & ECU_FLAG_DRIVER_LOST, 0);
    EXPECT_GE(t.gear, 1);
    EXPECT_LE(t.gear, 8);
    top = std::max(top, t.speed_mps);
  }
  EXPECT_NEAR(top, r.top_speed_mps, 1.0);
}
