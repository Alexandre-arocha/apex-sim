#include "ecu/ecu.h"

#include <stddef.h>

static double clamp(double v, double lo, double hi) {
  if (!(v > lo)) return lo; /* tambem pega NaN */
  return v < hi ? v : hi;
}

ecu_config_t ecu_default_config(void) {
  ecu_config_t c;
  c.rpm_limit_start = 14500.0;
  c.rpm_limit_end = 15000.0;
  c.brake_override_threshold = 0.05;
  c.telemetry_period_steps = 10u;
  c.upshift_rpm = 14000.0;
  c.downshift_rpm = 8500.0;
  c.gear_count = 8;
  return c;
}

void ecu_init(ecu_t *ecu, ecu_config_t config) {
  const ecu_config_t defaults = ecu_default_config();
  if (!(config.rpm_limit_end > config.rpm_limit_start && config.rpm_limit_start > 0.0)) {
    config.rpm_limit_start = defaults.rpm_limit_start;
    config.rpm_limit_end = defaults.rpm_limit_end;
  }
  if (!(config.brake_override_threshold >= 0.0)) {
    config.brake_override_threshold = defaults.brake_override_threshold;
  }
  if (config.telemetry_period_steps == 0u) {
    config.telemetry_period_steps = defaults.telemetry_period_steps;
  }
  if (!(config.downshift_rpm > 0.0 && config.upshift_rpm > config.downshift_rpm &&
        config.upshift_rpm < config.rpm_limit_start && config.gear_count >= 1)) {
    config.upshift_rpm = defaults.upshift_rpm;
    config.downshift_rpm = defaults.downshift_rpm;
    config.gear_count = defaults.gear_count;
  }
  ecu->config = config;
  ecu->gear_request = 1;
  ecu->sensors.speed_mps = 0.0;
  ecu->sensors.rpm = 0.0;
  ecu->sensors.gear = 1;
  ecu->step_count = 0u;
}

void ecu_gear_frame(const ecu_t *ecu, can_frame_t *out) { can_gear_pack(ecu->gear_request, out); }

int ecu_step(ecu_t *ecu, const can_frame_t *driver, const can_frame_t *sensors,
             can_frame_t *actuators, can_frame_t *telemetry) {
  const ecu_config_t *cfg = &ecu->config;
  can_command_t cmd = {0.0, 0.0, 0.0, 0u};
  can_sensors_t fresh;
  uint8_t flags = 0u;

  if (driver == NULL || can_command_unpack(CAN_ID_DRIVER, driver, &cmd) != 0) {
    cmd.throttle = 0.0; /* modo seguro: sem piloto, sem torque e sem freio */
    cmd.brake = 0.0;
    cmd.steer = 0.0;
    flags |= ECU_FLAG_DRIVER_LOST;
  }
  if (sensors != NULL && can_sensors_unpack(sensors, &fresh) == 0) ecu->sensors = fresh;

  cmd.throttle = clamp(cmd.throttle, 0.0, 1.0);
  cmd.brake = clamp(cmd.brake, 0.0, 1.0);
  cmd.steer = clamp(cmd.steer, -1.0, 1.0);

  if (cmd.brake > cfg->brake_override_threshold && cmd.throttle > cfg->brake_override_threshold) {
    cmd.throttle = 0.0;
    flags |= ECU_FLAG_BRAKE_OVERRIDE;
  }
  if (ecu->sensors.rpm > cfg->rpm_limit_start) {
    cmd.throttle *= clamp((cfg->rpm_limit_end - ecu->sensors.rpm) /
                              (cfg->rpm_limit_end - cfg->rpm_limit_start),
                          0.0, 1.0);
    flags |= ECU_FLAG_REV_LIMIT;
  }
  cmd.flags = flags;
  can_command_pack(CAN_ID_ACTUATORS, &cmd, actuators);

  {
    int gear = ecu->sensors.gear;
    if (gear < 1) gear = 1;
    if (gear > cfg->gear_count) gear = cfg->gear_count;
    if (ecu->sensors.rpm > cfg->upshift_rpm && gear < cfg->gear_count) {
      gear += 1;
    } else if (ecu->sensors.rpm < cfg->downshift_rpm && gear > 1) {
      gear -= 1;
    }
    ecu->gear_request = gear;
  }

  ecu->step_count += 1u;
  if (telemetry != NULL && ecu->step_count % cfg->telemetry_period_steps == 0u) {
    can_telemetry_t t;
    t.speed_mps = ecu->sensors.speed_mps;
    t.rpm = ecu->sensors.rpm;
    t.throttle = cmd.throttle;
    t.brake = cmd.brake;
    t.gear = ecu->sensors.gear;
    t.flags = flags;
    can_telemetry_pack(&t, telemetry);
    return 1;
  }
  return 0;
}
