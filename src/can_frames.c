#include "can/can_frames.h"

#include <math.h>
#include <string.h>

static uint16_t encode_u16(double value, double lsb) {
  const double x = value / lsb + 0.5;
  if (!(x > 0.0)) return 0u; /* negativo ou NaN */
  if (x >= 65535.0) return 65535u;
  return (uint16_t)x;
}

static uint8_t encode_u8(double value, double lsb) {
  const double x = value / lsb + 0.5;
  if (!(x > 0.0)) return 0u;
  if (x >= 255.0) return 255u;
  return (uint8_t)x;
}

static int16_t encode_i16(double value, double lsb) {
  const double x = value / lsb;
  if (isnan(x)) return 0;
  if (x >= 32767.0) return 32767;
  if (x <= -32768.0) return -32768;
  return (int16_t)(x < 0.0 ? x - 0.5 : x + 0.5);
}

static void put_u16(uint8_t *d, uint16_t v) {
  d[0] = (uint8_t)(v & 0xFFu);
  d[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *d) { return (uint16_t)(d[0] | (d[1] << 8)); }

static uint16_t crc15_bit(uint16_t crc, unsigned bit) {
  const unsigned wide = crc;
  const unsigned feedback = ((wide >> 14) & 1u) ^ (bit & 1u);
  crc = (uint16_t)((wide << 1) & 0x7FFFu);
  return feedback != 0u ? (uint16_t)(crc ^ 0x4599u) : crc;
}

uint16_t can_frame_crc(const can_frame_t *f) {
  uint16_t crc = 0u;
  int i;
  unsigned b;
  crc = crc15_bit(crc, 0u); /* SOF */
  for (i = 10; i >= 0; --i) crc = crc15_bit(crc, (f->id >> i) & 1u);
  crc = crc15_bit(crc, 0u); /* RTR */
  crc = crc15_bit(crc, 0u); /* IDE */
  crc = crc15_bit(crc, 0u); /* r0 */
  for (i = 3; i >= 0; --i) crc = crc15_bit(crc, ((unsigned)f->dlc >> i) & 1u);
  for (b = 0u; b < f->dlc && b < 8u; ++b) {
    for (i = 7; i >= 0; --i) crc = crc15_bit(crc, ((unsigned)f->data[b] >> i) & 1u);
  }
  return crc;
}

int can_frame_crc_ok(const can_frame_t *f) { return f->crc == can_frame_crc(f); }

static int header_ok(const can_frame_t *f, uint32_t id, uint8_t dlc) {
  return f->id == id && f->dlc == dlc && can_frame_crc_ok(f);
}

static void begin(can_frame_t *f, uint32_t id, uint8_t dlc) {
  memset(f, 0, sizeof *f);
  f->id = id;
  f->dlc = dlc;
}

static void finish(can_frame_t *f) { f->crc = can_frame_crc(f); }

void can_sensors_pack(const can_sensors_t *in, can_frame_t *out) {
  begin(out, CAN_ID_SENSORS, 5u);
  put_u16(&out->data[0], encode_u16(in->speed_mps, 0.01));
  put_u16(&out->data[2], encode_u16(in->rpm, 1.0));
  out->data[4] = encode_u8((double)in->gear, 1.0);
  finish(out);
}

int can_sensors_unpack(const can_frame_t *in, can_sensors_t *out) {
  if (!header_ok(in, CAN_ID_SENSORS, 5u)) return -1;
  out->speed_mps = get_u16(&in->data[0]) * 0.01;
  out->rpm = (double)get_u16(&in->data[2]);
  out->gear = in->data[4];
  return 0;
}

void can_command_pack(uint32_t id, const can_command_t *in, can_frame_t *out) {
  begin(out, id, 7u);
  put_u16(&out->data[0], encode_u16(in->throttle, 1e-4));
  put_u16(&out->data[2], encode_u16(in->brake, 1e-4));
  put_u16(&out->data[4], (uint16_t)encode_i16(in->steer, 1e-4));
  out->data[6] = in->flags;
  finish(out);
}

int can_command_unpack(uint32_t id, const can_frame_t *in, can_command_t *out) {
  if (!header_ok(in, id, 7u)) return -1;
  out->throttle = get_u16(&in->data[0]) * 1e-4;
  out->brake = get_u16(&in->data[2]) * 1e-4;
  out->steer = (int16_t)get_u16(&in->data[4]) * 1e-4;
  out->flags = in->data[6];
  return 0;
}

void can_gear_pack(int gear, can_frame_t *out) {
  begin(out, CAN_ID_GEAR, 1u);
  out->data[0] = encode_u8((double)gear, 1.0);
  finish(out);
}

int can_gear_unpack(const can_frame_t *in, int *gear) {
  if (!header_ok(in, CAN_ID_GEAR, 1u)) return -1;
  *gear = in->data[0];
  return 0;
}

void can_telemetry_pack(const can_telemetry_t *in, can_frame_t *out) {
  begin(out, CAN_ID_TELEMETRY, 8u);
  put_u16(&out->data[0], encode_u16(in->speed_mps, 0.01));
  put_u16(&out->data[2], encode_u16(in->rpm, 1.0));
  out->data[4] = encode_u8(in->throttle, 0.004);
  out->data[5] = encode_u8(in->brake, 0.004);
  out->data[6] = encode_u8((double)in->gear, 1.0);
  out->data[7] = in->flags;
  finish(out);
}

int can_telemetry_unpack(const can_frame_t *in, can_telemetry_t *out) {
  if (!header_ok(in, CAN_ID_TELEMETRY, 8u)) return -1;
  out->speed_mps = get_u16(&in->data[0]) * 0.01;
  out->rpm = (double)get_u16(&in->data[2]);
  out->throttle = in->data[4] * 0.004;
  out->brake = in->data[5] * 0.004;
  out->gear = in->data[6];
  out->flags = in->data[7];
  return 0;
}
