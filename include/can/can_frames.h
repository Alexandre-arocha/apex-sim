#ifndef CAN_CAN_FRAMES_H
#define CAN_CAN_FRAMES_H

/* Quadros CAN simulados entre fisica, piloto e ECU. Little-endian, 8 bytes no maximo.
 * Cada quadro tem pack/unpack; unpack devolve 0 se ok e -1 se id ou dlc nao baterem.
 * Valores fora da faixa saturam; NaN vira 0. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  CAN_ID_SENSORS = 0x100,    /* fisica -> ECU, 1 kHz */
  CAN_ID_DRIVER = 0x200,     /* piloto -> ECU, 1 kHz */
  CAN_ID_ACTUATORS = 0x300,  /* ECU -> fisica, 1 kHz */
  CAN_ID_GEAR = 0x310,       /* ECU -> fisica, 1 kHz: marcha pedida */
  CAN_ID_TELEMETRY = 0x400   /* ECU -> box, 100 Hz */
};

typedef struct {
  uint32_t id;      /* 11 bits */
  uint8_t dlc;
  uint8_t data[8];
  uint16_t crc;     /* CRC-15 do CAN sobre id, dlc e dados; preenchido por *_pack */
} can_frame_t;

/* CRC-15/CAN (polinomio 0x4599) de um quadro padrao de 11 bits: SOF, id, RTR, IDE, r0, DLC e os
 * dados, como no padrao. can_frame_crc_ok devolve 1 se f->crc confere. Todo *_unpack rejeita (-1)
 * um quadro cujo CRC nao confere. */
uint16_t can_frame_crc(const can_frame_t *f);
int can_frame_crc_ok(const can_frame_t *f);

/* SENSORS (dlc 5): speed u16 @0,01 m/s | rpm u16 @1 | gear u8 */
typedef struct {
  double speed_mps;
  double rpm;
  int gear;
} can_sensors_t;

/* DRIVER e ACTUATORS (dlc 7): throttle u16 @1e-4 | brake u16 @1e-4 | steer i16 @1e-4 | flags u8 */
typedef struct {
  double throttle; /* [0, 1] */
  double brake;    /* [0, 1] */
  double steer;    /* [-1, 1], positivo = esquerda */
  uint8_t flags;   /* bits de ECU_FLAG_* (0 no quadro do piloto) */
} can_command_t;

/* TELEMETRY (dlc 8): speed u16 @0,01 | rpm u16 @1 | throttle u8 @0,4% | brake u8 @0,4% |
 * gear u8 | flags u8 */
typedef struct {
  double speed_mps;
  double rpm;
  double throttle;
  double brake;
  int gear;
  uint8_t flags;
} can_telemetry_t;

void can_sensors_pack(const can_sensors_t *in, can_frame_t *out);
int can_sensors_unpack(const can_frame_t *in, can_sensors_t *out);

/* id e CAN_ID_DRIVER ou CAN_ID_ACTUATORS: o mesmo formato serve aos dois. */
void can_command_pack(uint32_t id, const can_command_t *in, can_frame_t *out);
int can_command_unpack(uint32_t id, const can_frame_t *in, can_command_t *out);

/* GEAR (dlc 1): marcha pedida u8 (0 = deixa o cambio automatico do carro). */
void can_gear_pack(int gear, can_frame_t *out);
int can_gear_unpack(const can_frame_t *in, int *gear);

void can_telemetry_pack(const can_telemetry_t *in, can_frame_t *out);
int can_telemetry_unpack(const can_frame_t *in, can_telemetry_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CAN_CAN_FRAMES_H */
