#ifndef ECU_ECU_H
#define ECU_ECU_H

/* Firmware da ECU: C99, sem alocacao dinamica, um ecu_step por milissegundo.
 * Recebe os quadros do piloto e dos sensores, devolve o quadro de atuadores e, a cada
 * telemetry_period_steps, um quadro de telemetria.
 * Protecoes: pedais incoerentes (freio vence acelerador), corte suave de torque perto do
 * limite de giro e modo seguro (acelerador e freio em 0) se faltar o quadro do piloto. */

#include <stdint.h>

#include "can/can_frames.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
  ECU_FLAG_REV_LIMIT = 1,      /* acelerador reduzido pelo limite de giro */
  ECU_FLAG_BRAKE_OVERRIDE = 2, /* acelerador cortado porque o freio esta pisado */
  ECU_FLAG_DRIVER_LOST = 4     /* sem quadro valido do piloto neste passo */
};

typedef struct {
  double rpm_limit_start;           /* o corte comeca aqui... */
  double rpm_limit_end;             /* ...e zera o acelerador aqui */
  double brake_override_threshold;  /* freio e acelerador acima disto sao incoerentes */
  unsigned telemetry_period_steps;  /* 10 = 100 Hz */
  double upshift_rpm;               /* sobe a marcha acima disto... */
  double downshift_rpm;             /* ...e reduz abaixo disto */
  int gear_count;                   /* marchas do cambio */
} ecu_config_t;

typedef struct {
  ecu_config_t config;
  can_sensors_t sensors; /* ultimo quadro de sensores valido */
  uint32_t step_count;
  int gear_request;      /* marcha que a ECU pede ao carro */
} ecu_t;

ecu_config_t ecu_default_config(void);
/* Configuracao invalida (faixa de giro invertida, periodo 0) cai nos valores padrao. */
void ecu_init(ecu_t *ecu, ecu_config_t config);

/* driver e sensors podem ser NULL (quadro perdido). telemetry pode ser NULL.
 * Devolve 1 se escreveu um quadro de telemetria neste passo, 0 se nao. */
int ecu_step(ecu_t *ecu, const can_frame_t *driver, const can_frame_t *sensors,
             can_frame_t *actuators, can_frame_t *telemetry);

/* Quadro CAN_ID_GEAR com a marcha que a ECU decidiu no ultimo ecu_step: sobe acima de
 * upshift_rpm, reduz abaixo de downshift_rpm e senao mantem a marcha reportada pelos sensores. */
void ecu_gear_frame(const ecu_t *ecu, can_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ECU_ECU_H */
