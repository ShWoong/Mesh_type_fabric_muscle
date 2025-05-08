#ifndef MA100BF103A_H
#define MA100BF103A_H

#include <stdint.h>   /* uint16_t 정의용 */

/* ───── Coefficient and constant ───── */
extern const float MA100BF_A;
extern const float MA100BF_B;
extern const float MA100BF_C;

extern const float MA100BF_R1;
extern const float MA100BF_ADC_RES;

float thermistor_celsius    (uint16_t adc_raw);

#endif /* MA100BF103A_H */
