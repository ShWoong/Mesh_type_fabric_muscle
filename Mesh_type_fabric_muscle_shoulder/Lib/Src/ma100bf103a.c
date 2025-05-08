#include "ma100bf103a.h"
#include <math.h>

const float MA100BF_A       = 1.111212003e-3f;
const float MA100BF_B       = 2.371339042e-4f;
const float MA100BF_C       = 0.7515578419e-7f;

const float MA100BF_R1      = 1.0e4f;     /* 10 kΩ */
const float MA100BF_ADC_RES = 4095.0f;    /* 12-bit */

/* ───── 내부 헬퍼 ───── */
static inline float calc_R_ntc(uint16_t adc_raw)
{
    if (adc_raw == 0 || adc_raw >= MA100BF_ADC_RES) return NAN;

    float v     = (float)adc_raw / MA100BF_ADC_RES;
    float r_ntc = MA100BF_R1 * v / (1.0f - v);

    return r_ntc;
}

float thermistor_celsius(uint16_t adc_raw)
{
    float r_ntc = calc_R_ntc(adc_raw);
    if (isnan(r_ntc)) return NAN;

    float lnR   = logf(r_ntc);
    float invT  = MA100BF_A + MA100BF_B*lnR + MA100BF_C*powf(lnR, 3.0f);

    return (1.0f / invT) - 273.15f;
}
