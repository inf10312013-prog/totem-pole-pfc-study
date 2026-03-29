#ifndef DQ_PLL_H
#define DQ_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float a;
    float b;
    float c;
} abc_frame_t;

typedef struct
{
    float alpha;
    float beta;
} alphabeta_frame_t;

typedef struct
{
    float d;
    float q;
} dq_frame_t;

typedef struct
{
    float kp;
    float ki;
    float ts;          // control period [s]
    float w_nom;       // nominal angular frequency [rad/s], e.g. 2*pi*50
    float w_min;       // minimum allowed angular frequency
    float w_max;       // maximum allowed angular frequency

    float theta;       // estimated angle [rad]
    float omega;       // estimated angular frequency [rad/s]
    float integ;       // PI integrator state

    float vd;          // last d-axis value
    float vq;          // last q-axis value
} srf_pll_t;

void clarke_transform(const abc_frame_t *in, alphabeta_frame_t *out);
void park_transform(const alphabeta_frame_t *in, float theta, dq_frame_t *out);
void inv_park_transform(const dq_frame_t *in, float theta, alphabeta_frame_t *out);

void srf_pll_init(srf_pll_t *pll,
                  float kp,
                  float ki,
                  float ts,
                  float w_nom,
                  float w_min,
                  float w_max);

void srf_pll_update(srf_pll_t *pll, const abc_frame_t *v_abc);

#ifdef __cplusplus
}
#endif

#endif