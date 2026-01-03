/*
 * MESCposition.h
 *
 *  Created on: Feb 14, 2023
 *      Author: HPEnvy
 */


#ifndef MESC_POSITION_H
#define MESC_POSITION_H
#include "MESCfoc.h"


void RunPosControl(MESC_motor_typedef *_motor);

static inline void UpdatePositionMultiTurn(MESC_motor_typedef* m, uint16_t enc)
{
    int32_t d = (int32_t)enc - (int32_t)m->position_ctrl.enc_last;

    if (d >  32768) m->position_ctrl.rev_count--;   // wrapped backward
    if (d < -32768) m->position_ctrl.rev_count++;   // wrapped forward

    m->position_ctrl.enc_last = enc;
    m->position_ctrl.pos_abs  = (m->position_ctrl.rev_count << 16) + (int32_t)enc; // counts
}


static inline float CountsPerSec_To_eHz(float cps, float pole_pairs)
{
    return (cps / 65536.0f) * pole_pairs;
}



static inline void PositionTrajectoryStep(MESC_motor_typedef* m, float dt)
{
    // error in counts
    float err = (float)(m->position_ctrl.pos_target - m->position_ctrl.pos_abs);

    // stopping distance in counts: v^2 / (2a)
    float v  = m->position_ctrl.vel_sp;
    float a  = m->position_ctrl.acc_limit;
    float d_stop = (v*v) / (2.0f * a + 1e-9f);

    // choose accel sign
    float accel = 0.0f;
    if (fabsf(err) <= d_stop) {
        // decelerate toward 0
        accel = (v > 0.0f) ? -a : +a;
    } else {
        // accelerate toward target
        accel = (err > 0.0f) ? +a : -a;
    }

    // integrate velocity, clamp
    v += accel * dt;
    if (v >  m->position_ctrl.vel_limit) v =  m->position_ctrl.vel_limit;
    if (v < -m->position_ctrl.vel_limit) v = -m->position_ctrl.vel_limit;

    // if very close, settle
    if (fabsf(err) < 5.0f && fabsf(v) < 20.0f) { // tweak thresholds
        v = 0.0f;
    }

    m->position_ctrl.vel_sp = v;

    // Convert to speed_req used by your speed controller (electrical Hz)
    float eHz_ff = CountsPerSec_To_eHz(v, (float)m->m.pole_pairs);

    // Optional: small P correction on position error (adds stiffness)
    float eHz_p  = m->position_ctrl.pos_kp * err;  // pos_kp should be tuned small

    m->FOC.speed_req = eHz_ff + eHz_p;
}



#endif
