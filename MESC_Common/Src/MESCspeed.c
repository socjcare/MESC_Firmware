;/*
 * MESCposition.c
 *
 *  Created on: Feb 14, 2023
 *      Author: HPEnvy
 */


/* Includes ------------------------------------------------------------------*/
#include "MESCposition.h"
#include <math.h>

//void RunModifiedSpeedControl(MESC_motor_typedef *_motor){
//
//	    float speed_error;
//	    uint16_t align_counter;
//	    _motor->speed_ctrl_limits.debug_counter++;
//
//
//
//	    if (_motor->MotorState != MOTOR_STATE_RUN) {
//	        _motor->FOC.Idq_prereq.q = 0.0f;
//	        _motor->FOC.speed_error_int = 0.0f;
//	        _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
//	        return;
//	    }
//
//	    switch (_motor->speed_ctrl_state) {
//
//	    /* ---------------- IDLE ---------------- */
//	    case SPEED_CTRL_IDLE:
//
//	        _motor->FOC.Idq_prereq.q = 0.0f;
//	        _motor->FOC.speed_error_int = 0.0f;
//
//	        if (fabsf(_motor->FOC.speed_req) > 0.0f) {
//	            _motor->speed_ctrl_state = SPEED_CTRL_ALIGN;
//	        }
//	        break;
//
//	     /* ---------------- ALIGN ---------------- */
//	    case SPEED_CTRL_ALIGN:
//
//	        align_counter = _motor->speed_ctrl_limits.align_counter++;
//	        _motor->FOC.Idq_prereq.d = ALIGN_ID;
//	        _motor->FOC.Idq_prereq.q = 0.0f;
//
//
//
//	        if (align_counter > ALIGN_TIME_TICKS) {
//	        	 _motor->speed_ctrl_limits.align_counter=0;
//	            _motor->FOC.Idq_prereq.d = 0.0f;
//	            _motor->speed_ctrl_state = SPEED_CTRL_TORQUE_START;
//	        }
//
//	        if (fabsf(_motor->FOC.speed_req) == 0.0f) {
//	            _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
//	        }
//	        break;
//
//
//	    /* ----------- TORQUE START MODE ---------- */
//	    case SPEED_CTRL_TORQUE_START:
//	    {
//	        float start_iq = copysignf(START_IQ, _motor->FOC.speed_req);
//
//	        start_iq = clamp(start_iq,
//	                         -START_IQ_MAX,
//	                          START_IQ_MAX);
//
//	        _motor->FOC.Idq_prereq.q = start_iq;
//
//	        /* Transition condition: real motion detected */
//	        if (fabsf(_motor->FOC.eHz) > START_SPEED_EPS) {
//	            _motor->FOC.speed_error_int = start_iq;  // smooth handoff
//	            _motor->speed_ctrl_state = SPEED_CTRL_CLOSED_LOOP;
//	        }
//
//	        /* Abort if request removed */
//	        if (fabsf(_motor->FOC.speed_req) == 0.0f) {
//	            _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
//	        }
//	        break;
//	    }
//
//	    /* ----------- SPEED CLOSED LOOP ---------- */
//	    case SPEED_CTRL_CLOSED_LOOP:
//
//	        if (fabsf(_motor->FOC.speed_req) == 0.0f) {
//	            _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
//	            break;
//	        }
//
//	        speed_error =
//	            _motor->FOC.speed_kp *
//	            (_motor->FOC.speed_req - _motor->FOC.eHz);
//
//	        speed_error = clamp(speed_error,
//	                            -_motor->input_vars.max_request_Idq.q,
//	                             _motor->input_vars.max_request_Idq.q);
//
//	        _motor->FOC.speed_error_int +=
//	            speed_error * _motor->FOC.speed_ki;
//
//	        _motor->FOC.speed_error_int =
//	            clamp(_motor->FOC.speed_error_int,
//	                  -_motor->input_vars.max_request_Idq.q,
//	                   _motor->input_vars.max_request_Idq.q);
//
//	        _motor->FOC.Idq_prereq.q =
//	            clamp(speed_error + _motor->FOC.speed_error_int,
//	                  _motor->input_vars.min_request_Idq.q,
//	                  _motor->input_vars.max_request_Idq.q);
//	        break;
//	    }
//}


void RunModifiedSpeedControl(MESC_motor_typedef *_motor)
{
    float p_term, err;

    const float REQ_ON_EPS  = 0.5f;          // eHz
    const float REQ_OFF_EPS = 0.2f;          // eHz
    bool pos_mode = (_motor->ControlMode == MOTOR_CONTROL_MODE_POSITION);

    _motor->speed_ctrl_limits.debug_counter++;

    if (_motor->MotorState != MOTOR_STATE_RUN) {
        _motor->FOC.Idq_prereq.q = 0.0f;
        _motor->FOC.speed_error_int = 0.0f;
        _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
        return;
    }

    switch (_motor->speed_ctrl_state)
    {
    case SPEED_CTRL_IDLE:
        _motor->FOC.Idq_prereq.q = 0.0f;
        _motor->FOC.speed_error_int = 0.0f;

        // In position mode, we want "servo ready" behavior, not stop/start bouncing
        if (pos_mode) {
            _motor->speed_ctrl_state = SPEED_CTRL_CLOSED_LOOP;
            break;
        }

        if (fabsf(_motor->FOC.speed_req) > REQ_ON_EPS) {
            _motor->speed_ctrl_state = SPEED_CTRL_ALIGN;
        }
        break;

    case SPEED_CTRL_ALIGN:
        _motor->FOC.Idq_prereq.d = ALIGN_ID;
        _motor->FOC.Idq_prereq.q = 0.0f;

        if (++_motor->speed_ctrl_limits.align_counter >= ALIGN_TIME_TICKS) {
            _motor->speed_ctrl_limits.align_counter = 0;
            _motor->FOC.Idq_prereq.d = 0.0f;
            _motor->speed_ctrl_state = SPEED_CTRL_TORQUE_START;
        }

        // Don’t drop out of ALIGN due to tiny req jitter
        if (!pos_mode && fabsf(_motor->FOC.speed_req) < REQ_OFF_EPS) {
            _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
        }
        break;

    case SPEED_CTRL_TORQUE_START:
    {
        float start_iq = copysignf(START_IQ, _motor->FOC.speed_req);
        start_iq = clamp(start_iq, -START_IQ_MAX, START_IQ_MAX);
        _motor->FOC.Idq_prereq.q = start_iq;

        if (fabsf(_motor->FOC.eHz) > START_SPEED_EPS) {
            _motor->FOC.speed_error_int = start_iq;   // smooth handoff
            _motor->speed_ctrl_state = SPEED_CTRL_CLOSED_LOOP;
        }

        if (!pos_mode && fabsf(_motor->FOC.speed_req) < REQ_OFF_EPS) {
            _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
        }
        break;
    }

    case SPEED_CTRL_CLOSED_LOOP:
        // In speed mode, allow disable when request goes near zero.
        // In position mode, NEVER disable just because speed_req is ~0 (servo hold).
        if (!pos_mode && fabsf(_motor->FOC.speed_req) < REQ_OFF_EPS) {
            _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
            break;
        }

        err = (_motor->FOC.speed_req - _motor->FOC.eHz);
        p_term = _motor->FOC.speed_kp * err;

        // PI output is in amps (Iq)

       _motor->FOC.speed_error_int += (err * _motor->FOC.speed_ki) * SPEED_DT;
       // _motor->FOC.speed_error_int += (err * _motor->FOC.speed_ki) ;
        // Clamp integrator in amps
        _motor->FOC.speed_error_int =
            clamp(_motor->FOC.speed_error_int,
                  -_motor->input_vars.max_request_Idq.q,
                   _motor->input_vars.max_request_Idq.q);

        // Sum P + I then clamp
        _motor->FOC.Idq_prereq.q =
            clamp(p_term + _motor->FOC.speed_error_int,
                  _motor->input_vars.min_request_Idq.q,
                  _motor->input_vars.max_request_Idq.q);

        // Breakaway assist: if near standstill but a meaningful speed is requested,
        // enforce a minimum Iq to overcome stiction.
        if (fabsf(_motor->FOC.eHz) < START_SPEED_EPS &&
            fabsf(_motor->FOC.speed_req) > 5.0f)
        {
            float iq_min = copysignf(START_IQ, _motor->FOC.speed_req);
            if (fabsf(_motor->FOC.Idq_prereq.q) < fabsf(iq_min)) {
                _motor->FOC.Idq_prereq.q = iq_min;
            }
        }
        break;

    default:
        _motor->speed_ctrl_state = SPEED_CTRL_IDLE;
        break;
   }
}
