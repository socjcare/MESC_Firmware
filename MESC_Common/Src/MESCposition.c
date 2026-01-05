;/*
 * MESCposition.c
 *
 *  Created on: Feb 14, 2023
 *      Author: HPEnvy
 */


/* Includes ------------------------------------------------------------------*/
#include "MESCposition.h"
#include <math.h>
//
//void RunPosControl(MESC_motor_typedef *_motor){
//	_motor->pos.set_position += 1;
////	if(_motor->pos.set_position>200000){_motor->pos.set_position = 100000;}
////	if(_motor->pos.set_position<200000){_motor->pos.set_position = _motor->pos.set_position+1500;}
//_motor->FOC.Idq_prereq.d = 2.0f;
//
//	_motor->pos.error = (float)(int)(_motor->pos.set_position - _motor->FOC.PLL_angle);
//	_motor->pos.p_error = _motor->pos.Kp * _motor->pos.error;
//	_motor->pos.int_error = _motor->pos.int_error + _motor->pos.Ki * _motor->pos.p_error;
//
//	_motor->pos.d_pos = (float)(int)(_motor->FOC.PLL_angle - _motor->pos.last_pll_pos);
//	_motor->pos.last_pll_pos = _motor->FOC.PLL_angle;
//	_motor->pos.d_error = -_motor->pos.Kd * _motor->pos.d_pos;
////Clamp the integral
//	if(_motor->pos.int_error>_motor->input_vars.max_request_Idq.q){_motor->pos.int_error = _motor->input_vars.max_request_Idq.q;}
//	if(_motor->pos.int_error<_motor->input_vars.min_request_Idq.q){_motor->pos.int_error = _motor->input_vars.min_request_Idq.q;}
////apply directional integral clamping
////	if(_motor->pos.error>0.0f){
////		if(_motor->pos.int_error<0.0f){
////			_motor->pos.int_error=0.0f;
////		}
////	}
////	if(_motor->pos.error<0.0f){
////		if(_motor->pos.int_error>0.0f){
////			_motor->pos.int_error=0.0f;
////		}
////	}
////Apply the PID
//	if(fabsf(_motor->pos.error)>_motor->pos.deadzone)
//	{
//		_motor->FOC.Idq_prereq.q = _motor->pos.p_error + _motor->pos.int_error + _motor->pos.d_error;
//	}else{_motor->FOC.Idq_prereq.q = 0.0f;}
////Clamp the output
//	if(_motor->FOC.Idq_prereq.q>_motor->input_vars.max_request_Idq.q){_motor->FOC.Idq_prereq.q = _motor->input_vars.max_request_Idq.q;}
//	if(_motor->FOC.Idq_prereq.q<_motor->input_vars.min_request_Idq.q){_motor->FOC.Idq_prereq.q = _motor->input_vars.min_request_Idq.q;}
//
//	__NOP();
//}


#include "MESCposition.h"
#include <math.h>

/*
 * Position controller:
 *  - Input:  pos_abs, pos_target  (encoder counts, multi-turn)
 *  - Output: FOC.speed_req        (electrical Hz)
 *
 * Structure:
 *   position P  -> speed request
 *   speed loop  -> Iq
 */
void RunPositionController(MESC_motor_typedef *m)
{
    int32_t pos_err_i;
    float   pos_err_f;
    float   mech_speed_cps;   // mechanical speed in counts/sec

    /* Position error in COUNTS */
    pos_err_i = m->position_ctrl.pos_target - m->position_ctrl.pos_abs;

    /* Deadband: hold position, do NOT command motion */
    if (abs(pos_err_i) <= m->position_ctrl.pos_eps) {
        m->FOC.speed_req = 0.0f;
        m->position_ctrl.vel_sp = 0.0f;
        return;
    }

    pos_err_f = (float)pos_err_i;

    /*
     * Position P controller
     * Units:
     *   pos_kp: (counts/sec) per count  => 1/sec
     */
    mech_speed_cps =
        m->position_ctrl.pos_kp * pos_err_f +
        m->position_ctrl.vel_sp;   // trajectory feedforward

    /* Clamp mechanical speed */
    if (mech_speed_cps >  m->position_ctrl.vel_limit)
        mech_speed_cps =  m->position_ctrl.vel_limit;
    if (mech_speed_cps < -m->position_ctrl.vel_limit)
        mech_speed_cps = -m->position_ctrl.vel_limit;

    /*
     * Convert mechanical counts/sec -> electrical Hz
     *
     * 1 mechanical rev = 65536 counts
     * electrical Hz = mech_rev/s * pole_pairs
     */
    m->FOC.speed_req =
        (mech_speed_cps / 65536.0f) * (float)m->m.pole_pairs;
}


