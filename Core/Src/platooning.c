#include "platooning.h"
#include <math.h>

typedef struct {
	float desired_dist;
	float dist_err;
	float v_diff;
	float ttc;
} PLATOON_spacing_signals_t;


PLATOON_spacing_signals_t compute_spacing_signals(const PLATOON_member_t* member, const PLATOON_inputs_t* in){

	float desired_dist = (float)fmax(
			member->min_spacing,
			(in->speed_mps * member->time_headway) + member->min_spacing
			);

	float dist_err = in->distance_to_front_m - desired_dist;

	float v_diff = in->preceding_speed_mps - in->speed_mps;

	float ttc = (float)1e3f; // Very high number by default

	if (v_diff < 0.1f && in->distance_to_front_m > 0.0f) {
		ttc = in->distance_to_front_m / (float)fabs(v_diff);
	}

	PLATOON_spacing_signals_t spacing_signals = {
			.desired_dist = desired_dist,
			.dist_err = dist_err,
			.v_diff = v_diff,
			.ttc = ttc
	};

	return spacing_signals;

}


float compute_brake(PLATOON_member_t* member, PLATOON_inputs_t* in, PLATOON_spacing_signals_t* spacing) {



}

PLATOON_command_t compute_control(const PLATOON_member_t* member,
		const PLATOON_inputs_t* in)
{
	PLATOON_command_t command = {
			.brake_cmd = 0.0f,
			.throttle_cmd = 0.0f
	};

	PLATOON_spacing_signals_t spacing_signals = compute_spacing_signals(member, in);

	// If platooning, must follow platoon setpoint,
	// if not, follow individual setpoint.
	float sp;
	if (member->is_platooning == _PLATOON_ENABLED) {
		sp = in->platoon_setpoint_mps;
	} else {
		sp = in->indiv_setpoint_mps;
	}

	float u = 0.0f;
	// Setpoint delta changes the setpoint slightly so that a 
	// certain distance to the car ahead can be mantained.
	float d_sp = 0.0f; 	

	// If there's no info of distance or car ahead too far
	if (in->distance_to_front_m > 0.0f){

		float desired_d = in->speed_mps * member->time_headway + member->min_spacing;
		// Always mantain a minimum distance, even when in low speed
		if (desired_d < (3.0f + member->min_spacing)) desired_d = 3.0f + member->min_spacing;
		float dist_err = in->distance_to_front_m - desired_d;
		float vel_err = in->preceding_speed_mps - in->speed_mps;

		// If too close, change setpoint harder
		if (dist_err < 0.0f) {
				d_sp = 4.0f * (member->k_dist * dist_err ) + (member->k_vel * vel_err);
		} else {
			// If too far, only speed up beyond base setpoint when platooning as a follower.
			if (member->is_platooning == _PLATOON_ENABLED
					&& member->platoon_member_index != 0) {
				d_sp = (member->k_dist * dist_err ) + (member->k_vel * vel_err);
				// TODO For safety, this speed should not exceed road's legal limit.
				// the LEADER should deccelerate,to not let the followers speed over
				// the limit.
			}
		}

	}

	float corrected_sp = sp + d_sp;
	if (corrected_sp > 33.33f) corrected_sp = 33.33f;
	// TODO Handle braking sepparately
	if (corrected_sp < 0.0f) corrected_sp = 0.0f; // Not sure about this one, might interfere with brakes

	u = member->get_controller_action(in->speed_mps, corrected_sp);

	if (u >= 0.0f) {
		// If positive, needs throttle
		command.throttle_cmd = u;
	} else {
		// If negative, brake
		command.brake_cmd = 0.08 * fabsf(u);
		// TODO Brake should not be pushable to max, only in emergency braking.
	}

	return command;

}
