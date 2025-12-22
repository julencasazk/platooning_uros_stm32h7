#include "platooning.h"
#include <math.h>





PLATOON_command_t compute_control(const PLATOON_member_t* member,
		const PLATOON_inputs_t* in)
{
	PLATOON_command_t command = {
			.brake_cmd = 0.0f,
			.throttle_cmd = 0.0f
	};

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

		// If too close, change setpoint harder
		if (dist_err < 0.0f) {
				d_sp = 2.0f * member->k_dist * dist_err;
		} else {
			// If too far, only speed up beyond base setpoint when platooning as a follower.
			if (member->is_platooning == _PLATOON_ENABLED
					&& member->role == _PLATOON_FOLLOWER) {
				d_sp = member->k_dist * dist_err;
				// TODO For safety, this speed should not exceed road's legal limit.
				// the LEADER should deccelerate,to not let the followers speed over
				// the limit.
			}
		}

	}


	u = member->get_controller_action(in->speed_mps, sp + d_sp);

	if (u >= 0.0f) {
		// If positive, needs throttle
		command.throttle_cmd = u;
	} else {
		// If negative, brake
		command.brake_cmd = fabsf(u);
		// TODO Brake should not be pushable to max, only in emergency braking.
	}

	return command;

}
