#include "platooning.h"
#include <math.h>




PLATOON_member_state_t get_state(PLATOON_member_t* member)
{
	PLATOON_member_state_t state;
	state.speed = member->get_speed();
	state.distance_to_front_veh = member->get_distance();
	return state;
}

PLATOON_command_t compute_control(PLATOON_member_t* member, float setpoint)
{
	PLATOON_command_t command = {
			.brake_cmd = 0.0f,
			.throttle_cmd = 0.0f
	};

	PLATOON_member_state_t state = get_state(member);

	float u = 0.0f;

	if (member->role ==  _PLATOON_FOLLOWER) {

		/*
		 *	Compute desired dist depending on the speed of the vehicle.
		 *	The distance between vehicles should increase as speed increases,
		 *	and should should represent enough space to hard brake in case
		 *	of emergency. Time headway is just the desired TTC if the car in front starts braking.
		 *	float desired_dist = state.speed * (member.time_headway + member.min_spacing);
		 *
		 *	If the vehicle ahead is the lead vehicle, there should be a bigger gap between
		 *	the lead and the first follower, since there is a delay between the lead emergency braking
		 *	and sending the correpsonding event to the followers.
		 */
		float desired_dist = state.speed * (member->time_headway + member->min_spacing);
		if (desired_dist <= 3.0f) desired_dist = 3.0f;

		float dist_err = state.distance_to_front_veh - desired_dist;
		float d_setpoint = 0.0f;
		if (dist_err >= 0.0f) {
			// If lacking behind, add some setpoint to catch up.
			d_setpoint = member->k_dist * dist_err;
		} else {
			// If too close, reduce the setpoint.
			// Should me MORE AGGRESIVE than accelerating,
			// as braking is way more important.
			d_setpoint = 2.0f * member->k_dist * dist_err;
		}

		u = member->get_controller_action(state.speed, setpoint + d_setpoint);


	} else {
		u = member->get_controller_action(state.speed, setpoint);
	}

	if (u >= 0.0f) {
		// If positive, needs throttle
		command.throttle_cmd = u;
	} else {
		command.brake_cmd = fabsf(u);
	}

	return command;

}
