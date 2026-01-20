#include <stdint.h>


#ifndef __PLATOONING_H__
#define __PLATOONING_H__


#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
	_PLATOON_DISABLED = 0, // Vehicle acts as in Adaptive Cruise Control mode, no following
	_PLATOON_ENABLED = 1   // Standard platoon mode
} PLATOON_platoon_enabled_TypeDef;

typedef struct {
	float speed_mps;
	float distance_to_front_m; // <= 0.0f means invalid/unavailable
	float indiv_setpoint_mps;
	float platoon_setpoint_mps;
	float preceding_speed_mps;
	float preceding_braking_intent_mps2;
} PLATOON_inputs_t;

typedef struct {
	float throttle_cmd;
	float brake_cmd;
	float brake_intend_cmd;
} PLATOON_command_t;



typedef struct {

	uint8_t platoon_member_index;
	PLATOON_platoon_enabled_TypeDef is_platooning;
	const char* name;

	float time_headway;
	float min_spacing;
	float k_dist; // Distance gain effect on setpoint variation
	float k_vel; // Relative speed difference effect on setpoint variation


	float (*get_controller_action)(float speed, float setpoint);

} PLATOON_member_t;


PLATOON_command_t compute_control(const PLATOON_member_t* member,
		const PLATOON_inputs_t* in);

#ifdef __cplusplus
}
#endif

#endif // __PLATOONING_H__
