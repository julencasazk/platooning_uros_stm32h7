
#ifndef __PLATOONING_H__
#define __PLATOONING_H__


#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
	_PLATOON_LEAD = 0,
	_PLATOON_FOLLOWER = 1
} PLATOON_member_type_TypeDef;

typedef enum {
	_PLATOON_DISABLED = 0, // Vehicle acts as in Adaptive Cruise Control mode, no following
	_PLATOON_ENABLED = 1   // Standard platoon mode
} PLATOON_platoon_enabled_TypeDef;

typedef struct {
	float speed_mps;
	float distance_to_front_m; // <= 0.0f means invalid/unavailable
	float indiv_setpoint_mps;
	float platoon_setpoint_mps;
} PLATOON_inputs_t;

typedef struct {
	float throttle_cmd;
	float brake_cmd;
} PLATOON_command_t;

typedef struct {
	 
	PLATOON_member_type_TypeDef role;
	PLATOON_platoon_enabled_TypeDef is_platooning;
	const char* name;

	float time_headway;
	float min_spacing;
	float k_dist;


	float (*get_controller_action)(float speed, float setpoint);

} PLATOON_member_t;


PLATOON_command_t compute_control(const PLATOON_member_t* member,
		const PLATOON_inputs_t* in);

#ifdef __cplusplus
}
#endif

#endif // __PLATOONING_H__
