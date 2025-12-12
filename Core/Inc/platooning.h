
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

// Each member points to a local data object that changes, this way
// it does not need manual updating constantly.
typedef struct {
	volatile float* speed;
	volatile float* indiv_setpoint;
	volatile float* distance_to_front_veh;
	volatile float* platoon_setpoint;
} PLATOON_member_state_t;

typedef struct {
	float throttle_cmd;
	float brake_cmd;
} PLATOON_command_t;

typedef struct {
	 
	PLATOON_member_type_TypeDef role;
	PLATOON_platoon_enabled_TypeDef is_platooning;
	PLATOON_member_state_t current_state;
	const char* name;

	float time_headway;
	float min_spacing;
	float k_dist;


	float (*get_controller_action)(float speed, float setpoint);

} PLATOON_member_t;


PLATOON_member_state_t get_state(PLATOON_member_t* member);
PLATOON_command_t compute_control(PLATOON_member_t* member);

#ifdef __cplusplus
}
#endif

#endif // __PLATOONING_H__
