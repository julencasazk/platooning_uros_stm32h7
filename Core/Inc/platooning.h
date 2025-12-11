
#ifndef __PLATOONING_H__
#define __PLATOONING_H__


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	_PLATOON_LEAD = 0,
	_PLATOON_FOLLOWER = 1
} PLATOON_member_type_TypeDef;

typedef struct {

	float speed;
	float distance_to_front_veh;

} PLATOON_member_state_t;

typedef struct {
	float throttle_cmd;
	float brake_cmd;
} PLATOON_command_t;


typedef struct {
	 
	PLATOON_member_type_TypeDef role;
	const char* name;

	float time_headway;
	float min_spacing;
	float k_dist;

	PLATOON_member_state_t current_state;

	float (*get_speed)(void);
	float (*get_distance)(void);

	float (*get_controller_action)(float speed, float setpoint);

} PLATOON_member_t;


PLATOON_member_state_t get_state(PLATOON_member_t* member);
PLATOON_command_t compute_control(PLATOON_member_t* member, float setpoint);

#ifdef __cplusplus
}
#endif

#endif // __PLATOONING_H__
