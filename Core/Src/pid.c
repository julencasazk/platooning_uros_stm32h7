#include "pid.h"
#include <stddef.h>


/*
 * Private function prototypes
 * Not to be used by user
 */
static void pid_update_derivative_coefficients(pid_controller_t* pid_handle);

static inline pid_float pid_integral_increment(pid_controller_t* pid_handle, pid_float e, pid_float e_prev)
{
	pid_float Ts = pid_handle->ts;
	pid_float Ki = pid_handle->ki;
	switch (pid_handle->i_method) {
		case PID_DISCRETE_FORWARD_EULER:
			return  Ki * Ts * e_prev;
		case PID_DISCRETE_BACKWARD_EULER:
			return Ki * Ts * e;
		case PID_DISCRETE_TUSTIN:
		default:
			return  0.5f * Ki * Ts * (e + e_prev);
	}

}


pid_err_t pid_init(pid_controller_t* pid_handle, pid_float kp, pid_float ki, pid_float kd, pid_float N, pid_float Ts)
{
    pid_err_t err = pid_set_params(pid_handle, kp, ki, kd, N, Ts, 0.0f);
    if (err != PID_OK) return err;

    pid_handle->anti_windup = PID_ANTI_WINDUP_OFF;
    pid_handle->derivative_on_meas = PID_DERIVATIVE_ON_MEASURE_OFF;
    pid_handle->proportional_on_meas = PID_PROPORTIONAL_ON_MEASURE_OFF;
    pid_handle->mode = PID_MODE_AUTO;
    pid_handle->smooth_manual_to_auto = PID_SMOOTH_MANUAL_TO_AUTO_ON;
    pid_handle->smooth_tuning = PID_SMOOTH_TUNING_ON;

	err = pid_set_derivative_discretization_method(pid_handle, PID_DISCRETE_TUSTIN);
	err = pid_set_integral_discretization_method(pid_handle, PID_DISCRETE_BACKWARD_EULER);
    err = pid_set_clampling(pid_handle, 1e6f, -1e6f);
    err = pid_reset(pid_handle);

    return err;
}


pid_err_t pid_set_clampling(pid_controller_t* pid_handle, pid_float max, pid_float min)
{
	if (max <= min) return PID_ERR_INVALID_ARGS;

	pid_handle->max_output = max;
	pid_handle->min_output = min;

	return PID_OK;
}


pid_err_t pid_reset(pid_controller_t* pid_handle)
{
    if (pid_handle == NULL) return PID_ERR_INVALID_ARGS;
    pid_handle->i_state = 0.0f;
    pid_handle->d_state = 0.0f;
    pid_handle->e_prev  = 0.0f;
    pid_handle->w_prev  = 0.0f;
    pid_handle->u_prev  = 0.0f;
    pid_handle->kb_aw   = 0.0f;
    return PID_OK;
}


pid_err_t pid_set_params(pid_controller_t* pid_handle, pid_float kp, pid_float ki, pid_float kd, pid_float N, pid_float Ts, pid_float kb_aw)
{
    if (pid_handle == NULL || kp < 0 || ki < 0 || kd < 0 || Ts <= 0 || N < 0 || kb_aw < 0)
        return PID_ERR_INVALID_ARGS;

    pid_handle->kp = kp;
    pid_handle->ki = ki;
    pid_handle->kd = kd;
    pid_handle->n  = N;
    pid_handle->ts = Ts;
    pid_handle->kb_aw = kb_aw;

    pid_update_derivative_coefficients(pid_handle);


    return PID_OK;
}


pid_float pid_run(pid_controller_t* pid_handle, pid_float setpoint, pid_float measurement)
{

	if(pid_handle == NULL) return PID_ERR_INVALID_ARGS;
	if(pid_handle->mode == PID_MODE_MANUAL) return PID_ERR_MANUAL_MODE_ON;

	pid_float e = setpoint - measurement;


	pid_float u_p = (pid_handle->proportional_on_meas ==  PID_PROPORTIONAL_ON_MEASURE_ON)
					? pid_handle->kp * measurement
					: pid_handle->kp * e;


	pid_float w = (pid_handle->derivative_on_meas == PID_DERIVATIVE_ON_MEASURE_ON)
					? -measurement
					: e;

    // Derivative using dedicated d_state (was mixing with u_prev)
    pid_float u_d = pid_handle->k_u * pid_handle->d_state
                  + pid_handle->k_w * w
                  - pid_handle->k_w * pid_handle->w_prev;
    pid_handle->d_state = u_d;
    pid_handle->w_prev  = w;

    // Integral base (no anti-windup)
    pid_float dI_base = pid_integral_increment(pid_handle, e, pid_handle->e_prev);
    pid_float i_candidate = pid_handle->i_state + dI_base;

    // Unsaturated output
    pid_float u_unsat = u_p + i_candidate + u_d;

    // Saturate
    pid_float u = clip(u_unsat, pid_handle->min_output, pid_handle->max_output);

    switch (pid_handle->anti_windup) {
        case PID_ANTI_WINDUP_INTEGRATOR_CLAMPING:
        {
            uint8_t pushing_high = (u >= pid_handle->max_output) && (e > 0.0f);
            uint8_t pushing_low  = (u <= pid_handle->min_output) && (e < 0.0f);
            if (!(pushing_high || pushing_low)) {
                pid_handle->i_state = i_candidate;
            }
            break;
        }
        case PID_ANTI_WINDUP_BACK_CALCULATION:
        {
            pid_float aw = pid_handle->kb_aw * (u - u_unsat);
            pid_handle->i_state = i_candidate + aw;  // = i_state_old + dI_base + kb*(u_sat - u_unsat)
            break;
        }
        case PID_ANTI_WINDUP_OFF:
        default:
            pid_handle->i_state = i_candidate;
            break;
    }

    pid_handle->e_prev = e;
    pid_handle->u_prev = u; // keep last saturated output only if needed elsewhere

    return u;
}


pid_err_t pid_set_discretization_method(pid_controller_t* pid_handle, pid_discretization_method_enum method)
{
	if (pid_handle == NULL) return PID_ERR_INVALID_ARGS;

	pid_handle->i_method = pid_handle->d_method = method;
	pid_update_derivative_coefficients(pid_handle);
	return PID_OK;
}

pid_err_t pid_set_integral_discretization_method(pid_controller_t* pid_handle, pid_discretization_method_enum method)
{
	if (pid_handle == NULL) return PID_ERR_INVALID_ARGS;

	pid_handle->i_method = method;
	return PID_OK;

}

pid_err_t pid_set_derivative_discretization_method(pid_controller_t* pid_handle, pid_discretization_method_enum method)
{
	if (pid_handle == NULL) return PID_ERR_INVALID_ARGS;

	pid_handle->d_method = method;
	pid_update_derivative_coefficients(pid_handle);
	return PID_OK;

}

pid_err_t pid_set_derivative_on_meas(pid_controller_t* pid_handle, pid_derivative_on_meas_enum derivative_on_meas)
{
    if (pid_handle == NULL ||
        derivative_on_meas < PID_DERIVATIVE_ON_MEASURE_OFF ||
        derivative_on_meas > PID_DERIVATIVE_ON_MEASURE_ON)
        return PID_ERR_INVALID_ARGS;

    pid_handle->derivative_on_meas = derivative_on_meas;

    return PID_OK;
}

pid_err_t pid_set_proportional_on_meas(pid_controller_t* pid_handle, pid_proportional_on_meas_enum proportional_on_meas)
{
    if (pid_handle == NULL ||
        proportional_on_meas < PID_PROPORTIONAL_ON_MEASURE_OFF ||
        proportional_on_meas > PID_PROPORTIONAL_ON_MEASURE_ON)
        return PID_ERR_INVALID_ARGS;

    pid_handle->proportional_on_meas = proportional_on_meas;

    return PID_OK;
}

static void pid_update_derivative_coefficients(pid_controller_t* pid_handle)
{
	if (pid_handle == NULL) return;

	pid_float Kd = pid_handle->kd;
	pid_float N = pid_handle->n;
	pid_float Ts = pid_handle->ts;

	// If derivative component disabled
	if (Kd <= 0.0f || N <= 0.0f) {

		pid_handle->k_u = 0.0f;
		pid_handle->k_w = 0.0f;
		return;
	}

	switch (pid_handle->d_method) {
		case PID_DISCRETE_FORWARD_EULER:
			/*
			 * s = (Ts * z^-1) / (Ts * z)
			 *
			 * u_d[k] = -(N*Ts - 1)*u_d[k-1] + K_d*N*W[k] - K_d*N*W[k-1]
			 *
			 */
			pid_handle->k_u = -(N*Ts - 1.0f);
			pid_handle->k_w = Kd*N;
			break;
		case PID_DISCRETE_BACKWARD_EULER:
			/*
			 * s = (1-z^-1) / Ts
			 *
			 * u_d[k] = (1/(1+N*Ts))*u_d[k-1] + ((Kd*N)/(1+N*Ts))*W[k] - ((Kd*N)/(1+N*Ts))W[k-1]
			 *
			 */
			pid_handle->k_u = 1.0f / (1.0f + N*Ts);
			pid_handle->k_w = ((Kd*N)/(1.0f+N*Ts));
			break;
		case PID_DISCRETE_TUSTIN:
		default:
			/*
			 * s = (2/Ts)*(1-z^-1)(1+z^-1)
			 *
			 * u_d[k] = -((N*Ts - 2)/(N*Ts + 2))*u_d[k-1] + ((2*Kd)/(N*Ts + 2))*W[k] - ((2*Kd)/(N*Ts + 2))W[k-1]
			 *
			 */

			pid_handle->k_u = -((N*Ts - 2.0f)/(N*Ts + 2.0f));
			pid_handle->k_w = ((2.0f*Kd)/(N*Ts + 2.0f));
			break;
	}

}

pid_err_t pid_set_kp( pid_controller_t* pid_handle, pid_float kp )
{
	if (pid_handle == NULL || kp < 0)
		return PID_ERR_INVALID_ARGS;

	pid_handle->kp = kp;

	return PID_OK;
}

pid_err_t pid_set_ki( pid_controller_t* pid_handle, pid_float ki )
{
	if (pid_handle == NULL || ki < 0)
		return PID_ERR_INVALID_ARGS;

	pid_handle->ki = ki;

	return PID_OK;
}

pid_err_t pid_set_kd( pid_controller_t* pid_handle, pid_float kd )
{
	if (pid_handle == NULL || kd < 0)
		return PID_ERR_INVALID_ARGS;

	pid_handle->kd = kd;
	pid_update_derivative_coefficients(pid_handle);

	return PID_OK;
}


pid_err_t pid_set_n(  pid_controller_t* pid_handle, pid_float n  )
{
	if (pid_handle == NULL || n < 0)
		return PID_ERR_INVALID_ARGS;

	pid_handle->n = n;
	pid_update_derivative_coefficients(pid_handle);

	return PID_OK;
}

pid_err_t pid_set_ts( pid_controller_t* pid_handle, pid_float ts  )
{
	if (pid_handle == NULL || ts <= 0)
		return PID_ERR_INVALID_ARGS;

	pid_handle->ts = ts;
	pid_update_derivative_coefficients(pid_handle);

	return PID_OK;
}

pid_err_t pid_set_kb_aw( pid_controller_t* pid_handle, pid_float kb_aw )
{
	if (pid_handle == NULL || kb_aw < 0)
		return PID_ERR_INVALID_ARGS;

	pid_handle->kb_aw = kb_aw;

	return PID_OK;
}

pid_err_t pid_set_anti_windup(pid_controller_t* pid_handle, pid_anti_windup_enum anti_windup)
{
	if (pid_handle == NULL ||
		anti_windup < PID_ANTI_WINDUP_OFF ||
		anti_windup > PID_ANTI_WINDUP_BACK_CALCULATION)
		return PID_ERR_INVALID_ARGS;

	pid_handle->anti_windup = anti_windup;

	return PID_OK;
}





