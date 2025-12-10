/*
 * Following example at
 *      https://www.scilab.org/discrete-time-pid-controller-implementation
 *
 * Also following "official" Arduino PID controller lib:
 *      https://github.com/Tellicious/ArduPID-Library
 */

#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t pid_err_t;

// Define for using high precission floats
#ifdef PID_USE_FLOAT64
typedef double pid_float;
#else
typedef float pid_float;
#endif


#define clip(in,inf,sup) ((in)<(inf)?(inf):((in)>(sup)?(sup):(in)))

// Error codes
#define PID_OK  0
#define PID_ERR_INVALID_ARGS (-1)
#define PID_ERR_MANUAL_MODE_ON (-2)

typedef enum {
    PID_MODE_MANUAL = 0,
    PID_MODE_AUTO = 1
} pid_mode_enum;

typedef enum {
    PID_SMOOTH_MANUAL_TO_AUTO_OFF = 0,
    PID_SMOOTH_MANUAL_TO_AUTO_ON = 1
} pid_smooth_manual_to_auto_enum;

typedef enum {
    PID_SMOOTH_TUNING_OFF = 0,
    PID_SMOOTH_TUNING_ON = 1
} pid_smooth_tuning_enum;

typedef enum {
    PID_ANTI_WINDUP_OFF = 0,
    PID_ANTI_WINDUP_INTEGRATOR_CLAMPING = 1,
	PID_ANTI_WINDUP_BACK_CALCULATION =2
} pid_anti_windup_enum;

typedef enum {
    PID_DERIVATIVE_ON_MEASURE_OFF = 0,
    PID_DERIVATIVE_ON_MEASURE_ON = 1
} pid_derivative_on_meas_enum;

typedef enum {
    PID_DISCRETE_FORWARD_EULER = 0,
    PID_DISCRETE_BACKWARD_EULER = 1,
    PID_DISCRETE_TUSTIN = 2
} pid_discretization_method_enum;

typedef enum {
	PID_PROPORTIONAL_ON_MEASURE_OFF = 0,
	PID_PROPORTIONAL_ON_MEASURE_ON = 1
} pid_proportional_on_meas_enum;

/*
 * pid_float gives 7 bits precision.
 * float64_t gives 16 bits.
 */

typedef struct {

    /*
     * Standard continuous time PID controller constants in parallel form:
     *
     *                						de(t)
     *    u(t) = Kp*e(t) + Ki*∫e(t) + Kd * -----
     *                 						dt
     */
    pid_float kp;
    pid_float ki;
    pid_float kd;

    /*
     * Low-pass filter parameters for smoothing derivative component:
     *
     *                 N
     *   e(s) * s -> -----
     *               1+N/s
     */
    pid_float n;
    pid_float ts;


    /*
     * Integral and derivative states, so that each component
     * can be separately discretized.
     */

    pid_float i_state;
    pid_float d_state;

    /*
     * Error, measurement and action vars
     * for last samples
     */

    pid_float e_prev;
    pid_float w_prev;
    pid_float u_prev;


    pid_float max_output;
    pid_float min_output;

    // Config enums
    pid_mode_enum mode;
    pid_derivative_on_meas_enum derivative_on_meas;
    pid_proportional_on_meas_enum proportional_on_meas;
    pid_anti_windup_enum anti_windup;
    // TODO they do nothing as of now
    pid_smooth_manual_to_auto_enum smooth_manual_to_auto;
    pid_smooth_tuning_enum smooth_tuning;


    pid_discretization_method_enum i_method;
    pid_discretization_method_enum d_method;

    /*
     * Back-calculation anti-windup
     * If PID_ANTI_RESET_WINDUP_BACK_CALCULATION is selected,
     * integral contribution will be calculated like:
     *
     * 		I[k] += Kb * (u_sat[k] - u_unsat[k])
     *
     * Output saturation is done in pid_run with clip() macro function
     */

    pid_float kb_aw;


    /*
     * Discrete derivative filter coefficients:
     *
     * 		u_d[k] = -k_u * u_d[k-1] + k_w * w[k] + d_b1 * w[k-1]
     *
     * where if DERIVATIVE_ON_MEASURE_ON:
     * 		w[k] = -y[k]
     * 		 if DERIVATIVE_ON_MEASURE_OFF:
     * 		w[k] = e[k]
     */

    pid_float k_u;
    pid_float k_w;

} pid_controller_t;

/*
 * @brief 	Initialize PID controller with given gains and default config parameters
 *
 * @param 	pid_handle: 	already created PID Object
 * @param 	kp: 			proportional gain
 * @param 	ki: 			integral gain
 * @param 	kd: 			derivative gain
 * @param 	N: 				derivative low-pass filter constant, inverse to time constant tau N = 1/tau where s = 1/(1+tau*s)
 * @param 	Ts: 			sample time, controller supposes its constant, like it should for discrete systems
 *
 * @retVal: 				PID error code
 */
pid_err_t pid_init(pid_controller_t* pid_handle, pid_float kp, pid_float ki, pid_float kd, pid_float N, pid_float Ts);

/*
 * @brief	Reset controller, deleting all previous sample data.
 *
 * @param 	pid_handle: 	already created PID Object
 *
 * @retVal:					PID error code
 */
pid_err_t pid_reset(pid_controller_t* pid_handle);

/*
 * @brief 	Set saturation or "clamping" values for the output of the controller. It should match the actuator's input range.
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	max:			upper saturation limit
 * @param 	min:			lower saturation limit
 *
 * @retVal 					PID error code
 */
pid_err_t pid_set_clampling(pid_controller_t* pid_handle, pid_float max, pid_float min);

/*
 * @brief 	Set anti-windup method
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	anti_windup:	anti-windup method selector.
 * 							It can have the following values:
 *    						@arg PID_ANTI_WINDUP_OFF:					No anti-windup
 *   							@arg PID_ANTI_WINDUP_INTEGRATOR_CLAMPING:	Integral state is clamped when output saturates
 *   							@arg PID_ANTI_WINDUP_BACK_CALCULATION:		Back-calculation anti-windup method
 *
 * @retVal:					PID error code
 */
pid_err_t pid_set_anti_windup(pid_controller_t* pid_handle, pid_anti_windup_enum anti_windup);

/*
 * @brief	Change the PID controller's gains, low-pass filter time constant and sampling time. It is used by pid_init internally.
 *
 * @param 	pid_handle: 	already created PID Object
 * @param 	kp: 			proportional gain
 * @param 	ki: 			integral gain
 * @param 	kd: 			derivative gain
 * @param 	N: 				derivative low-pass filter constant, inverse to time constant tau N = 1/tau where s = 1/(1+tau*s)
 * @param 	Ts: 			sample time, controller supposes its constant, like it should for discrete systems
 * @param 	kb_aw:			back-calculation anti-windup gain
 *
 * @retVal: 				PID error code
 */
pid_err_t pid_set_params(pid_controller_t* pid_handle, pid_float kp, pid_float ki, pid_float kd, pid_float N, pid_float Ts, pid_float kb_aw);

/*
 * @brief 	Set proportional gain
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	kp: 			proportional gain
 *
 * @retVal:					PID error code
*/
pid_err_t pid_set_kp( pid_controller_t* pid_handle, pid_float kp );

/*
 * @brief 	Set integral gain
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	ki: 			integral gain
 *
 * @retVal:					PID error code
*/
pid_err_t pid_set_ki( pid_controller_t* pid_handle, pid_float ki );

/*
 * @brief 	Set derivative gain
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	kd: 			derivative gain
 *
 * @retVal:					PID error code
*/
pid_err_t pid_set_kd( pid_controller_t* pid_handle, pid_float kd );

/*
 * @brief 	Set derivative low-pass filter constant N
 * @param 	pid_handle:		already created PID Object
 * @param 	n: 				derivative low-pass filter constant, inverse to time constant tau N = 1/tau where s = 1/(1+tau*s)
 *
 * @retVal:					PID error code
*/
pid_err_t pid_set_n(  pid_controller_t* pid_handle, pid_float n  );

/*
 * @brief 	Set sample time Ts
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	ts: 			sample time, controller supposes its constant, like it should for discrete systems
 *
 * @retVal:					PID error code
*/
pid_err_t pid_set_ts( pid_controller_t* pid_handle, pid_float ts  );

/*
 * @brief 	Set back-calculation anti-windup gain kb_aw
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	kb_aw:			back-calculation anti-windup gain
 *
 * @retVal:					PID error code
*/
pid_err_t pid_set_kb_aw( pid_controller_t* pid_handle, pid_float kb_aw );

/*
 * @brief 	Run a single iteration of a PID controller, given a setpoint and a measurement. It assumes constant sample-time
 *
 * @param	pid_handle:		already created PID Object
 * @param 	setpoint:		setpoint, desired measurement value
 * @param 	measurement:	actual measurement or sensor reading
 *
 * @retVal:					PID error code
 */
pid_float pid_run(pid_controller_t* pid_handle, pid_float setpoint, pid_float measurement);

/*
 * @brief 	Set the discretization approximation method used in the calculation of Integral and Derivative components
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	method:			discretization method
 * 							It can have the following values:
 * 							@arg PID_DISCRETE_FORWARD_EULER: 	Laplace 's' is replaced by:
 * 																	Ts * z^-1
 * 																s=-------------
 * 																	  1-z^-1
 * 															 	It's reacts the most abruptly, only use
 * 															 	if reaction speed is important.
 *							@arg PID_DISCRETE_BACKWARD_EULER: 	Laplace 's' is replaced by:
 *																	1-z^-1
 *																s=-----------
 *																	  Ts
 *																Always stable in integral component, tends to give
 *																a more dampened response.
 *							@arg PID_DISCRETE_TUSTIN:			Laplace 's' is replaced by:
 *																	2*(1-z^-1)
 *																s=--------------
 *																	Ts*(1+z^-1)
 *																Is usually the best for derivative component, accurate
 *																but tends to slow down or "smooth" response
 * @retVal:					PID error code
 *
 */
pid_err_t pid_set_discretization_method(pid_controller_t* pid_handle, pid_discretization_method_enum method);

/*
 * @brief 	Set the discretization approximation method used in the calculation of Integral component
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	method:			discretization method
 * 							It can have the following values:
 * 							@arg PID_DISCRETE_FORWARD_EULER: 	Laplace 's' is replaced by:
 * 																	Ts * z^-1
 * 																s=-------------
 * 																	  1-z^-1
 * 															 	It's reacts the most abruptly, only use
 * 															 	if reaction speed is important.
 *							@arg PID_DISCRETE_BACKWARD_EULER: 	Laplace 's' is replaced by:
 *																	1-z^-1
 *																s=-----------
 *																	  Ts
 *																Always stable in integral component, tends to give
 *																a more dampened response.
 *							@arg PID_DISCRETE_TUSTIN:			Laplace 's' is replaced by:
 *																	2*(1-z^-1)
 *																s=--------------
 *																	Ts*(1+z^-1)
 *																Is usually the best for derivative component, accurate
 *																but tends to slow down or "smooth" response
 * @retVal:					PID error code
 *
 */
pid_err_t pid_set_integral_discretization_method(pid_controller_t* pid_handle, pid_discretization_method_enum method);

/*
 * @brief 	Set the discretization approximation method used in the calculation of  Derivative component
 *
 * @param 	pid_handle:		already created PID Object
 * @param 	method:			discretization method
 * 							It can have the following values:
 * 							@arg PID_DISCRETE_FORWARD_EULER: 	Laplace 's' is replaced by:
 * 																	Ts * z^-1
 * 																s=-------------
 * 																	  1-z^-1
 * 															 	It's reacts the most abruptly, only use
 * 															 	if reaction speed is important.
 *							@arg PID_DISCRETE_BACKWARD_EULER: 	Laplace 's' is replaced by:
 *																	1-z^-1
 *																s=-----------
 *																	  Ts
 *																Always stable in integral component, tends to give
 *																a more dampened response.
 *							@arg PID_DISCRETE_TUSTIN:			Laplace 's' is replaced by:
 *																	2*(1-z^-1)
 *																s=--------------
 *																	Ts*(1+z^-1)
 *																Is usually the best for derivative component, accurate
 *																but tends to slow down or "smooth" response
 * @retVal:					PID error code
 *
 */
pid_err_t pid_set_derivative_discretization_method(pid_controller_t* pid_handle, pid_discretization_method_enum method);


/*
 * @brief Set if derivative component acts on measurement or on error
 *
 * @param 	pid_handle:				already created PID Object
 * @param	derivative_on_meas:		derivative behaviour selector.
 * 									Can have the following values:
 *    								@arg	PID_DERIVATIVE_ON_MEASURE_OFF:	Derivative will act on e(t) error.
 *   								@arg	PID_DERIVATIVE_ON_MEASURE_ON:	Derivative will act only on measurement.
 *
 * @retVal:							PID error code
 */
pid_err_t pid_set_derivative_on_meas(pid_controller_t* pid_handle, pid_derivative_on_meas_enum derivative_on_meas);


/*
 * @brief Set if proportional gain is applied on measurement or on error
 *
 * @param 	pid_handle:				already created PID Object
 * @param	derivative_on_meas:		derivative behaviour selector.
 * 									Can have the following values:
 *    								@arg	PID_PROPORTIONAL_ON_MEASURE_OFF:	Proportional gain will act on e(t) error.
 *   								@arg	PID_PROPORTIONAL_ON_MEASURE_ON:		Proportional gain will act only on measurement.
 *
 * @retVal:							PID error code
 */
pid_err_t pid_set_proportional_on_meas(pid_controller_t* pid_handle, pid_proportional_on_meas_enum proportional_on_meas);

#ifdef __cplusplus
}
#endif

#endif //__PID_H__

