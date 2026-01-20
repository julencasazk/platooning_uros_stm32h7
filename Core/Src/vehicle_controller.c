#include "vehicle_controller.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>




VC_Err_t vehicle_controller_init(vehicle_controller_t *ctrl,
                                 const vehicle_controller_params_t *params)
{
    // Input Sanitizing
    // =============================================================
    if (ctrl == NULL || params == NULL)
        return VC_ERR_NULL_PARAM;
    if (
        params->pid_ts_s <= 0.0f ||
        params->speed_sp_min_mps > params->speed_sp_max_mps ||
        params->v1_mps >= params->v2_mps ||
        params->hysteresis_mps <= 0.0f)
        return VC_ERR_BAD_PARAM;

    if (params->brake_enable &&
        (params->brake_deadband < 0.0f ||
         params->brake_rate_limit_per_s < 0.0f ||
         params->brake_map_high.min_brake >= params->brake_map_high.max_brake ||
         params->brake_map_high.min_decel >= params->brake_map_high.max_decel ||
         params->brake_map_mid.min_brake >= params->brake_map_mid.max_brake ||
         params->brake_map_mid.min_decel >= params->brake_map_mid.max_decel ||
         params->brake_map_low.min_brake >= params->brake_map_low.max_brake ||
         params->brake_map_low.min_decel >= params->brake_map_low.max_decel))
        return VC_ERR_BAD_PARAM;

    if (params->coop_enable &&
        (params->tick_hz <= 0 ||
         params->coop_timeout_s < 0 ||
         params->coop_decel_off > params->coop_decel_on))
        return VC_ERR_BAD_PARAM;

    // TODO More sanity, like brake gating, histeresis... etc

    // ==============================================================

    // Start from 0 and safe default values
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->brake_active = false;
    ctrl->brake_cmd_prev = 0.0f;
    ctrl->current_range = VC_SPEED_RANGE_LOW;

    pid_controller_t *pidptr = &(ctrl->speed_pid);

    pid_err_t piderr = pid_init(
        pidptr,
        (pid_float)params->gains_low.kp,
        (pid_float)params->gains_low.ki,
        (pid_float)params->gains_low.kd,
        (pid_float)params->gains_low.n,
        params->pid_ts_s);
    if (piderr != _PID_OK)
        return VC_ERR_PIDLIB;
    piderr = pid_reset(pidptr);
    if (piderr != _PID_OK)
        return VC_ERR_PIDLIB;
    piderr = pid_set_anti_windup(pidptr, _PID_ANTI_WINDUP_BACK_CALCULATION);
    if (piderr != _PID_OK)
        return VC_ERR_PIDLIB;
    // KAW and Clamping values are Hardcoded for now,
    // i'm too lazy to change how they are set
    piderr = pid_set_kb_aw(pidptr, 1.0f);
    if (piderr != _PID_OK)
        return VC_ERR_PIDLIB;
    piderr = pid_set_derivative_on_meas(pidptr, _PID_DERIVATIVE_ON_MEASURE_ON);
    if (piderr != _PID_OK)
        return VC_ERR_PIDLIB;
    piderr = pid_set_discretization_method(pidptr, _PID_DISCRETE_TUSTIN);
    if (piderr != _PID_OK)
        return VC_ERR_PIDLIB;
    piderr = pid_set_clamping(pidptr, 1.0f, 0.0f);
    if (piderr != _PID_OK)
        return VC_ERR_PIDLIB;

    return VC_OK;
}




VC_Err_t vehicle_controller_step(vehicle_controller_t *ctrl,
                                 const vehicle_controller_params_t *params,
                                 const vehicle_controller_inputs_t *in,
                                 VC_Tick_t now_tick,
                                 vehicle_controller_outputs_t *out)
{

    // Select PID based on current speed +  histeresis to avoid chatter
    VC_SpeedRange_t prev = ctrl->current_range;
    switch (prev)
    {
    case VC_SPEED_RANGE_LOW:
        if (in->speed_mps > (params->v1_mps + params->hysteresis_mps))
            ctrl->current_range = VC_SPEED_RANGE_MID;
        break;
    case VC_SPEED_RANGE_MID:
        if (in->speed_mps > (params->v2_mps + params->hysteresis_mps))
        {
            ctrl->current_range = VC_SPEED_RANGE_HIGH;
        }
        else if (in->speed_mps < (params->v1_mps - params->hysteresis_mps))
        {
            ctrl->current_range = VC_SPEED_RANGE_LOW;
        }
        break;
    case VC_SPEED_RANGE_HIGH:
        if (in->speed_mps < (params->v2_mps - params->hysteresis_mps))
            ctrl->current_range = VC_SPEED_RANGE_MID;
        break;
    default: // Should never reach
        break;
    }

    // Only reset PID internal states when switching gains
    if (ctrl->current_range != prev)
    {
        pid_reset(&(ctrl->speed_pid));

        switch (ctrl->current_range)
        {
        case VC_SPEED_RANGE_LOW:
            pid_set_params(
                &(ctrl->speed_pid),
                params->gains_low.kp,
                params->gains_low.ki,
                params->gains_low.kd,
                params->gains_low.n,
                params->pid_ts_s,
                ctrl->speed_pid.kb_aw);
            break;
        case VC_SPEED_RANGE_MID:
            pid_set_params(
                &(ctrl->speed_pid),
                params->gains_mid.kp,
                params->gains_mid.ki,
                params->gains_mid.kd,
                params->gains_mid.n,
                params->pid_ts_s,
                ctrl->speed_pid.kb_aw);
            break;
        case VC_SPEED_RANGE_HIGH:
            pid_set_params(
                &(ctrl->speed_pid),
                params->gains_high.kp,
                params->gains_high.ki,
                params->gains_high.kd,
                params->gains_high.n,
                params->pid_ts_s,
                ctrl->speed_pid.kb_aw);
            break;
        default:
            break;
        }
    }

    // THROTTLE COMPUTING
    // Computed with local setpoint, that changes with distance to
    // vehicle ahead and TTC
    // ==============================================================
    float throttle = 0.0f;

    // Base setpoint selection 
    float base_sp = in->platoon_enabled ? in->platoon_setpoint_mps : in->indiv_setpoint_mps;
    float ttc = 0.0f;
    float dist_error = 0.0f;

    // Preceding speed can be missing early; treat as equal speed (no closing).
    const float v_lead = (in->preceding_speed_mps > 0.0f) ? in->preceding_speed_mps : in->speed_mps;
    const float speed_diff = v_lead - in->speed_mps; 

    // Spacing-based setpoint correction 
    if ((params->platoon_index > 0) && (in->distance_to_front_m > 0.0f))
    {
        const float desired_dist = fmaxf(
            params->min_spacing_m + 3.0f,
            in->speed_mps * params->desired_time_headway_s + params->min_spacing_m);
        dist_error = in->distance_to_front_m - desired_dist;

        float d_sp = (params->k_dist * dist_error) + params->k_vel * speed_diff;
        if (dist_error < 0.0f)
            d_sp = 4.0f * d_sp;

        base_sp = fminf(base_sp + d_sp, params->speed_sp_max_mps);
        base_sp = fmaxf(base_sp, params->speed_sp_min_mps);
    }
    else
    {
        // TODO leader also must keep distance 
        base_sp = fminf(base_sp, params->speed_sp_max_mps);
        base_sp = fmaxf(base_sp, params->speed_sp_min_mps);
    }

    // TTC calculation
    if ((in->distance_to_front_m > 0.0f) && (speed_diff < -0.1f))
    {
        ttc = in->distance_to_front_m / (-1.0f * speed_diff);
    }
    else
    {
        ttc = 1e3f; // Large number, in Python its "inf" 
    }

    throttle = (float)pid_run(&(ctrl->speed_pid), base_sp, in->speed_mps);
    // =======================================================================

    // BRAKE COMPUTING
    // The brake should be applied with the preceding vehicle's braking intent
    // if possible, and with TTC and relative speed if it's not available
    // =======================================================================
    float brake = 0.0f;
    float desired_decel = 0.0f;

    if (params->brake_enable)
    {
        // Gate checks
        bool too_close_on, too_close_off,
            ttc_on, ttc_off,
            overspeed_on, overspeed_off,
            ff_on, ff_off;

        too_close_on = (params->platoon_index > 0) && (in->distance_to_front_m > 0.0f) && (dist_error < -1.0f * params->dist_on_m);
        too_close_off = (params->platoon_index > 0) && (in->distance_to_front_m > 0.0f) && (dist_error < -1.0f * params->dist_off_m);

        ttc_on = (params->platoon_index > 0) && (ttc < params->ttc_on_s);
        ttc_off = (params->platoon_index > 0) && (ttc < params->ttc_off_s);

        float overspeed = in->speed_mps - base_sp;
        // Only activate when throttle is almost off and still not slowing down
        overspeed_on = (throttle < 0.05f) && (overspeed > params->v_margin_on_mps);
        overspeed_off = (throttle < 0.05f) && (overspeed > params->v_margin_off_mps);

        // Cooperative braking feedforward
        float ff_decel = 0.0f;
        if (params->coop_enable && (params->platoon_index > 0) && in->preceding_desired_decel_valid)
        {
            const uint32_t age_ticks = (uint32_t)(now_tick - in->preceding_desired_decel_rx_tick);
            const uint32_t timeout_ticks = (uint32_t)lrintf(params->coop_timeout_s * (float)params->tick_hz);
            if (age_ticks <= timeout_ticks)
            {
                ff_decel = fmaxf(0.0f, in->preceding_desired_decel_mps2);
            }
        }

        ff_on = (ff_decel >= params->coop_decel_on);
        ff_off = (ff_decel >= params->coop_decel_off);

        // Optional debug, track if we're following feedforward intent
        ctrl->ff_active = ff_off;

        bool want_brake = false;

        if (ctrl->brake_active)
        {
            want_brake = (too_close_off || ttc_off || overspeed_off || ff_off);
        }
        else
        {
            want_brake = (too_close_on || ttc_on || overspeed_on || ff_on);
        }

        float fb_decel = 0.0f; // Local braking desire, not coming from
                               // preceding intent

        if (!want_brake)
        {
            ctrl->brake_active = false;
            brake = 0.0f;
            desired_decel = 0.0f;
        }
        else
        {
            if (!ctrl->brake_active)
            {
                ctrl->brake_active = true;
                pid_reset(&(ctrl->speed_pid));
            }

            // 3 different braking levels
            // - Mild: Only overspeeding
            // - Moderate: closing in or too close
            // - Full: TTC very low, collission imminent
            if (too_close_on && ttc_on)
            {
                if (ttc < 1.2f || dist_error < -2.0f)
                {
                    fb_decel = params->decel_full;
                }
                else
                {
                    fb_decel = params->decel_mod;
                }
            }
            else if (overspeed_on || overspeed_off)
            {
                fb_decel = params->decel_mild;
            }

            desired_decel = fmaxf(fb_decel, params->coop_gain * ff_decel);

            // Desired decel to brake value mapping
            VC_BrakeMap_t brakemap = {0};
            switch (ctrl->current_range)
            {
            case VC_SPEED_RANGE_LOW:
                brakemap = params->brake_map_low;
                break;
            case VC_SPEED_RANGE_MID:
                brakemap = params->brake_map_mid;
                break;
            case VC_SPEED_RANGE_HIGH:
                brakemap = params->brake_map_high;
                break;
            default:
                break;
            }

            if (desired_decel <= brakemap.min_decel)
            {
                brake = brakemap.min_brake;
            }
            else if (desired_decel >= brakemap.max_decel)
            {
                brake = brakemap.max_brake;
            }
            else
            {
                // Lerp brake value from min to max
                float t = (desired_decel - brakemap.min_decel) / (brakemap.max_decel - brakemap.min_decel);
                brake = brakemap.min_brake + t * (brakemap.max_brake - brakemap.min_brake);
            }

            brake = params->k_brake * brake;
        }

        // Rate limit, brake cannot be 1.0 immediately
        float dt_s = params->pid_ts_s;
        if (params->tick_hz > 0 && ctrl->have_last_step_tick)
        {
            dt_s = (float)(now_tick - ctrl->last_step_tick) / (float)params->tick_hz;
        }
        if (dt_s < 1e-3f)
            dt_s = 1e-3f;

        float max_brake_step = params->brake_rate_limit_per_s * dt_s;
        brake = fmaxf(
            ctrl->brake_cmd_prev - max_brake_step,
            fminf(brake, ctrl->brake_cmd_prev + max_brake_step)
        );

        // Deadband near 0.0 to avoid chattering 
        if ((brake < params->brake_deadband) && (!ctrl->brake_active))
            brake = 0.0f;

        brake = fmaxf(0.0f, fminf(1.0f, brake));
    }

    ctrl->brake_cmd_prev = brake; // Record last brake for ratelimit
    ctrl->last_step_tick = now_tick;
    ctrl->have_last_step_tick = true;

    // Write outputs
    out->brake_cmd = brake;
    out->desired_decel_mps2 = desired_decel;
    out->effective_speed_sp_mps = base_sp;
    if (brake > 0.0f)
    {
        out->throttle_cmd = 0.0f;
    }
    else
    {
        out->throttle_cmd = throttle;
    }

    return VC_OK;

    // =======================================================================
}
