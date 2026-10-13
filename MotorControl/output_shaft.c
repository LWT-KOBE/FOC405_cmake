#include "MyProject.h"

volatile bool output_shaft_zero_ready_ = false;
volatile float output_pos_estimate_ = 0.0f;
volatile float output_vel_estimate_ = 0.0f;
float output_motor_circular_ = 0.0f;

volatile float output_input_pos_ = 0.0f;
volatile float output_input_vel_ = 0.0f;
volatile float output_input_torque_ = 0.0f;
volatile OutputShaftMitCommand output_mit_command_;

enum
{
	OUTPUT_INPUT_POSITION = 1U << 0,
	OUTPUT_INPUT_VELOCITY = 1U << 1,
	OUTPUT_INPUT_TORQUE = 1U << 2,
	OUTPUT_INPUT_MIT = 1U << 3
};
static volatile uint32_t pending_inputs_ = 0U;

// Track output-side values, never converted input_* values (which the controller
// can wrap or update itself). Unchanged targets must not restart trajectories.
static float last_input_pos_ = 0.0f;
static float last_input_vel_ = 0.0f;
static float last_input_torque_ = 0.0f;

static float motor_anchor_turns_ = 0.0f;
static float output_anchor_turns_ = 0.0f;

static float output_shaft_ratio(void)
{
#if USE_OUTPUT_SHAFT_GEAR_RATIO
	return (float)GR;
#else
	return 1.0f;
#endif
}

static float output_shaft_direction(void)
{
	return (float)OUTPUT_SHAFT_DIRECTION;
}

static float correct_external_position(float external_position_turns)
{
	float position = (float)OUTPUT_ENCODER_DIRECTION *
		(external_position_turns - OUTPUT_ENCODER_ZERO_OFFSET_TURNS);
	return fmodf_pos(position, 1.0f);
}

void output_shaft_init(void)
{
	output_shaft_zero_ready_ = false;
	motor_anchor_turns_ = 0.0f;
	output_anchor_turns_ = 0.0f;
	output_pos_estimate_ = 0.0f;
	output_vel_estimate_ = 0.0f;
	output_motor_circular_ = 0.0f;
	output_shaft_reset_inputs();
}

void output_shaft_invalidate(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	output_shaft_zero_ready_ = false;
	output_pos_estimate_ = 0.0f;
	output_vel_estimate_ = 0.0f;
	output_motor_circular_ = 0.0f;
	output_shaft_reset_inputs();
	if (primask == 0U)
	{
		__enable_irq();
	}
}

bool output_shaft_capture_power_on_position(float external_position_turns)
{
	float ratio = output_shaft_ratio();
	if (!isfinite(external_position_turns) ||
		(external_position_turns < 0.0f) ||
		(external_position_turns >= 1.0f) ||
		!(ratio > 0.0f))
	{
		return false;
	}

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (!dc_calib_valid || !is_ready_ || (motor_error != ERROR_NONE) ||
		!isfinite(pos_estimate_) || !isfinite(vel_estimate_))
	{
		if (primask == 0U)
		{
			__enable_irq();
		}
		return false;
	}

	motor_anchor_turns_ = pos_estimate_;
	output_anchor_turns_ = correct_external_position(external_position_turns);
	output_pos_estimate_ = output_anchor_turns_;
	output_vel_estimate_ = output_shaft_direction() * vel_estimate_ / ratio;
	if (isfinite(ctrl_config.circular_setpoint_range) &&
		(ctrl_config.circular_setpoint_range > 0.0f))
	{
		output_motor_circular_ = fmodf_pos(
			output_shaft_direction() * ratio * output_anchor_turns_,
			ctrl_config.circular_setpoint_range);
	}
	else
	{
		output_motor_circular_ = 0.0f;
	}
	output_shaft_zero_ready_ = true;
	output_shaft_reset_inputs();
	if (primask == 0U)
	{
		__enable_irq();
	}

	return true;
}

void output_shaft_update(void)
{
	float ratio = output_shaft_ratio();
	if (!output_shaft_zero_ready_)
	{
		return;
	}

	output_vel_estimate_ = output_shaft_direction() * vel_estimate_ / ratio;
	output_pos_estimate_ = output_anchor_turns_ +
		output_shaft_direction() * (pos_estimate_ - motor_anchor_turns_) / ratio;

	if (ctrl_config.circular_setpoints &&
		isfinite(ctrl_config.circular_setpoint_range) &&
		(ctrl_config.circular_setpoint_range > 0.0f))
	{
		output_motor_circular_ = fmodf_pos(
			output_shaft_direction() * ratio * output_pos_estimate_,
			ctrl_config.circular_setpoint_range);
	}
}

bool output_shaft_is_ready(void)
{
	return output_shaft_zero_ready_;
}

static bool output_shaft_accepts_inputs(void)
{
	return output_shaft_zero_ready_ && dc_calib_valid && is_ready_ && is_armed_ &&
		(current_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL) &&
		(motor_error == ERROR_NONE) && !anticogging.calib_anticogging;
}

void output_shaft_reset_inputs(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	pending_inputs_ = 0U;
	output_input_pos_ = output_shaft_zero_ready_ ? output_pos_estimate_ : 0.0f;
	output_input_vel_ = 0.0f;
	output_input_torque_ = 0.0f;
	last_input_pos_ = output_input_pos_;
	last_input_vel_ = output_input_vel_;
	last_input_torque_ = output_input_torque_;
	output_mit_command_.position = output_input_pos_;
	output_mit_command_.velocity = 0.0f;
	output_mit_command_.torque = 0.0f;
	output_mit_command_.kp = 0.0f;
	output_mit_command_.kd = output_shaft_velocity_gain_from_motor(0.05f);
	if (primask == 0U)
	{
		__enable_irq();
	}
}

void output_shaft_start_inputs(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	output_shaft_reset_inputs();
	// Keep legacy controller inputs motor-side, including on an immediate re-enable.
	input_pos_ = ctrl_config.circular_setpoints ? output_motor_circular_ : pos_estimate_;
	input_vel_ = 0.0f;
	input_torque_ = 0.0f;
	mit_target_pos_ = pos_estimate_;
	mit_target_velocity_ = 0.0f;
	mit_target_torque_ = 0.0f;
	mit_kp = 0.0f;
	mit_kd = 0.05f;
	if (primask == 0U)
	{
		__enable_irq();
	}
}

bool output_shaft_set_input_position(float position)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	float motor_position;
	bool accepted = output_shaft_accepts_inputs() &&
		output_shaft_position_to_motor(position, &motor_position);
	if (accepted)
	{
		output_input_pos_ = position;
		pending_inputs_ |= OUTPUT_INPUT_POSITION;
	}
	if (primask == 0U)
	{
		__enable_irq();
	}
	return accepted;
}

bool output_shaft_set_input_velocity(float velocity)
{
	if (!isfinite(velocity) || !isfinite(output_shaft_velocity_to_motor(velocity)))
	{
		return false;
	}
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	bool accepted = output_shaft_accepts_inputs();
	if (accepted)
	{
		output_input_vel_ = velocity;
		pending_inputs_ |= OUTPUT_INPUT_VELOCITY;
	}
	if (primask == 0U)
	{
		__enable_irq();
	}
	return accepted;
}

bool output_shaft_set_input_torque(float torque)
{
	if (!isfinite(torque) || !isfinite(output_shaft_torque_to_motor(torque)))
	{
		return false;
	}
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	bool accepted = output_shaft_accepts_inputs();
	if (accepted)
	{
		output_input_torque_ = torque;
		pending_inputs_ |= OUTPUT_INPUT_TORQUE;
	}
	if (primask == 0U)
	{
		__enable_irq();
	}
	return accepted;
}

bool output_shaft_set_mit_command(float position, float velocity, float torque,
	float kp, float kd)
{
	if (!isfinite(position) || !isfinite(velocity) || !isfinite(torque) ||
		!isfinite(kp) || !isfinite(kd))
	{
		return false;
	}
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	// MIT always uses the controller's linear motor estimate, even in circular mode.
	float motor_position = motor_anchor_turns_ + output_shaft_direction() *
		output_shaft_ratio() * (position - output_anchor_turns_);
	bool accepted = output_shaft_accepts_inputs() && isfinite(motor_position) &&
		isfinite(output_shaft_velocity_to_motor(velocity)) &&
		isfinite(output_shaft_torque_to_motor(torque)) &&
		isfinite(output_shaft_velocity_gain_to_motor(kp)) &&
		isfinite(output_shaft_velocity_gain_to_motor(kd));
	if (accepted)
	{
		output_mit_command_.position = position;
		output_mit_command_.velocity = velocity;
		output_mit_command_.torque = torque;
		output_mit_command_.kp = kp;
		output_mit_command_.kd = kd;
		pending_inputs_ |= OUTPUT_INPUT_MIT;
	}
	if (primask == 0U)
	{
		__enable_irq();
	}
	return accepted;
}

void output_shaft_apply_inputs(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint32_t pending = pending_inputs_;
	pending_inputs_ = 0U;
	// Snapshot the public variables so direct writes use the same conversion path
	// as protocol commands, without requiring a setter or an update flag.
	float position = output_input_pos_;
	float velocity = output_input_vel_;
	float torque = output_input_torque_;
	if (output_shaft_accepts_inputs())
	{
		float motor_position;
		if ((pending & OUTPUT_INPUT_POSITION) || (position != last_input_pos_))
		{
			if (output_shaft_position_to_motor(position, &motor_position))
			{
				input_pos_ = motor_position;
				last_input_pos_ = position;
				// Only a new position target may restart trajectory planning.
				input_pos_updated_ = true;
			}
			else
			{
				output_input_pos_ = last_input_pos_;
			}
		}
		if ((pending & OUTPUT_INPUT_VELOCITY) || (velocity != last_input_vel_))
		{
			float motor_velocity = output_shaft_velocity_to_motor(velocity);
			if (isfinite(velocity) && isfinite(motor_velocity))
			{
				input_vel_ = motor_velocity;
				last_input_vel_ = velocity;
			}
			else
			{
				output_input_vel_ = last_input_vel_;
			}
		}
		if ((pending & OUTPUT_INPUT_TORQUE) || (torque != last_input_torque_))
		{
			float motor_torque = output_shaft_torque_to_motor(torque);
			if (isfinite(torque) && isfinite(motor_torque))
			{
				input_torque_ = motor_torque;
				last_input_torque_ = torque;
			}
			else
			{
				output_input_torque_ = last_input_torque_;
			}
		}
		if (pending & OUTPUT_INPUT_MIT)
		{
			mit_target_pos_ = motor_anchor_turns_ + output_shaft_direction() *
				output_shaft_ratio() * (output_mit_command_.position - output_anchor_turns_);
			mit_target_velocity_ = output_shaft_velocity_to_motor(output_mit_command_.velocity);
			mit_target_torque_ = output_shaft_torque_to_motor(output_mit_command_.torque);
			mit_kp = output_shaft_velocity_gain_to_motor(output_mit_command_.kp);
			mit_kd = output_shaft_velocity_gain_to_motor(output_mit_command_.kd);
		}
	}
	else
	{
		// A direct write while idle/faulted/calibrating must not be replayed on enable.
		output_input_pos_ = last_input_pos_;
		output_input_vel_ = last_input_vel_;
		output_input_torque_ = last_input_torque_;
	}
	if (primask == 0U)
	{
		__enable_irq();
	}
}

float output_shaft_get_position(void)
{
	return output_pos_estimate_;
}

float output_shaft_get_velocity(void)
{
	return output_vel_estimate_;
}

bool output_shaft_position_to_motor(float output_position, float *motor_position)
{
	if (!output_shaft_zero_ready_ || motor_position == NULL || !isfinite(output_position))
	{
		return false;
	}

	if (ctrl_config.circular_setpoints)
	{
		*motor_position = output_shaft_direction() * output_shaft_ratio() * output_position;
	}
	else
	{
		*motor_position = motor_anchor_turns_ + output_shaft_direction() *
			output_shaft_ratio() * (output_position - output_anchor_turns_);
	}
	return isfinite(*motor_position);
}

float output_shaft_velocity_to_motor(float output_velocity)
{
	return output_shaft_direction() * output_shaft_ratio() * output_velocity;
}

float output_shaft_velocity_from_motor(float motor_velocity)
{
	return output_shaft_direction() * motor_velocity / output_shaft_ratio();
}

float output_shaft_torque_to_motor(float output_torque)
{
	return output_shaft_direction() * output_torque / output_shaft_ratio();
}

float output_shaft_torque_from_motor(float motor_torque)
{
	return output_shaft_direction() * output_shaft_ratio() * motor_torque;
}

float output_shaft_motion_limit_to_motor(float output_value)
{
	return output_shaft_ratio() * output_value;
}

float output_shaft_motion_limit_from_motor(float motor_value)
{
	return motor_value / output_shaft_ratio();
}

float output_shaft_torque_limit_to_motor(float output_value)
{
	return output_value / output_shaft_ratio();
}

float output_shaft_torque_limit_from_motor(float motor_value)
{
	return output_shaft_ratio() * motor_value;
}

float output_shaft_velocity_gain_to_motor(float output_gain)
{
	float ratio = output_shaft_ratio();
	return output_gain / (ratio * ratio);
}

float output_shaft_velocity_gain_from_motor(float motor_gain)
{
	float ratio = output_shaft_ratio();
	return motor_gain * ratio * ratio;
}
