#ifndef __OUTPUT_SHAFT_H
#define __OUTPUT_SHAFT_H

#include <stdbool.h>

// Output-shaft feedback is expressed in output turns and output turns/s.
extern volatile bool output_shaft_zero_ready_;
extern volatile float output_pos_estimate_;
extern volatile float output_vel_estimate_;
extern float output_motor_circular_;

// Writable output-side commands: turns, turns/s and Nm. Direct assignment is
// detected and converted to the legacy motor-side input_* on the next control tick.
// Assign after closed-loop entry; stop/start/calibration reset the targets.
extern volatile float output_input_pos_;
extern volatile float output_input_vel_;
extern volatile float output_input_torque_;

typedef struct
{
	float position;
	float velocity;
	float torque;
	float kp;
	float kd;
} OutputShaftMitCommand;

extern volatile OutputShaftMitCommand output_mit_command_;

// Optional setters used by communication handlers. Direct assignment to the three
// output_input_* variables also works; use the position setter to retrigger an
// unchanged position target. MIT still requires its setter for an atomic command.
// Targets are accepted only in ready, armed closed-loop operation, outside calibration.
// A false result leaves both output-side targets and motor-side inputs unchanged.
bool output_shaft_set_input_position(float position);
bool output_shaft_set_input_velocity(float velocity);
bool output_shaft_set_input_torque(float torque);
bool output_shaft_set_mit_command(float position, float velocity, float torque,
	float kp, float kd);
// Detect direct writes and consume new commands once, before controller_update().
// Invalid direct values are restored to the last valid target; blocked writes are discarded.
void output_shaft_apply_inputs(void);
// Initialize both sides to hold the current position with zero speed/torque before arming.
void output_shaft_start_inputs(void);
// Clear pending commands on stop/start/calibration. Does not change input_* or the anchor.
void output_shaft_reset_inputs(void);

// Anchors the output coordinate once the internal encoder estimate is stable.
void output_shaft_init(void);
void output_shaft_invalidate(void);
bool output_shaft_capture_power_on_position(float external_position_turns);
void output_shaft_update(void);
bool output_shaft_is_ready(void);

float output_shaft_get_position(void);
float output_shaft_get_velocity(void);

// Signed command/feedback conversions. Torque conversions assume an ideal gearbox.
bool output_shaft_position_to_motor(float output_position, float *motor_position);
float output_shaft_velocity_to_motor(float output_velocity);
float output_shaft_velocity_from_motor(float motor_velocity);
float output_shaft_torque_to_motor(float output_torque);
float output_shaft_torque_from_motor(float motor_torque);

// Magnitude conversions for limits, ramps and trajectory parameters.
float output_shaft_motion_limit_to_motor(float output_value);
float output_shaft_motion_limit_from_motor(float motor_value);
float output_shaft_torque_limit_to_motor(float output_value);
float output_shaft_torque_limit_from_motor(float motor_value);

// Velocity-loop gains are torque/velocity, so both sides of the ratio apply.
float output_shaft_velocity_gain_to_motor(float output_gain);
float output_shaft_velocity_gain_from_motor(float motor_gain);

#endif
