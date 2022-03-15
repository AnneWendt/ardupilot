#include "Sub.h"


/*
 * control_acro.pde - init and run calls for acro flight mode
 */

// acro_init - initialise acro controller
bool Sub::acro_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    // attitude hold inputs become thrust inputs in acro mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    // make sure the user knows this is a modified ArduSub
    gcs().send_text(MAV_SEVERITY_INFO,"#Acrobatics mode for maintenance work");

    // get and save current yaw
    acro_start_yaw = ahrs.yaw_sensor;

    return true;
}

// acro_run - runs the acro controller
// should be called at 100hz or more
void Sub::acro_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (pitch_and_dock) {
        //we're still trying to maneuver ourselves into the correct position
        //pitch to 90 degrees and let the ROV move around
        attitude_control.input_euler_angle_roll_pitch_yaw(0.0F, 8900.0F, acro_start_yaw, true);
        //intention: turn ROV using the roll input - problem: this function triggers INSTANT movement to target
        //attitude_control.input_euler_angle_roll_pitch_yaw(0.0F, 8900.0F, 18000 * channel_roll->norm_input(), true);
        motors.set_throttle(channel_throttle->norm_input());
        motors.set_forward(channel_forward->norm_input());
        motors.set_lateral(channel_lateral->norm_input());
    } else {
        //we are ready to dock to the wall
        motors.set_throttle(0.5 - (gain/2));
    }

    //motors.set_roll(0.0F);
    //motors.set_pitch((pitch_and_throttle ? (0.0F - gain) : 0.0F));
    //motors.set_pitch(0.0F); //goes into full spin for some reason
    //motors.set_yaw(0.0F);
    //motors.set_throttle(0.5F); //neutral throttle
    //motors.set_throttle(0.5 - (gain/2)); //works with ROV!
    //motors.set_forward(0.0F);  //positive is forward, negative is backward
    //motors.set_lateral(0.0F);  //positive is right, negative is left

    //attitude_control.input_euler_angle_roll_pitch_yaw(0.0F, 89.0F, 0.0F, true); //only spins a little bit but nothing else
    //attitude_control.input_euler_angle_roll_pitch_yaw(0.0F, 8900.0F, 0.0F, true);  //works with ROV but always turns North because of yaw = 0
}

// This method is not necessary for Frankenstein, but who knows what else is using it so let's keep it here just to be on the safe side

// get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
// returns desired angle rates in centi-degrees-per-second
void Sub::get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > ROLL_PITCH_INPUT_MAX) {
        float ratio = (float)ROLL_PITCH_INPUT_MAX / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // calculate roll, pitch rate requests
    if (g.acro_expo <= 0) {
        rate_bf_request.x = roll_in * g.acro_rp_p;
        rate_bf_request.y = pitch_in * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_expo > 1.0f) {
            g.acro_expo = 1.0f;
        }

        // roll expo
        rp_in = float(roll_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.x = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = float(pitch_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.y = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request.z = yaw_in * g.acro_yaw_p;

    // calculate earth frame rate corrections to pull the vehicle back to level while in ACRO mode

    if (g.acro_trainer != ACRO_TRAINER_DISABLED) {
        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
        rate_ef_level.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
        rate_ef_level.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            if (roll_angle > aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle-aparm.angle_max);
            } else if (roll_angle < -aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle+aparm.angle_max);
            }

            if (pitch_angle > aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle-aparm.angle_max);
            } else if (pitch_angle < -aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle+aparm.angle_max);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control.euler_rate_to_ang_vel(attitude_control.get_att_target_euler_cd()*radians(0.01f), rate_ef_level, rate_bf_level);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.z += rate_bf_level.z;
        } else {
            float acro_level_mix = constrain_float(1-MAX(MAX(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0, 0, 1)*ahrs.cos_pitch();

            // Scale leveling rates by stick input
            rate_bf_level = rate_bf_level*acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.x)-fabsf(rate_bf_level.x));
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.x = constrain_float(rate_bf_request.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.y)-fabsf(rate_bf_level.y));
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.y = constrain_float(rate_bf_request.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.z)-fabsf(rate_bf_level.z));
            rate_bf_request.z += rate_bf_level.z;
            rate_bf_request.z = constrain_float(rate_bf_request.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}
