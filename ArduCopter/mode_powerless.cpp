#include "Copter.h"

/*
 * Init and run calls for powerless flight mode
 */

// powerless_init - initialise powerless controller
bool Copter::ModePowerless::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// powerless_run - runs the main powerless controller
// should be called at 100hz or more
void Copter::ModePowerless::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
    pilot_throttle_scaled = (pilot_throttle_scaled - 1500) * 0.1 + 1500.;
    //pilot_throttle_scaled = 0.;
    //printf("[%8.3f]",pilot_throttle_scaled);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
