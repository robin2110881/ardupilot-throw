#include "Copter.h"

#if MODE_THROW_ENABLED

// throw_init - initialise throw controller
bool ModeThrow::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // init state
    stage = Throw_Disarmed;
    nextmode_attempted = false;

    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), BRAKE_MODE_DECEL_RATE);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_U_cmss(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void ModeThrow::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    if (!motors->armed()) {
        // state machine entry is always from a disarmed state
        stage = Throw_Disarmed;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO,"waiting for throw");
        stage = Throw_Detecting;
        // record the detection start altitude (meters) for altitude-based triggers
        {
            float altitude_above_home;
            if (ahrs.home_is_set()) {
                ahrs.get_relative_position_D_home(altitude_above_home);
                altitude_above_home = -altitude_above_home; // altitude above home is returned as negative
            } else {
                altitude_above_home = pos_control->get_pos_estimate_NEU_cm().z * 0.01f; // cm -> m
            }
            detect_start_alt_m = altitude_above_home;
        }

    } else if (stage == Throw_Detecting && throw_detected()){
        gcs().send_text(MAV_SEVERITY_INFO,"throw detected - spooling motors");
        copter.set_land_complete(false);
        stage = Throw_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        gcs().send_text(MAV_SEVERITY_INFO,"throttle is unlimited - uprighting");
        stage = Throw_Uprighting;
    } else if (stage == Throw_Uprighting && throw_attitude_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = Throw_HgtStabilise;

        // initialise the z controller
        pos_control->init_U_controller_no_descent();

        // initialise the demanded height below/above the throw height from user parameters
        // this allows for rapidly clearing surrounding obstacles
        if (g2.throw_type == ThrowType::Drop) {
            pos_control->set_pos_desired_U_cm(pos_control->get_pos_estimate_NEU_cm().z - g.throw_altitude_descend * 100.0f);
        } else {
            pos_control->set_pos_desired_U_cm(pos_control->get_pos_estimate_NEU_cm().z + g.throw_altitude_ascend * 100.0f);
        }

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise && throw_height_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        stage = Throw_PosHold;

        // initialise position controller
        pos_control->init_NE_controller();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && throw_position_good()) {
        if (!nextmode_attempted) {
            switch ((Mode::Number)g2.throw_nextmode.get()) {
                case Mode::Number::AUTO:
                case Mode::Number::GUIDED:
                case Mode::Number::RTL:
                case Mode::Number::LAND:
                case Mode::Number::BRAKE:
                case Mode::Number::LOITER:
                    set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
                    break;
                default:
                    // do nothing
                    break;
            }
            nextmode_attempted = true;
        }
    }

    // Throw State Processing
    switch (stage) {

    case Throw_Disarmed:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        break;

    case Throw_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Wait_Throttle_Unlimited:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        break;

    case Throw_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control->set_pos_target_U_from_climb_rate_cm(0.0f);
        pos_control->update_U_controller();

        break;

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // use position controller to stop
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_NE_cm(vel, accel);
        pos_control->update_NE_controller();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0f);

        // call height controller
        pos_control->set_pos_target_U_from_climb_rate_cm(0.0f);
        pos_control->update_U_controller();

        break;
    }

#if HAL_LOGGING_ENABLED
    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = pos_control->get_vel_estimate_NEU_cms().length();
        const float velocity_z = pos_control->get_vel_estimate_NEU_cms().z;
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();

// @LoggerMessage: THRO
// @Description: Throw Mode messages
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Throw Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Throw: True if a throw has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright 
// @Field: HgtOk: True if the vehicle is within 50cm of the demanded height
// @Field: PosOk: True if the vehicle is within 50cm of the demanded horizontal position

        AP::logger().WriteStreaming(
            "THRO",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk",
            "s-nnoo----",
            "F-0000----",
            "QBffffbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity,
            (double)velocity_z,
            (double)accel,
            (double)ef_accel_z,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok);
    }
#endif  // HAL_LOGGING_ENABLED
}

bool ModeThrow::throw_detected()
{
    // Check that the AHRS is healthy enough for us to be doing detection:
    if (!ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID)) {
        return false;
    }
    if (!ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS)) {
        return false;
    }
    if (!ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return false;
    }

    // Determine thresholds (cm/s) and enforce parameter rules
    // New defaults and rules requested by user:
    // - default trig vel (horizontal) = 500 cm/s
    // - default trig velz (vertical) = 50 cm/s
    // - default throw_trig_z = 4.0 m
    // - THROW_TRIG_SRC: 0 = velocity-based (use trig_vel/trig_velz), 1 = altitude-based
    // - trig_vel/trig_velz: only take effect if > default and >= 10
    // - throw_trig_z: values < 1.0 are treated as default (4.0)

    int32_t thresh_high = 10;
    int32_t thresh_vert = 10;

    if (g2.throw_trig_src.get() == 0) {
        // velocity-based mode: use user params only when they are > default and >= 10
        int32_t v = g2.throw_trig_vel.get();
        if (v >= 10) {
            thresh_high = v;
        }
        int32_t vz = g2.throw_trig_vert_vel.get();
        if (vz >= 10) {
            thresh_vert = vz;
        }
    }

    // Check for high speed using selected threshold
    bool high_speed = pos_control->get_vel_estimate_NEU_cms().length_squared() > (thresh_high * thresh_high);

    // check for upwards or downwards trajectory (airdrop)
    bool changing_height;
    if (g2.throw_type == ThrowType::Drop) {
        changing_height = pos_control->get_vel_estimate_NEU_cms().z < -thresh_vert;
    } else {
        changing_height = pos_control->get_vel_estimate_NEU_cms().z > thresh_vert;
    }

    // Check the vertical acceleration is greater than 0.25g
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
    bool no_throw_action = copter.ins.get_accel().length() < 1.05f * GRAVITY_MSS;

    // fetch the altitude above home
    float altitude_above_home;  // Use altitude above home if it is set, otherwise relative to EKF origin
    if (ahrs.home_is_set()) {
        ahrs.get_relative_position_D_home(altitude_above_home);
        altitude_above_home = -altitude_above_home; // altitude above home is returned as negative
    } else {
        altitude_above_home = pos_control->get_pos_estimate_NEU_cm().z * 0.01f; // centimeters to meters
    }

    // Check that the altitude is within user defined limits
    const bool height_within_params = (g.throw_altitude_min == 0 || altitude_above_home > g.throw_altitude_min) && (g.throw_altitude_max == 0 || (altitude_above_home < g.throw_altitude_max));

    // Decide detection method based on THROW_TRIG_SRC:
    //  - 0: velocity-based detection (default behavior) using thresh_high and thresh_vert
    //  - 1: altitude-based detection (for drops) using THROW_TRIG_Z (meters)

    bool possible_throw_detected = false;
    bool throw_condition_confirmed = false;

    if (g2.throw_trig_src.get() == 1 && g2.throw_type == ThrowType::Drop) {
        // altitude-based trigger for drops: trigger when we've fallen more than THROW_TRIG_Z meters
        // enforce minimum allowed value for THROW_TRIG_Z (>= 0.1 m);
        float z = g2.throw_trig_z.get();
        if (z < 0.1f) {
            z = 0.1;
        }
        if (detect_start_alt_m <= 0.0f) {
            // if detect_start_alt_m wasn't recorded for some reason, use current altitude as reference and don't trigger yet
            detect_start_alt_m = altitude_above_home;
        }
        const float fallen_m = detect_start_alt_m - altitude_above_home;
        possible_throw_detected = (fallen_m > z) && height_within_params && no_throw_action;
        // we immediately confirm based on altitude (no additional velocity confirmation)
        throw_condition_confirmed = possible_throw_detected;
    } else {
        // velocity-based detection (existing logic)
        // High velocity or free-fall combined with increasing height indicate a possible air-drop or throw release
        possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action && height_within_params;

        // Record time and vertical velocity when we detect the possible throw.
        // If this is the first time, record immediately (free_fall_start_ms == 0);
        // otherwise only refresh if the previous record is older than 500 ms.
        if (possible_throw_detected && ((AP_HAL::millis() - free_fall_start_ms) > 500)) {
            free_fall_start_ms = AP_HAL::millis();
            free_fall_start_velz = pos_control->get_vel_estimate_NEU_cms().z;
            free_fall_start_alt = altitude_above_home;
        }

        // Confirmation: check for a sufficient downwards velocity change within 0.5s.
        // Previously this was hard-coded to 250 cm/s (2.5 m/s) which made the parameter
        // trig_vert ineffective for the final confirmation stage. Compute a confirmation
        // delta based on the selected vertical trigger threshold but enforce a sensible
        // minimum (250 cm/s) to avoid overly-sensitive confirmation.
        float confirmation_delta = 5.0f * (float)thresh_vert;
        throw_condition_confirmed = ((AP_HAL::millis() - free_fall_start_ms < 500) &&
                                    ((pos_control->get_vel_estimate_NEU_cms().z - free_fall_start_velz) < -confirmation_delta));
    }

    // start motors and enter the control mode if we are in continuous freefall / confirmed
    return throw_condition_confirmed;
}

bool ModeThrow::throw_attitude_good() const
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool ModeThrow::throw_height_good() const
{
    // Check that we are within 0.5m of the demanded height
    return (pos_control->get_pos_error_U_cm() < 50.0f);
}

bool ModeThrow::throw_position_good() const
{
    // check that our horizontal position error is within 50cm
    return (pos_control->get_pos_error_NE_cm() < 50.0f);
}

#endif
