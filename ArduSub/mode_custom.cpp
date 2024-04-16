#include "Sub.h"


bool ModeCustom_stab::init(bool ignore_checks) {
    // Message in the console to prevent the user
    gcs().send_text(MAV_SEVERITY_INFO, "Salut c'est 22Cardot, si tu veux utiliser mon mode custom, swipe up !");
   

    treatment.Connect_Camera();

    if (!treatment.isConnected()){
	    gcs().send_text(MAV_SEVERITY_INFO, "Camera not connected"
	return false;
    }
    
    x = 0;
    y = 0;
    z = 0;


    // Set reference to 0 (I guess)
    position_control->set_pos_target_z_cm(0);
  
    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void ModeCustom_stab::run()
{
    float displacement = tratment.updateDisplacement();
    
    x = displacement[0];
    y = displacement[1];
    z = displacement[2]; //In the image coordinate (z is the distance of the object)

    attitude_stab();
    translat_stab(x, y, z);
}


void ModeCustom_stab::attitude_stab()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(0.5,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->relax_z_controller(motors.get_throttle_hover());
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        float target_yaw;
        Quaternion(
            sub.set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = degrees(target_roll);
        target_pitch = degrees(target_pitch);
        target_yaw = degrees(target_yaw);

        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, sub.last_pilot_heading, true);
        }
    }
}

void ModeCustom_stab::translat_stab(float x, float y, float z)
{ //Ajout PID pour le déplacement
    float error_x = 0 - x;
    float error_y = 0 - y;
    float error_z = 0 - z;
    

    float x_comm = pid_x -> update_error(error_x, 0.01f);
    float y_comm = pid_y -> update_error(error_y, 0.01f);
    float z_comm = pid_z -> update_error(error_z, 0.01f); 
 
    motors.set_lateral(x);
    motors.set_forward(z);
    motors.set_throttle(y); // because x,y,z are on the image ref
}


