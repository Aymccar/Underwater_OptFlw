#include "Sub.h"


bool ModeCustom_stab::init(bool ignore_checks) {
    // Message in the console to prevent the user
    gcs().send_text(MAV_SEVERITY_INFO, "Salut c'est 22Cardot, si tu veux utiliser mon mode custom, swipe up !");
   

    //treatment.Connect_Camera();

    //if (!treatment.isConnected()){
    //	    gcs().send_text(MAV_SEVERITY_INFO, "Camera not connected"
    //	return false;
    //}
    
    x = 0;
    y = 0;
    z = 0;


    // Set reference to 0 (I guess)
    position_control->set_pos_target_z_cm(0);
  
    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void ModeCustom_stab::run()
{
    displacement = {0, 0, 0}; //tratment.updateDisplacement();
    
    x = displacement[0];
    y = displacement[1];
    z = displacement[2]; //In the image coordinate (z is the distance of the object)

    attitude_stab();
    translat_stab(x, y, z);
}


void ModeCustom_stab::attitude_stab()
{
    uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert pilot input to lean angles
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    // TODO2: move into mode.h
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // call attitude controller
    // update attitude controller targets

    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0;  // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, sub.last_pilot_heading, true);
        }
    }

}

  

void ModeCustom_stab::translat_stab(float x_mes, float y_mes, float z_mes)
{ //Ajout PID pour le déplacement
    float error_x = 0 - x_mes;
    float error_y = 0 - y_mes;
    float error_z = 0 - z_mes;
    

    float x_comm = pid_x -> update_error(error_x, 0.01f);
    float y_comm = pid_y -> update_error(error_y, 0.01f);
    float z_comm = pid_z -> update_error(error_z, 0.01f); 
 
    motors.set_lateral(x_comm);
    motors.set_forward(z_comm);
    motors.set_throttle(y_comm); // because x,y,z are on the image ref
}


