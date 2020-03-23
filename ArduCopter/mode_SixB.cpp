#include "Copter.h"

/*
 * Init and run calls for guided flight mode
 */

//init()和run()是两个必备的

// guided_init - initialise guided controller
//初始化时应该计算飞行路径
bool Copter::ModeSixB::init(bool ignore_checks)
{

    if (copter.position_ok() || ignore_checks) {
        // initialise yaw
        auto_yaw.set_mode_to_default(false);

        //将当前航点清零
        path_num=0;
        //产生要飞行的路径
        generate_path();

        // start in position control mode
        //产生路径之后飞控就会启动位置控制引导
        //飞机按照规划路径走
        pos_control_start();
        return true;
    }else{
        return false;
    }
}

void Copter:: ModeSixB::generate_path()
{
    float radius_cm=1000.0;
    //停止点作为第0航点
    wp_nav->get_wp_stopping_point(path[0]);
    path[1]=path[0]+Vector3f(cosf(radians(30)),-sinf(radians(30)),0)*radius_cm;
    path[2]=path[0]+Vector3f(0.0,-1.0f,0.0)*radius_cm;
    path[3]=path[0]+Vector3f(-cosf(radians(30)),-sinf(radians(30)),0)*radius_cm;
    path[4]=path[0]+Vector3f(-cosf(radians(30)),sinf(radians(30)),0)*radius_cm;
    path[5]=path[0]+Vector3f(0.0,1.0f,0.0)*radius_cm;
    path[6]=path[0]+Vector3f(cosf(radians(30)),sinf(radians(30)),0)*radius_cm;
    path[7]=path[1];
}

// initialise guided mode's position controller
void Copter::ModeSixB::pos_control_start()
{

    // initialise waypoint and spline controller
    //航点导航库的初始化
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used

    //将当前航点设为目标停止点
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    //航向控制设为默认
    auto_yaw.set_mode_to_default(false);
}


// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::ModeSixB::run()
{
    if(path_num<7){
            //判断是否到达目标航点
           if(wp_nav->reached_wp_destination()){
               //到达目标航点，切换到下一行点
               path_num++;
               wp_nav->set_wp_destination(path[path_num],false);

           }

        }

       pos_control_run();
 }


// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::ModeSixB::pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}
