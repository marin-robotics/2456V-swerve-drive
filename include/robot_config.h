#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/gps.hpp"

// Controller
extern pros::Controller controller;

// GPS
extern pros::GPS gps;
extern pros::c::gps_status_s_t gps_status;

// Primary Drive Motors
extern pros::Motor front_left_primary;
extern pros::Motor front_right_primary;
extern pros::Motor back_left_primary;
extern pros::Motor back_right_primary;
extern pros::Motor_Group primary_motors;

// Angle Drive Motors
extern pros::Motor front_left_angle;
extern pros::Motor front_right_angle;
extern pros::Motor back_left_angle;
extern pros::Motor back_right_angle;
extern pros::Motor_Group angle_motors;

// Shooter
extern pros::Motor shooter;

// Sensors
extern pros::ADIDigitalIn triball_loaded;
extern pros::ADIDigitalIn shooter_ready;
extern pros::ADIDigitalIn auton_selector;

#endif // ROBOT_CONFIG_H
