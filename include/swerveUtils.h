#ifndef SWERVE_UTILS_H
#define SWERVE_UTILS_H

#include <sys/types.h>
#include <vector>
#include <cmath>
#include <string>
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robot_config.h"

// The necessary constants and types
const double PI = 3.14159265358979323846;
using std::vector; // unused

enum TeamColor{
    BLUE,
    RED
};

enum CommandType {
    RELATIVE,
    ABSOLUTE
};

struct Point {
    double x;
    double y;
};

struct RectangularVector {
    double x;
    double y;
};

struct PolarVector {
    double mag;
    double theta;
};

struct SwerveModuleTelemetry {
    std::vector<double> angleErrors;
    std::vector<double> angleVelocities;
    std::vector<double> primaryVelocities;
};

// Function declarations
int sgn(double value);
double pow_with_sign(double x);
double normalize_angle(double angle);
double true_error(double initial_degree, double final_degree);
RectangularVector polar_to_rect(PolarVector polar_vector);
PolarVector rect_to_polar(RectangularVector rect_vector);
RectangularVector rotate_rect_vect(RectangularVector rect_vector, double adjust_amount);
SwerveModuleTelemetry update_modules(PolarVector polar_translate_vector, double yaw_magnitude, CommandType orientation);

class RobotController {
public:

    TeamColor team_color;
    Point current_position;             // Robot's current position on the field
    double current_angle;               // Robot's current field orientation in degrees
    bool currently_shooting = false;    //
    //bool blocker_state = false;         // Whether the blocker is enabled
    CommandType controller_orientation = RELATIVE; // Robot's current control mode

    // variables
    int swerve_size = 4;
    double angle_gear_ratio = 5.5/3;

    void set_team_color(TeamColor color);
    void toggle_orientation();
    void reset_modules();
    void update_position();
    void rotate_to_field_angle(CommandType orientation, double angle, double velocity=127);
    void move_to_position(Point target, double move_velocity, double angle = 0, double yaw_velocity = 0);
    void strafe(CommandType orientation, double angle, double velocity);
    void autonomous_drive(CommandType orientation, double velocity, double angle, double yaw, int millis_duration);
    void autonomous_spin(CommandType orientation, double yaw, int millis_duration);
    void autonomous(int route);
    void manual_drive(double left_x, double left_y, double right_x);
    void orbit_circle(Point center, double radius, double left_x=0, double right_x=0);
};

#endif // SWERVE_UTILS_H
