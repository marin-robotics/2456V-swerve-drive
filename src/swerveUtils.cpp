#include "swerveUtils.h"
#include "math.h"

/**
 * Returns the sign of the argument
 * as -1 for negative, 1 for positive,
 * or 0 for a value neither - nor +
*/
int sgn(double value) {
	return (value > 0) - (value < 0);
}
/**
 * Squares the given value,
 * accounting for its sign.
*/
double pow_with_sign(double x) {
	if (x < 0) {
	return -x * x;
	} else {
	return x * x;
	}
}
/**
 * Normalizes the given angle into
 * the range of 0-360 degrees.
*/
double normalize_angle(double angle){
	return fmod(angle + 360, 360);
}
/**
 * Calculates the real difference between two degrees
 * and returns the smaller error value. Accounts for both
 * clockwise and counter-clockwise rotation.
*/
double true_error(double initial_degree, double final_degree) { // Positive is clockwise rotation, negative is counterclockwise
    double counter_clockwise_error = normalize_angle(final_degree - initial_degree);
    double clockwise_error = normalize_angle(initial_degree - final_degree);

	// return the smaller error
    if (counter_clockwise_error <= clockwise_error) {
        return -counter_clockwise_error;
    } else {
        return clockwise_error;
    }
}

// Vector Utils
/**
 * Does exactly what you think it does.
 * @param mag The magnitude of the vector.
 * @param theta The angle (in degrees) of the vector.
*/
RectangularVector polar_to_rect(PolarVector polar_vector){
	// Converts a polar coordinate vector (magnitude, theta) in degrees to a rectangular vector.
	RectangularVector rect_vector;
    rect_vector.x = polar_vector.mag*cos((PI/180)*polar_vector.theta);
	rect_vector.y = polar_vector.mag*sin((PI/180)*polar_vector.theta);
	return rect_vector;
}

/**
 * Does exactly what you think it does.
 * @param x The X coordinate of the joystick being used to control translation.
 * @param y The Y coordinate of the joystick being used to control translation.
*/

PolarVector rect_to_polar(RectangularVector rect_vector){
	// Converts a polar coordinate vector (magnitude, theta) in degrees to a rectangular vector.
    PolarVector polar_vector;
	polar_vector.mag = hypot(rect_vector.x, rect_vector.y);
	polar_vector.theta = (180/PI)*atan2(rect_vector.y, rect_vector.x);
	return polar_vector;
}

/**
 * Does exactly what you think it does.
 * @param x The X coordinate of the joystick being used to control translation.
 * @param y The Y coordinate of the joystick being used to control translation.
*/


RectangularVector rotate_rect_vect(RectangularVector rect_vector, double adjust_amount){ // Check if field oriented first, adjust_amount = gps_status.yaw+field_orient_offset
	// get translate vector in polar coordinates
	PolarVector polar_vector = rect_to_polar(rect_vector);
	// Adjust the angle of the vector (by the field oriented net direction)
	polar_vector.theta = polar_vector.theta + adjust_amount;
	// Return the vector converted back to rectangular
	return polar_to_rect(polar_vector);
}

// RobotController method implementations

void RobotController::set_team_color(TeamColor color){
    team_color = color;
}

void RobotController::toggle_orientation(){
    if (controller_orientation == ABSOLUTE){
        controller_orientation = RELATIVE;
    } else if (controller_orientation == RELATIVE){
        controller_orientation = ABSOLUTE;
    }
}

/**
 * Resets modules to absolute zero to untangle wires at the end of a match.
*/
void RobotController::reset_modules(){
    for (int i = 0; i < swerve_size; i++){
        angle_motors[i].move_absolute((90*angle_gear_ratio), 200);
    }
}

void RobotController::update_position() { // GPS while loop updates this
    gps_status = gps.get_status();
    if (team_color == RED) {
        current_position = {-gps_status.x, -gps_status.y};
        current_angle = normalize_angle(gps_status.yaw+180);
    } else {
        current_position = {gps_status.x, gps_status.y};
        current_angle = normalize_angle(gps_status.yaw);
    }

}

// Turn to a specific angle
void RobotController::rotate_to_field_angle(CommandType orientation, double angle, double velocity) { // Just rotation vector
    if (orientation == ABSOLUTE) {
        double error = true_error(current_angle, angle);
        while (abs(int(error)) < 5) {
            update_modules(
                PolarVector {0,0},
                velocity,
                RELATIVE
                );
            error = true_error(current_angle, angle);
        }
    } else { // Same thing but the angle we are approaching is relative to the current angle
        angle = current_angle+angle;
        double error = true_error(current_angle, angle);
        while (abs(int(error)) < 5) {
            update_modules(
                PolarVector {0,0},
                velocity,
                ABSOLUTE
                );
            error = true_error(current_angle, angle);
        }
    }

}

void RobotController::move_to_position(Point target, double move_velocity, double angle, double yaw_velocity){ // Just translate vector
    move_velocity = std::abs(move_velocity);
    RectangularVector rect_error_vector;
    PolarVector error_vector;
    double angle_error = 0;
    double rotation_magnitude = 0;

    do {
        rect_error_vector = {target.x-current_position.x, target.y-current_position.y};
        error_vector = rect_to_polar(rect_error_vector); // Get the vector from the current position to the destination
        if (yaw_velocity != 0) { // Only compute if yaw_velocity is given (if the bot should turn)
            angle_error = true_error(current_angle, angle);
            rotation_magnitude = angle_error * yaw_velocity * (127.0/180.0);
        }

        update_modules(
            PolarVector {move_velocity, error_vector.theta},
            rotation_magnitude,
            ABSOLUTE
        );
        pros::delay(20); // A delay to prevent CPU overload, adjust as needed
    } while ((error_vector.mag > 10) || (yaw_velocity != 0 && std::abs(angle_error) > 5)); // Check angle only if yaw_velocity is given
}

void RobotController::strafe(CommandType orientation, double angle, double velocity){ // Just translate vector
    update_modules(
        PolarVector {velocity, angle},
        0,
        orientation);
}

void RobotController::autonomous_drive(CommandType orientation, double velocity, double angle, double yaw, int millis_duration){ // Just translate vector
    int current_time = pros::millis();
    while (pros::millis()-current_time < millis_duration){
        update_modules(
            PolarVector {velocity, angle},
            yaw,
            orientation);
    }
    update_modules( // Turn off modules
            PolarVector {0, angle},
            0,
            orientation);
}

void RobotController::autonomous_spin(CommandType orientation, double yaw, int millis_duration){
    int current_time = pros::millis();
    while (pros::millis()-current_time < millis_duration){
        update_modules(
            PolarVector {0, 0},
            yaw,
            orientation);
    }
    update_modules( // Turn off modules
            PolarVector {0, 0},
            0,
            orientation);
}

void RobotController::autonomous(int route){
    // Set the team color/side for field oriented controls
    if (route < 6/2) {
        set_team_color(RED);
    } else {
        set_team_color(BLUE);
    }
    // Choose route
    if (route == 0 || route == 3) { // Defense

        // old routine
        autonomous_drive(RELATIVE, 100, 90, 0, 1000);
        for (int i = 0; i < 3; i++){ // Ram preload into goal 3 times
            autonomous_drive(RELATIVE, 300, -90, 0, 250);
            pros::delay(300);
            autonomous_drive(RELATIVE, 500, 90, 0, 500);
            pros::delay(300);
        }

        // new routine (needs refinement but will be better)
        // for (int i = 0; i < 3; i++){ // Ram preload into goal 3 times
        //     autonomous_drive(RELATIVE, 100, 90, 0, 1000); // go forward
        //     pros::delay(300);
        //     autonomous_drive(RELATIVE, 100, -90, 0, 1000); // back up
        //     pros::delay(300);
        //     autonomous_drive(ABSOLUTE, 100, 90, 0, 500); // move sideways
        // }

        reset_modules();
        pros::delay(2500);
    } else if (route == 1 || route == 4) { // Offense
        autonomous_drive(RELATIVE, 100, 90, 0, 1000);
        for (int i = 0; i < 3; i++){ // Ram preload into goal 3 times
            autonomous_drive(RELATIVE, 300, -90, 0, 250);
            pros::delay(300);
            autonomous_drive(RELATIVE, 500, 90, 0, 500);
            pros::delay(300);
        }
        reset_modules();
        pros::delay(2500);
    }
    // route 2 & 5 are 'do nothing'

}

void RobotController::manual_drive(double left_x, double left_y, double right_x) {
    update_modules(
        PolarVector {hypot(left_x, left_y), atan2(left_y, left_x)*(180/PI)},
        right_x,
        controller_orientation
    );
}

void RobotController::orbit_circle(Point center, double radius, double left_x, double right_x){ // Run this as a task
    Point closest_point; // Calculate
    Point new_point; // Calculate
    double perpendicular_angle; // Calculate
    double target_angle = normalize_angle(perpendicular_angle + (right_x*45));
    double modifier;
    double move_velocity = left_x * modifier;
    // Basically calculate the point closest to the perimeter of this circle.
    // Then choose the goal as a point left or right of that circle based on the left_x stick
    // Then apply the move_to_position function
    //move_to_position(new_point, move_velocity, target_angle, right_x*127);
}
