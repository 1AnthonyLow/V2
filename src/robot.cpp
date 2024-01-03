#include "robot.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor lf(8, MOTOR_GEARSET_06, true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lm(9, MOTOR_GEARSET_06, true, MOTOR_ENCODER_DEGREES);
pros::Motor lb(10, MOTOR_GEARSET_06, true, MOTOR_ENCODER_DEGREES);

pros::Motor rf(4, MOTOR_GEARSET_06, false, MOTOR_ENCODER_DEGREES);
pros::Motor rm(2, MOTOR_GEARSET_06, false, MOTOR_ENCODER_DEGREES);
pros::Motor rb(3, MOTOR_GEARSET_06, false, MOTOR_ENCODER_DEGREES);

pros::Motor_Group left({lf, lm, lb});
pros::Motor_Group right({rf, rm, rb});

pros::Motor slapper(1, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);

pros::Motor intake(1, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalOut bar_hang('A', false);
pros::ADIDigitalOut side_hang('B', false);
pros::ADIDigitalOut vert_wings('C', false);
pros::ADIDigitalOut horiz_wings('D', false);

