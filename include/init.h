#include "main.h"

extern int frontLeftMotorPort;
extern int frontRightMotorPort;
extern int backLeftMotorPort;
extern int backRightMotorPort;
extern int leftIntakePort;
extern int rightIntakePort;
extern int portG;
extern int portH;

extern pros::Motor intakeLeft;
extern pros::Motor intakeRight;
extern pros::Motor driveMotorG;
extern pros::Motor driveMotorH;

extern pros::Motor *rightIntake;
extern pros::Motor *leftIntake;
extern pros::Motor *ejectorMotor;
extern pros::Motor *cyclerMotor;

extern okapi::ControllerButton cycleButton;
extern okapi::ControllerButton ejectButton;
extern okapi::ControllerButton reverseButton;
extern okapi::ControllerButton lineButton;
extern pros::ADIAnalogIn lineSensorOne;
extern pros::ADIAnalogIn lineSensorTwo;
