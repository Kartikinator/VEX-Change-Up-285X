#include "main.h"
#pragma once

int frontLeftMotorPort = 1;
int frontRightMotorPort = 11;
int backLeftMotorPort = -12;
int backRightMotorPort = 2;
int leftIntakePort = 6;
int rightIntakePort = 20; //6
int indexer = 10;
int rollers = 9;

pros::Motor intakeLeft(leftIntakePort, pros::E_MOTOR_GEARSET_18 );
pros::Motor intakeRight(rightIntakePort, pros::E_MOTOR_GEARSET_18 );

pros::Motor indexerMotorMotor(indexer, pros::E_MOTOR_GEARSET_06 );
pros::Motor rollersMotorMotor(rollers, pros::E_MOTOR_GEARSET_18 );

pros::Motor *rightIntake = &intakeRight;
pros::Motor *leftIntake = &intakeLeft;
pros::Motor *indexerMotor = &indexerMotorMotor;
pros::Motor *rollersMotor = &rollersMotorMotor;

okapi::ControllerButton cycleButton = okapi::ControllerDigital::L2;
okapi::ControllerButton ejectButton = okapi::ControllerDigital::R1;
okapi::ControllerButton reverseButton = okapi::ControllerDigital::L1;
okapi::ControllerButton lineButton = okapi::ControllerDigital::R2;
pros::ADIAnalogIn lineSensorOne ('A');
