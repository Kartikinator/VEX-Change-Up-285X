#include "main.h"

// MOTOR PORTS
int DRIVE_FRONT_LEFT = 11;
int DRIVE_FRONT_RIGHT = -2;
int DRIVE_BACK_RIGHT = -10;
int DRIVE_BACK_LEFT = 20;

int STRAFE_DRIVE_FRONT_LEFT = 11;
int STRAFE_DRIVE_FRONT_RIGHT = 2;
int STRAFE_DRIVE_BACK_RIGHT = -10;
int STRAFE_DRIVE_BACK_LEFT = -20;

int INDEXER = 9;
int MAIN_INTAKE = 5;
int LEFT_INTAKE = 16;
int RIGHT_INTAKE = 17;

const double Deadzone = 0.1;

bool front = false;
std::string autoColor = "";

void on_right_button()
{
	front = true;
	autoColor = "Red";
	pros::lcd::set_text(1, "Front Red Autonomous Initiated");
}

void on_left_button()
{
	front = true;
	autoColor = "Blue";
	pros::lcd::set_text(1, "Front Blue Autonomous Initiated");
}

void on_center_button()
{

	if (autoColor == "Blue" && front)
	{
		pros::lcd::set_text(1, "Back Blue Autonomous Initiated");
		front = false;
	}
	else if (autoColor == "Red" && front)
	{
		pros::lcd::set_text(1, "Back Red Autonomous Initiated");
		front = false;
	}
	else if (!front & autoColor != "")
	{
		pros::lcd::set_text(1, "Front " + autoColor + " Autonomous Initiated");
		front = true;
	}
	else
	{
		pros::lcd::set_text(1, "Please select a side first");
	}
}
pros::Motor main_intake(MAIN_INTAKE, true);
pros::Motor indexer(INDEXER);
pros::Motor left_intake(LEFT_INTAKE);
pros::Motor right_intake(RIGHT_INTAKE, true);

Motor drive_b_l(DRIVE_BACK_LEFT);
Motor drive_b_r(DRIVE_BACK_RIGHT);
Motor drive_f_l(DRIVE_FRONT_LEFT);
Motor drive_f_r(DRIVE_FRONT_RIGHT);

okapi::MotorGroup driveL({DRIVE_FRONT_LEFT, DRIVE_BACK_LEFT});
okapi::MotorGroup driveR({DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT});

pros::ADIAnalogIn line_sensor('A');

pros::ADIAnalogIn bumper('C');

pros::Imu imuSensor(8);

ADIEncoder leftencoder('G', 'H');
ADIEncoder rightencoder('C', 'D');

// Declaring Chassis ---
std::shared_ptr<OdomChassisController> odomchas =
	ChassisControllerBuilder()
		.withMotors(DRIVE_FRONT_LEFT, DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT, DRIVE_BACK_LEFT)
		//.withSensors(leftencoder, rightencoder, middleencoder)
		.withGains(
			{0.002, 0.00001, 0}, // Distance controller gains
			{0.007, 0, 0},		 // Turn controller gains
			{0.002, 0, 0.00006}	 // Angle controller gains (helps drive straight)
			)
		.withSensors(
			ADIEncoder{'G', 'H'},
			ADIEncoder{'C', 'D', true})
		.withDimensions(AbstractMotor::gearset::green, {{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
		.withOdometry()
		.buildOdometry();

std::shared_ptr<AsyncMotionProfileController> moveProfile =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
			2.0, // Maximum linear velocity of the Chassis in m/s
			4.0, // Maximum linear acceleration of the Chassis in m/s/s
			8.0	 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(odomchas)
		.buildMotionProfileController();

std::shared_ptr<OdomChassisController> strafeodomchas =
	ChassisControllerBuilder()
		.withMotors(STRAFE_DRIVE_FRONT_LEFT, STRAFE_DRIVE_FRONT_RIGHT, STRAFE_DRIVE_BACK_RIGHT, STRAFE_DRIVE_BACK_LEFT)
		//.withSensors(leftencoder, rightencoder, middleencoder)
		.withGains(
			{0.002, 0.00001, 0}, // Distance controller gains
			{0.007, 0, 0},		 // Turn controller gains
			{0.002, 0, 0.00006}	 // Angle controller gains (helps drive straight)
			)
		.withDimensions(AbstractMotor::gearset::green, {{3.75_in, 15_in}, imev5GreenTPR})
		.withOdometry()
		.buildOdometry();

std::shared_ptr<AsyncMotionProfileController> strafeProfile =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
			2.0, // Maximum linear velocity of the Chassis in m/s
			4.0, // Maximum linear acceleration of the Chassis in m/s/s
			8.0	 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(strafeodomchas)
		.buildMotionProfileController();

auto xModel = std::dynamic_pointer_cast<XDriveModel>(odomchas->getModel());

Controller controller;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "285X Autonomous Selector");

	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);

	imuSensor.reset();

	moveProfile->generatePath(
		{{0_ft, 0_ft, 0_deg},
		 {2.03_ft, 0_ft, 0_deg}},
		"move1");

	moveProfile->generatePath(
		{{0_ft, 0_ft, 0_deg},
		 {0.5_ft, 0_ft, 0_deg}},
		"move2");

	moveProfile->generatePath(
		{{0_ft, 0_ft, 0_deg},
		 {1_ft, 0_ft, 0_deg}},
		"move3");

	strafeProfile->generatePath(
		{{0_ft, 0_ft, 0_deg},
		 {2.75_ft, 0_ft, 0_deg}},
		"strafe1");

	strafeProfile->generatePath(
		{{0_ft, 0_ft, 0_deg},
		 {2.75_ft, 0_ft, 0_deg}},
		"strafe2");

	strafeProfile->generatePath(
		{{0_ft, 0_ft, 0_deg},
		 {1.2_ft, 0_ft, 0_deg}},
		"strafe3");

	strafeProfile->generatePath(
		{{0_ft, 0_ft, 0_deg},
		 {3.3_ft, 0_ft, 0_deg}},
		"strafe4");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

/* 
	Personal notes:
		- ft: negative for backward, positive for forward
		- only forward / backward
		- 20 is about the distance from the first part to the goal
		- keep move values between 0 - 20, absolute
*/

const double GLOBAL_kP = 4;
const double GLOBAL_kI = 0.00001;
const double GLOBAL_kD = 0.8;

double deg = 0;
bool absolute = true;

void calibrate()
{
	imuSensor.reset();
}

void turn(double degrees)
{

	driveR.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	driveL.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	double thetaI = imuSensor.get_heading();
	double thetaF = degrees;

	double sensorValue = thetaI;
	double turnTarget = thetaF;

	double deltaI = abs(thetaF - thetaI);

	if (deltaI > 180)
	{
		if (thetaF > 180)
		{
			turnTarget = thetaF - 360;
		}
		else
		{
			turnTarget = thetaF;
		}

		if (thetaI > 180)
		{
			sensorValue = thetaI - 360;
		}
		else
		{
			sensorValue = thetaI;
		}
	}

	double error = turnTarget - sensorValue;
	double oldError = error;
	double sumError = 0;

	bool TURN_NOT_FINISH = true;
	while (TURN_NOT_FINISH)
	{
		sensorValue = imuSensor.get_heading();

		if (deltaI > 180)
		{
			if (sensorValue > 180)
			{
				sensorValue = sensorValue - 360;
			}
		}

		// PROPORTIONAL
		error = turnTarget - sensorValue;
		// DERIVATIVE
		double changeInError = error - oldError;
		// INTEGRAL
		if (abs(error) < 50)
		{
			sumError += error;
		}
		else
		{
			sumError = 0; // might be += 0?
		}

		//P, I, D
		double P = GLOBAL_kP * error;
		double I = GLOBAL_kI * sumError;
		double D = GLOBAL_kD * changeInError;

		double sum = P + I + D;

		driveL.moveVelocity(sum);
		driveR.moveVelocity(-sum);

		oldError = error;
		double errorThreshold = 1.5;
		double velocityThreshold = 2;

		TURN_NOT_FINISH = !((abs(error) < errorThreshold) && (abs(changeInError) < velocityThreshold));
	}
	driveL.moveVoltage(0);
	driveR.moveVoltage(0);
}

void autonomous()
{

	/*
		NOTES:
			- Intakes: - is outtake, + is intake, 0 is stop
			- Indexer: - is shoot, + is pull in, 0 is stop
			- Main intake: - is pull in, + is push up, 0 is stop
			- Turn: - is counterclockwise, + is clockwise
	*/

	// initialize
	turn(-45);

	// ---- bottom left corner ----
	strafeProfile->setTarget("strafe3");
	strafeProfile->waitUntilSettled();

	left_intake.move_velocity(-150);
	right_intake.move_velocity(-150);
	indexer.move_velocity(-150);
	main_intake.move_velocity(150);
	pros::delay(400);

	left_intake.move_velocity(0);
	right_intake.move_velocity(0);
	indexer.move_velocity(0);
	main_intake.move_velocity(0);
	pros::delay(50);

	// ---- middle goal (???) ----
	left_intake.move_velocity(200);
	right_intake.move_velocity(200);

	moveProfile->setTarget("move3");
	moveProfile->waitUntilSettled();

	main_intake.move_velocity(200);
	indexer.move_velocity(-200);
	pros::delay(1100);
	left_intake.move_velocity(-200);
	right_intake.move_velocity(-200);
	main_intake.move_velocity(200);
	indexer.move_velocity(-200);
	moveProfile->setTarget("move3", true);
	moveProfile->waitUntilSettled();

	left_intake.move_velocity(0);
	right_intake.move_velocity(0);
	main_intake.move_velocity(0);
	indexer.move_velocity(0);

	// ---- center goal ----
	turn(105);

	moveProfile->setTarget("move1");
	moveProfile->waitUntilSettled();

	turn(90);

	left_intake.move_velocity(200);
	right_intake.move_velocity(200);

	strafeProfile->setTarget("strafe4");
	strafeProfile->waitUntilSettled();

	left_intake.move_velocity(0);
	right_intake.move_velocity(0);

	strafeProfile->setTarget("strafe4", true);
	strafeProfile->waitUntilSettled();
}

/*
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	/*
		NOTES:
			- Intakes: - is outtake, + is intake, 0 is stop
			- Indexer: - is shoot, + is pull in, 0 is stop
			- Main intake: - is pull in, + is push up, 0 is stop
			- Turn: - is counterclockwise, + is clockwise
	*/

	pros::motor_brake_mode_e_t mode = pros::E_MOTOR_BRAKE_HOLD;

	ControllerDigital a{DIGITAL_A};
	ControllerButton a_button(a);

	ControllerDigital y{DIGITAL_Y};
	ControllerButton y_button(y);

	ControllerDigital r1{DIGITAL_R1};
	ControllerButton right1(r1);

	ControllerDigital r2{DIGITAL_R2};
	ControllerButton right2(r2);

	ControllerDigital l1{DIGITAL_L1};
	ControllerButton left1(l1);

	ControllerDigital l2{DIGITAL_L2};
	ControllerButton left2(l2);

	ControllerDigital rightarrow{DIGITAL_RIGHT};
	ControllerButton right_arrow(rightarrow);

	bool detected = false;

	while (true)
	{
		pros::lcd::set_text(3, "Right Motor Temperature: N/A");
		pros::lcd::set_text(4, "Battery Temperature: " + std::to_string(int(pros::battery::get_temperature())) + " C");
		pros::lcd::set_text(5, "Battery Current: " + std::to_string(pros::battery::get_current()));

		if (detected == false)
		{
			if (left1.isPressed()) // ONLY shoot with indexer (LT 1)
			{
				main_intake.move_velocity(0);
				indexer.move_velocity(-200);
				left_intake.move_velocity(0);
				right_intake.move_velocity(0);
			}
			else if (right1.isPressed()) // Intake all balls up to line sensor, then only run intakes (RT 1)
			{
				if (line_sensor.get_value() > 1500)
				{
					main_intake.move_velocity(200);
					indexer.move_velocity(-100);
					left_intake.move_velocity(200);
					right_intake.move_velocity(200);
				}
				else
				{
					main_intake.move_velocity(0);
					indexer.move_velocity(0);
					left_intake.move_velocity(150);
					right_intake.move_velocity(150);
				}
			}
			else if (left2.isPressed()) // ONLY outtake stuff inside intakes (LT 2)
			{
				left_intake.move_velocity(-200);
				right_intake.move_velocity(-200);
			}
			else if (right2.isPressed()) // ONLY shoot balls in tower (no intakes) (RT 2)
			{
				main_intake.move_velocity(200);
				indexer.move_velocity(-200);
			}
			else if (y_button.isPressed()) // Cycle balls (full shooting motion) (R SCUFF)
			{
				main_intake.move_velocity(150);
				indexer.move_velocity(-200);
				left_intake.move_velocity(175);
				right_intake.move_velocity(175);
			}
			else if (right_arrow.isPressed()) // Eject all balls through tower back (L SCUFF)
			{
				main_intake.move_velocity(200);
				indexer.move_velocity(200);
				left_intake.move_velocity(200);
				right_intake.move_velocity(200);
			}
			else if (a_button.isPressed()) // Eject all balls through intakes (A BTN)
			{
				main_intake.move_velocity(-200);
				indexer.move_velocity(200);
				left_intake.move_velocity(-200);
				right_intake.move_velocity(-200);
			}
			else // Stop motors that might be running
			{
				main_intake.move_velocity(0);
				indexer.move_velocity(0);
				left_intake.move_velocity(0);
				right_intake.move_velocity(0);
			}
		}

		xModel->xArcade(
			controller.getAnalog(ControllerAnalog::leftX),	// side to side movement (L Stick)
			controller.getAnalog(ControllerAnalog::leftY),	// front & back (L Stick)
			controller.getAnalog(ControllerAnalog::rightX), // rotation (R Stick)
			Deadzone
		);

		pros::delay(10);
	}
}
