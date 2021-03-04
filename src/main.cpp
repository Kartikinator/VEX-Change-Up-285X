<<<<<<< HEAD
#include "main.h"
// #include "devices.cpp"
// #include "C:\Users\srika_5auwk87\Documents\VCU-285X-AUTONOMOUS\VEX-Change-Up-285X\src\AutonFiles\skills.cpp"


// MOTOR PORTS
int DRIVE_FRONT_LEFT = -1;
int DRIVE_FRONT_RIGHT = 12;
int DRIVE_BACK_RIGHT = -11;
int DRIVE_BACK_LEFT = 2;

int INDEXER = 10;
int MAIN_INTAKE = 9;
int LEFT_INTAKE = 8;
int RIGHT_INTAKE = 20;


bool front = false;
std::string autoColor = "";

void on_right_button() {
	front = true;
	autoColor = "Red";
	pros::lcd::set_text(1, "Front Red Autonomous Initiated");
}

void on_left_button() {
	front = true;
	autoColor = "Blue";
	pros::lcd::set_text(1, "Front Blue Autonomous Initiated");
}

void on_center_button() {

	if (autoColor == "Blue" && front) {
		pros::lcd::set_text(1, "Back Blue Autonomous Initiated");
		front = false;
	} else if (autoColor == "Red" && front) {
		pros::lcd::set_text(1, "Back Red Autonomous Initiated");
		front = false;
	} else if (!front & autoColor != "") {
		pros::lcd::set_text(1, "Front " + autoColor + " Autonomous Initiated");
		front = true;
	} else {
		pros::lcd::set_text(1, "Please select a side first");
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "285X Autonomous Selector");

	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);

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
void turn(ADIEncoder encoder, int amount, okapi::MotorGroup left, okapi::MotorGroup right) {
 	left.moveVelocity(-200);
 	right.moveVelocity(200);
 }

void autonomous() {
	pros::Motor main_intake (MAIN_INTAKE, true);
	pros::Motor indexer (INDEXER);
	pros::Motor left_intake (LEFT_INTAKE);
	pros::Motor right_intake (RIGHT_INTAKE, true);

	pros::ADIAnalogIn limit_switch ('B');

	pros::ADIAnalogIn bumper ('C');

	pros::ADIEncoder encoder ('F', 'E');

	// Declaring Chassis ---
	std::shared_ptr<OdomChassisController> odomchas =
	    ChassisControllerBuilder()
	        .withMotors(DRIVE_FRONT_LEFT, DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT, DRIVE_BACK_LEFT)
	        .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
					.withOdometry()
	        .buildOdometry();


	std::shared_ptr<AsyncMotionProfileController> profileController =
	  AsyncMotionProfileControllerBuilder()
	    .withLimits({
	      1.0, // Maximum linear velocity of the Chassis in m/s
	      2.0, // Maximum linear acceleration of the Chassis in m/s/s
	      5.0 // Maximum linear jerk of the Chassis in m/s/s/s
	    })
	    .withOutput(odomchas)
	    .buildMotionProfileController();

		// Declaring Motors ---
		// pros::Motor TOP_RIGHT (12, true);
		// pros::Motor TOP_LEFT (DRIVE_FRONT_LEFT);
		// pros::Motor BACK_RIGHT (11, true);
		// pros::Motor BACK_LEFT (DRIVE_BACK_LEFT);

		//Drive Wheels
		// okapi::MotorGroup left({DRIVE_FRONT_LEFT, DRIVE_BACK_LEFT});
		// okapi::MotorGroup right({DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT});

		indexer.move_velocity(200);
		pros::delay(100);
		indexer.move_velocity(0);

		profileController->generatePath({
			{0_ft, 0_ft, 0_deg},
			{1.5_ft, 0_ft, 50_deg}},
			"straight1"
		);

		left_intake.move_velocity(200);
		right_intake.move_velocity(200);

		profileController->setTarget("straight1");
		profileController->waitUntilSettled();

		profileController->generatePath({
			{0_ft, 0_ft, 0_deg},
			{0.5_ft, 0_ft, 0_deg}},
			"littlemove"
		);

		profileController->setTarget("littlemove");
		profileController->waitUntilSettled();


		// odomchas->turnAngle(-35_deg);

		indexer.move_velocity(-200);

		pros::delay(750);

		indexer.move_velocity(0);

		left_intake.move_velocity(0);
		right_intake.move_velocity(0);

		// Away from first

		profileController->generatePath({
			{0_ft, 0_ft, 50_deg},
			{1_ft, 0_ft, 0_deg}},
			"straight2"
		);

		profileController->setTarget("straight1", true);
		profileController->waitUntilSettled();

		// odomchas->turnAngle(-100_deg);

		// Approach 2nd goal

		profileController->generatePath({
			{0_ft, 0_ft, 0_deg},
			{5_ft, 0_ft, 0_deg}},
			"straight3"
		);

		profileController->setTarget("straight3");
		profileController->waitUntilSettled();

		left_intake.move_velocity(200);
		right_intake.move_velocity(200);

		indexer.move_velocity(-200);
		main_intake.move_velocity(200);

		pros::delay(750);

		indexer.move_velocity(0);
		main_intake.move_velocity(0);

		// Away from 2nd goal

		profileController->generatePath({
			{0_ft, 0_ft, 0_deg},
			{1.5_ft, 0_ft, 0_deg}},
			"straight4"
		);

		profileController->setTarget("straight4", true);
		profileController->waitUntilSettled();

		odomchas->turnAngle(-25_deg);

		// Approach 3rd goal

		profileController->generatePath({
			{0_ft, 0_ft, 0_deg},
			{6_ft, 0_ft, 0_deg}},
			"straight5"
		);

		profileController->setTarget("straight5");
		profileController->waitUntilSettled();







		// int turnValue = encoder.get_value() + 100;
		//
		// while(encoder.get_value() < turnValue) {
		// 	left.moveVelocity(200);
		// 	right.moveVelocity(-200);
		// }



		//Autonomous start
		// profileController->generatePath({
		// 	{0_ft, 0_ft, 0_deg},
		// 	{1_ft, 2_ft, 0_deg}},
		// 	"curve"
		// );
		//
		// profileController->setTarget("curve");
		// profileController->waitUntilSettled();
		//
		// pros::delay(10000);

		// profileController->generatePath({
		// 	{0_ft, 0_ft, 0_deg},
		// 	{1.3_ft, 0_ft, 0_deg}},
		// 	"Straight1"
		// );
		//
		// profileController->generatePath({
		// 	{0_ft, 0_ft, 0_deg},
		// 	{1.5_ft, 0_ft, 0_deg}},
		// 	"Straight2"
		// );
		//
		// left_intake.move_velocity(200);
		// right_intake.move_velocity(200);
		//
		// profileController->setTarget("Straight1");
		// profileController->waitUntilSettled();
		//
		//
		// odomchas->turnAngle(-85_deg);
		//
		// profileController->setTarget("Straight2");
		// profileController->waitUntilSettled();
		//
		// indexer.move_velocity(-200);
		//
		// pros::delay(550);
		//
		// while(limit_switch.get_value() >11) {
		// 	main_intake.move_velocity(150);
		// }
		//
		// left_intake.move_velocity(-100);
		// right_intake.move_velocity(-100);
		//
		// indexer.move_velocity(0);
		// main_intake.move_velocity(0);
		//
		// profileController->generatePath({
		// 	{0_ft, 0_ft, 0_deg},
		// 	{3_ft, 0_ft, 0_deg}},
		// 	"Straight3"
		// );
		//
		// profileController->setTarget("Straight3", true);
		// profileController->waitUntilSettled();
		//
		// left_intake.move_velocity(200);
		// right_intake.move_velocity(200);
		//
		// odomchas->turnAngle(-94_deg);
		//
		// left_intake.move_velocity(0);
		// right_intake.move_velocity(0);
		//
		// // Approach Middle Goal
		//
		// profileController->generatePath({
		// 	{0_ft, 0_ft, 0_deg},
		// 	{3.5_ft, 0_ft, 0_deg}},
		// 	"Straight4"
		// );
		//
		// profileController->setTarget("Straight4");
		// profileController->waitUntilSettled();
		//
		// main_intake.move_velocity(200);
		// indexer.move_velocity(-200);
		// // left_intake.move_velocity(200);
		// // right_intake.move_velocity(200);
		//
		// pros::delay(1000);
		//
		// profileController->generatePath({
		// 	{0_ft, 0_ft, 0_deg},
		// 	{1.5_ft, 0_ft, 0_deg}},
		// 	"Straight5"
		// );
		//
		// profileController->setTarget("Straight5", true);
		// profileController->waitUntilSettled();
		//
		// odomchas->turnAngle(-38_deg);
		//
		// // Approach last goal
		//
		// profileController->generatePath({
		// 	{0_ft, 0_ft, 0_deg},
		// 	{6_ft, 0_ft, 0_deg}},
		// 	"Straight6"
		// );
		//
		// left_intake.move_velocity(200);
		// right_intake.move_velocity(200);
		//
		// profileController->setTarget("Straight6");
		// profileController->waitUntilSettled();
		//
		// odomchas->turnAngle(40_deg);
		//
		// profileController->setTarget("Straight1");
		// profileController->waitUntilSettled();
		//
		// pros::delay(750);
		//
		// profileController->setTarget("Straight1", true);
		// profileController->waitUntilSettled();

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

void opcontrol() {

	pros::Motor main_intake (MAIN_INTAKE);
	pros::Motor indexer (INDEXER);
	pros::Motor left_intake (LEFT_INTAKE);
	pros::Motor right_intake (RIGHT_INTAKE);
	pros::ADIAnalogIn line_sensor ('A');
	pros::ADIAnalogIn limit_switch ('B');

	pros::ADIEncoder encoder ('F', 'E');

	std::shared_ptr<ChassisController> drive =
	    ChassisControllerBuilder()
	        .withMotors(DRIVE_FRONT_LEFT, DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT, DRIVE_BACK_LEFT)
	        .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
	        .build();

	Controller controller;

	pros::motor_brake_mode_e_t mode = pros::E_MOTOR_BRAKE_HOLD;
	ControllerDigital a_button {17};
	ControllerButton button(a_button);

	ControllerDigital r1 {8};
	ControllerButton right1(r1);

	ControllerDigital r2 {9};
	ControllerButton right2(r2);

	ControllerDigital l1 {DIGITAL_L1};
	ControllerButton left1(l1);

	ControllerDigital l2 {DIGITAL_L2};
	ControllerButton left2(l2);

	bool detected = false;

  while (true) {
		pros::lcd::set_text(2, "Encoder: " + std::to_string(encoder.get_value())); //  + " C");
		pros::lcd::set_text(3, "Right Motor Temperature: N/A"); // + std::to_string(int(right_wheels.get_temperature())) + " C");
		pros::lcd::set_text(4, "Battery Temperature: " + std::to_string(int(pros::battery::get_temperature())) + " C");
		pros::lcd::set_text(5, "Battery Current: " + std::to_string(pros::battery::get_current()));
		pros::lcd::set_text(6, "Line Sensor: " + std::to_string(line_sensor.get_value()));

		if (detected == false) {

		if (right1.isPressed()) {
			main_intake.move_velocity(0);
			indexer.move_velocity(-200);
			left_intake.move_velocity(0);
			right_intake.move_velocity(0);
		}
		else if (button.isPressed()) {
				main_intake.move_velocity(-200);
				indexer.move_velocity(-200);
		}
		else if (left1.isPressed()) {
				main_intake.move_velocity(200);
				indexer.move_velocity(200);
				left_intake.move_velocity(-200);
				right_intake.move_velocity(200);
		}
		else if (left2.isPressed()) {
				main_intake.move_velocity(-200);
				indexer.move_velocity(200);
				left_intake.move_velocity(200);
				right_intake.move_velocity(-200);
		}
		else if (right2.isPressed()) {

		if (limit_switch.get_value() >11) {
		main_intake.move_velocity(-100);
		indexer.move_velocity(-50);
		left_intake.move_velocity(150);
		right_intake.move_velocity(-150);
	} else {
		main_intake.move_velocity(0);
		indexer.move_velocity(0);
		left_intake.move_velocity(150);
		right_intake.move_velocity(-150);
	}

	} else if (right1.isPressed() && right2.isPressed()) {
			main_intake.move_velocity(-200);
			indexer.move_velocity(-200);
			left_intake.move_velocity(150);
			right_intake.move_velocity(-150);

		}

		else {
			main_intake.move_velocity(0);
			indexer.move_velocity(0);
			left_intake.move_velocity(0);
			right_intake.move_velocity(0);
		}
	}


		// if (right2.isPressed()) {
		// 	pros::lcd::set_text(7, "You are holding the button!");
		// 	main_intake.move_velocity(50);
		// 	secondary_intake.move_velocity(50);
		// } else {
		// 	main_intake.move_velocity(0);
		// 	secondary_intake.move_velocity(0);
		// }



		drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                            controller.getAnalog(ControllerAnalog::rightX));

    pros::delay(10);
  }
}
=======
#include "main.h"

bool front = false;
std::string autoColor = "";

void on_right_button() {
	front = true;
	autoColor = "Red";
	pros::lcd::set_text(1, "Front Red Autonomous Initiated");
}

void on_left_button() {
	front = true;
	autoColor = "Blue";
	pros::lcd::set_text(1, "Front Blue Autonomous Initiated");
}

void on_center_button() {

	if (autoColor == "Blue" && front) {
		pros::lcd::set_text(1, "Back Blue Autonomous Initiated");
		front = false;
	} else if (autoColor == "Red" && front) {
		pros::lcd::set_text(1, "Back Red Autonomous Initiated");
		front = false;
	} else if (!front & autoColor != "") {
		pros::lcd::set_text(1, "Front " + autoColor + " Autonomous Initiated");
		front = true;
	} else {
		pros::lcd::set_text(1, "Please select a side first");
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "285X Autonomous Selector");

	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);

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
void autonomous() {
	pros::Motor left_wheels (1);
  pros::Motor right_wheels (11, true);
	// if (autoColor == "Red" && front==true) {
	// 	right_wheels.move_relative(1000, 200);
	// 	left_wheels.move_relative(1000, 200);
	//
	// }
	right_wheels.move_velocity(50);
	pros::delay(10000);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Motor left_wheels (1);
  pros::Motor right_wheels (11, true); // This reverses the motor

	pros::Motor main_intake (9);
	pros::Motor indexer (10);
	pros::Motor left_intake (6);
	pros::Motor right_intake (20);
	pros::ADIAnalogIn line_sensor ('A');


	std::shared_ptr<ChassisController> drive =
	    ChassisControllerBuilder()
	        .withMotors(1, -12, 11, 2)
	        .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
	        .build();

	Controller controller;

	pros::motor_brake_mode_e_t mode = pros::E_MOTOR_BRAKE_HOLD;
	ControllerDigital a_button {17};
	ControllerButton button(a_button);

	ControllerDigital r1 {8};
	ControllerButton right1(r1);

	ControllerDigital r2 {9};
	ControllerButton right2(r2);

	ControllerDigital l1 {DIGITAL_L1};
	ControllerButton left1(l1);

  while (true) {
		pros::lcd::set_text(2, "Left Motor Temperature: " + std::to_string(int(left_wheels.get_temperature())) + " C");
		pros::lcd::set_text(3, "Right Motor Temperature: " + std::to_string(int(right_wheels.get_temperature())) + " C");
		pros::lcd::set_text(4, "Battery Temperature: " + std::to_string(int(pros::battery::get_temperature())) + " C");
		pros::lcd::set_text(5, "Battery Current: " + std::to_string(pros::battery::get_current()));
		pros::lcd::set_text(6, "Line Sensor: " + std::to_string(line_sensor.get_value()));

		if (right1.isPressed()) {
			pros::lcd::set_text(7, "You are holding the button!");
			main_intake.move_velocity(-200);
			indexer.move_velocity(-600);
			left_intake.move_velocity(200);
			right_intake.move_velocity(-200);

		}
		else if (right2.isPressed()) {
				main_intake.move_velocity(-200);
				left_intake.move_velocity(200);
				right_intake.move_velocity(-200);
		}
		else if (left1.isPressed()) {
				main_intake.move_velocity(200);
				indexer.move_velocity(600);
				left_intake.move_velocity(-200);
				right_intake.move_velocity(200);
		}
		else if (button.isPressed()) {
				main_intake.move_velocity(-200);
				indexer.move_velocity(600);
				left_intake.move_velocity(200);
				right_intake.move_velocity(-200);
		}

		else {
			main_intake.move_velocity(0);
			indexer.move_velocity(0);
			left_intake.move_velocity(0);
			right_intake.move_velocity(0);
		}

		// if (right2.isPressed()) {
		// 	pros::lcd::set_text(7, "You are holding the button!");
		// 	main_intake.move_velocity(50);
		// 	secondary_intake.move_velocity(50);
		// } else {
		// 	main_intake.move_velocity(0);
		// 	secondary_intake.move_velocity(0);
		// }


		left_wheels.set_brake_mode(mode);
		right_wheels.set_brake_mode(mode);

		drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                            controller.getAnalog(ControllerAnalog::rightX));

    pros::delay(10);
  }
}
>>>>>>> master
