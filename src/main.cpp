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


	std::shared_ptr<ChassisController> drive =
	    ChassisControllerBuilder()
	        .withMotors(1, 11, -12, 2)
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

  while (true) {
		pros::lcd::set_text(2, "Left Motor Temperature: " + std::to_string(int(left_wheels.get_temperature())) + " C");
		pros::lcd::set_text(3, "Right Motor Temperature: " + std::to_string(int(right_wheels.get_temperature())) + " C");
		pros::lcd::set_text(4, "Battery Temperature: " + std::to_string(int(pros::battery::get_temperature())) + " C");
		pros::lcd::set_text(5, "Battery Current: " + std::to_string(pros::battery::get_current()));
		pros::lcd::set_text(6, "Battery Voltage: " + std::to_string(pros::battery::get_voltage()));

		if (right1.isPressed()) {
			pros::lcd::set_text(7, "You are holding the button!");
			main_intake.move_velocity(200);
			indexer.move_velocity(600);
			left_intake.move_velocity(-200);
			right_intake.move_velocity(200);

		}
		else if (right2.isPressed()) {
				main_intake.move_velocity(-200);
				indexer.move_velocity(-600);
				left_intake.move_velocity(200);
				right_intake.move_velocity(-200);
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
