#include "main.h"
#include "devices.cpp"
// MOTOR PORTS
int DRIVE_FRONT_LEFT = 11;
int DRIVE_FRONT_RIGHT = -2;
int DRIVE_BACK_RIGHT = -10;
int DRIVE_BACK_LEFT = 20;

int INDEXER = 9;
int MAIN_INTAKE = 5;
int LEFT_INTAKE = 16;
int RIGHT_INTAKE = 17;

const double Deadzone = 0.1;


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
pros::Motor main_intake (MAIN_INTAKE, true);
pros::Motor indexer (INDEXER);
pros::Motor left_intake (LEFT_INTAKE);
pros::Motor right_intake (RIGHT_INTAKE, true);

pros::Motor drive_b_l(DRIVE_BACK_LEFT);
pros::Motor drive_b_r(DRIVE_BACK_RIGHT);
pros::Motor drive_f_l(DRIVE_FRONT_LEFT);
pros::Motor drive_f_r(DRIVE_FRONT_RIGHT);


pros::ADIAnalogIn limit_switch ('A');

pros::ADIAnalogIn bumper ('C');

ADIEncoder leftencoder ('F', 'E');
ADIEncoder rightencoder ('F', 'E');
ADIEncoder middleencoder ('F', 'E');

// Declaring Chassis ---
std::shared_ptr<OdomChassisController> odomchas =
		ChassisControllerBuilder()
				.withMotors(DRIVE_FRONT_LEFT, DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT, DRIVE_BACK_LEFT)
				.withSensors(leftencoder, rightencoder, middleencoder)
				.withGains(
						{0.0035, 0, 0}, // Distance controller gains
						{0.006, 0, 0}, // Turn controller gains
						{0.002, 0, 0.00006}  // Angle controller gains (helps drive straight)
					)
				.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
				.withOdometry({{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
				.buildOdometry();


auto xModel = std::dynamic_pointer_cast<XDriveModel>(odomchas->getModel());

std::shared_ptr<AsyncMotionProfileController> profileController =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			2.0, // Maximum linear acceleration of the Chassis in m/s/s
			5.0 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(odomchas)
		.buildMotionProfileController();

Controller controller;

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


		xModel->strafe(50);
		pros::delay(500);
		xModel->strafe(0);

		pros::delay(100);

		xModel->rotate(50);
		pros::delay(200);
		xModel->rotate(0);

		xModel->forward(50);
		pros::delay(1000);
		xModel->forward(0);



		profileController->generatePath({
			{0_ft, 0_ft, 0_deg},
			{0_ft, 2_ft, 0_deg}},
			"straight1"
		);

		profileController->setTarget("straight1", true);
		profileController->waitUntilSettled();

		// indexer.move_velocity(200);
		// pros::delay(500);
		// indexer.move_velocity(0);
		//





}

/*
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

 	pros::motor_brake_mode_e_t mode = pros::E_MOTOR_BRAKE_HOLD;
 	ControllerDigital a {DIGITAL_A};
 	ControllerButton a_button(a);

 	ControllerDigital b {DIGITAL_B};
 	ControllerButton b_button(b);

 	ControllerDigital r1 {DIGITAL_R1};
 	ControllerButton right1(r1);

 	ControllerDigital r2 {DIGITAL_R2};
 	ControllerButton right2(r2);

 	ControllerDigital l1 {DIGITAL_L1};
 	ControllerButton left1(l1);

 	ControllerDigital l2 {DIGITAL_L2};
 	ControllerButton left2(l2);

 	bool detected = false;

   while (true) {
 	//	pros::lcd::set_text(2, "Encoder: " + std::to_string(encoder.get_value())); //  + " C");
 		pros::lcd::set_text(3, "Right Motor Temperature: N/A"); // + std::to_string(int(right_wheels.get_temperature())) + " C");
 		pros::lcd::set_text(4, "Battery Temperature: " + std::to_string(int(pros::battery::get_temperature())) + " C");
 		pros::lcd::set_text(5, "Battery Current: " + std::to_string(pros::battery::get_current()));
 //		pros::lcd::set_text(6, "Line Sensor: " + std::to_string(line_sensor.get_value()));

 		if (detected == false) {

 		if (right1.isPressed()) {
 			main_intake.move_velocity(0);
 			indexer.move_velocity(-200);
 			left_intake.move_velocity(0);
 			right_intake.move_velocity(0);
 		}
 		else if (a_button.isPressed()) {
 				main_intake.move_velocity(200);
 				indexer.move_velocity(200);
 		}
 		else if (b_button.isPressed()) {
 			left_intake.move_velocity(-200);
 			right_intake.move_velocity(-200);
 		}
 		else if (left1.isPressed()) {
 				main_intake.move_velocity(-200);
 				indexer.move_velocity(200);
 				left_intake.move_velocity(-200);
 				right_intake.move_velocity(-200);
 		}
 		else if (left2.isPressed()) {
 				main_intake.move_velocity(-200);
 				indexer.move_velocity(200);
 				left_intake.move_velocity(200);
 				right_intake.move_velocity(-200);
 		}
 		else if (right2.isPressed()) {

 		if (limit_switch.get_value() >11) {
 		main_intake.move_velocity(150);
 		indexer.move_velocity(-50);
 		left_intake.move_velocity(200);
 		right_intake.move_velocity(200);
 	} else {
 		main_intake.move_velocity(0);
 		indexer.move_velocity(0);
 		left_intake.move_velocity(150);
 		right_intake.move_velocity(150);
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
		xModel->xArcade(
		  			controller.getAnalog(ControllerAnalog::rightX), //side to side
		      	controller.getAnalog(ControllerAnalog::rightY), //front back
		      	controller.getAnalog(ControllerAnalog::leftX), //spin
						Deadzone
				  );


     pros::delay(10);
   }
}
