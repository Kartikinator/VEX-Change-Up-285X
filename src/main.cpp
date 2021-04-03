#include "main.h"

// MOTOR PORTS
int DRIVE_FRONT_LEFT = 11;
int DRIVE_FRONT_RIGHT = -2;
int DRIVE_BACK_RIGHT = -10;
int DRIVE_BACK_LEFT = 20;

int chassisDRIVE_FRONT_LEFT = 11;
int chassisDRIVE_FRONT_RIGHT = 2;
int chassisDRIVE_BACK_RIGHT = 10;
int chassisDRIVE_BACK_LEFT = 20;

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

/*
pros::Motor drive_b_l(DRIVE_BACK_LEFT);
pros::Motor drive_b_r(DRIVE_BACK_RIGHT);
pros::Motor drive_f_l(DRIVE_FRONT_LEFT);
pros::Motor drive_f_r(DRIVE_FRONT_RIGHT);
*/

okapi::MotorGroup driveF({2, 11}); //negative voltage
okapi::MotorGroup driveB({20, 10});	//positive voltage

pros::ADIAnalogIn limit_switch ('A');

pros::ADIAnalogIn bumper ('C');

ADIEncoder leftencoder ('F', 'E');
ADIEncoder rightencoder ('F', 'E');
ADIEncoder middleencoder ('E', 'F');

// Declaring Chassis ---
std::shared_ptr<OdomChassisController> odomchas =
		ChassisControllerBuilder()
				.withMotors(DRIVE_FRONT_LEFT, DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT, DRIVE_BACK_LEFT)
				//.withSensors(leftencoder, rightencoder, middleencoder)
				.withGains(
						{0.0035, 0, 0}, // Distance controller gains
						{0.006, 0, 0}, // Turn controller gains
						{0.002, 0, 0.00006}  // Angle controller gains (helps drive straight)
					)
				.withSensors(
					ADIEncoder{'G', 'H'},
					ADIEncoder{'C', 'D', true},
					ADIEncoder{'E', 'F'}
				)
				.withDimensions(AbstractMotor::gearset::green, {{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
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

auto xModel = std::dynamic_pointer_cast<XDriveModel>(odomchas->getModel());

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
void competition_initialize() {

}

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
double move_kP = 3.0;
double move_kI = 0.0;
double move_kD = 0.01;

double move_Error = 0;
double move_lastError = 0;
double move_intg = 0;
double move_deriv = 0;
double move_totalError =0;
double velocity = 0;

bool atTarget = false;
//use inches for coords
void strafeBot(double distn){

middleencoder.reset();
  while (1){
		/////////// STRAFE PID
		double enc_val = middleencoder.get();
		double dist_to_tick = (distn/(2.75*pi))*360;
    //Proportional
    move_Error = dist_to_tick-enc_val;

    //Derivative
    velocity = move_Error-move_lastError;

		if (fabs(enc_val - dist_to_tick) < 10 && fabs(velocity)< 20){
			break;
		}
    //Integral
    if(fabs(move_Error) < 300){
    move_totalError += move_Error;
  } else {
    move_totalError = 0;
	}

    double updated_voltage = move_kP*move_Error + move_kI*move_totalError + move_kD*velocity;
		// float input_voltage = (updated_dist/dist)*50;

		driveF.moveVoltage(-updated_voltage);
		driveB.moveVoltage(updated_voltage);

		move_lastError = move_Error;
		pros::delay(20);

}
driveF.moveVoltage(0);
driveB.moveVoltage(0);
}


void strafe(int angle, int dist){


}
void autonomous() {

	odomchas->setState({0_in,0_in,0_deg});
	//bottom center + initalize
	xModel->strafe(-50);

	left_intake.move_velocity(-100);
	right_intake.move_velocity(-100);
	pros::delay(500);
	left_intake.move_velocity(0);
	right_intake.move_velocity(0);

	/*
	profileController->generatePath(
		{{0_ft, 0_ft, 0_deg},
		{4_ft, 0_ft, 90_deg}},
		"Straight1");

	profileController-> generatePath(
		{{0_ft, 0_ft, 0_deg},
		{1.5_ft, 0_ft, 0_deg}},
		"Straight2");

	profileController-> generatePath(
			{{0_ft, 0_ft, 0_deg},
			{3_ft, 0_ft, 90_deg}},
			"Straight3");

	profileController->generatePath(
		{{0_ft, 0_ft, 0_deg},
		{1_ft, 0_ft, 0_deg}},
		"Straight4");
	*/
	//bottom right corner
	left_intake.move_velocity(200);
	right_intake.move_velocity(200);
	profileController->setTarget("Straight1");
	profileController->waitUntilSettled();
	profileController->removePath("Straight1");
	profileController->setTarget("Straight2");
	indexer.move_velocity(200);
	pros::delay(500);
	indexer.move_velocity(0);
	left_intake.move_velocity(0);
	right_intake.move_velocity(0);

	profileController->generatePath(
		{{0_ft, 0_ft, 0_deg},
		{3_ft, -1_ft, 0_deg}},
		"Straight4");

	//middle right
	profileController->setTarget("Straight2", true);
	profileController->removePath("Straight2");
	odomchas->turnAngle(-100_deg);
	left_intake.move_velocity(200);
	right_intake.move_velocity(200);
	profileController->setTarget("Straight3");
	profileController->waitUntilSettled();
	profileController->removePath("Straight3");
	profileController->setTarget("Straight4");
	indexer.move_velocity(200);
	pros::delay(500);
	indexer.move_velocity(0);
	left_intake.move_velocity(0);
	right_intake.move_velocity(0);

	//top right
	profileController->setTarget("Straight4", true);
	profileController->waitUntilSettled();
	profileController->removePath("Straight4");






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
 		main_intake.move_velocity(200);
 		indexer.move_velocity(-100);
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
