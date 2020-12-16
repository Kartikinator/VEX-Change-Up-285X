#include "main.h"
#include "main.cpp"

// op control drive

auto drive =
ChassisControllerBuilder()
  .withMotors(1, 11, -12, 2)
  .withDimensions(AbstractMotor::gearset::green, {{4.125_in, 10.5_in}, imev5GreenTPR})
  .build();


// op control buttons

ControllerButton leftArrow(ControllerDigital::left);
ControllerButton rightArrow(ControllerDigital::right);
ControllerButton upArrow(ControllerDigital::up);
ControllerButton downArrow(ControllerDigital::down);

//motor config

pros::Motor main_intake(9);
pros::Motor sec_intake(10);


void move(std::shared_ptr<okapi::AsyncMotionProfileController> profile, okapi::QLength distance, bool dir){
  profile->generatePath({
    {0_ft,0_ft,0_deg},
    {distance, 0_ft,0_deg}},
    "Point1"
  );

  profile->setTarget("Point1", dir);
  profile->waitUntilSettled();
  profile->removePath("Point1");
}
