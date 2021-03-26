#include "main.h"
#include "main.cpp"

const double move_kP = 0;
const double move_kI = 0;
const double move_kD = 0;

const double turn_kP = 0;
const double turn_kI = 0;
const double turn_kD = 0;


double move_Error = 0;
double move_lastError = 0;
double move_intg = 0;
double move_deriv = 0;
double move_totalError =0;

double turn_Error = 0;
double turn_lastError = 0;
double turn_intg = 0;
double turn_deriv = 0;
double turn_totalError = 0;

bool resetencoders = false;
bool isPIDon = true;

//use inches for coords pls
void moveBot(int xcor, int ycor){
  //Vertical movement
  double mag = sqrt(xcor^2+ycor^2);
  double heading = atan2 (ycor,xcor);

  while (isPIDon){
    if(resetencoders){
      resetencoders = false;
      leftencoder.reset();
      rightencoder.reset();
    }
    /////////// DISTANCE PID
    double avg_enc_val = (leftencoder.get()*41.669+rightencoder.get()*41.669)/2;

    //Proportional
    move_Error = mag-avg_enc_val;

    //Derivative
    move_deriv = move_Error-move_lastError;

    //Integral
    if(move_Error != 0){
    move_totalError += move_Error;
  } else {
    move_totalError = 0;
  }

    move_lastError = move_Error;

    double mag_sum = move_kP*move_Error + move_kI*move_totalError + move_kD*move_deriv;


    //double updated_x_cor = mag_sum*sin(heading);
    //double updated_y_cor = mag_sum*cos(heading);

    drive_b_l.move_voltage((int)(sin(heading-pi/4)*mag_sum)+0.5);
    drive_b_r.move_voltage((int)(sin(heading+pi/4)*mag_sum)+0.5);
    drive_f_l.move_voltage((int)(sin(heading+pi/4)*mag_sum)+0.5);
    drive_f_r.move_voltage((int)(sin(heading-pi/4)*mag_sum)+0.5);

    pros::delay(20);
}

}

//Use degrees
void turnBot(int angle){

    double deltaL = leftencoder.get()*41.669;
    double deltaR = rightencoder.get()*41.669;
    double track_wheel_dist = 7.25+7.25; //MEASURE THESE ON BOT

    double current_heading = (deltaL+deltaR)/track_wheel_dist;
    double heading_deg = current_heading*180/pi;

    //turn PID

    //Proportion
    turn_Error = angle-heading_deg;
    //Derivative
    turn_deriv = turn_Error-turn_lastError;
    //Integral
    if(turn_Error != 0){
      turn_totalError += turn_Error;
    }else{
      move_totalError = 0;
    }

    turn_lastError = turn_Error;

    double angle_sum = turn_kP*turn_Error + turn_kI*turn_totalError + turn_kD*turn_deriv;

    
}
