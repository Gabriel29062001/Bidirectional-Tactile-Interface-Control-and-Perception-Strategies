// PIN DEFINITIONS AND VARIABLES
//*****************************************************************************************************
/*
  Color code definition
  White M+
  Blue Encoder +
  Green Enc B
  Yellow Enc A
  Black Enc -
  Red M-
*/

int analogPin = A3;
float val = 0;
// ENCODER
//*****************************************************************************************************
#define ENCODER_USE_INTERRUPTS  // Enable interrupts in arduino mega
#include "Encoder.h"            // To read the encoders
Encoder myEnc_A(40, 41);          // Physical pins for the encoders, 2 and 3 for Arduino MEGA
Encoder myEnc_B(42, 43);         // 18 and 19 on Arduino MEGA
Encoder myEnc_C(44, 45);         // 20 and 21 on Arduino MEGA



struct referencemotorangle {
  float motor1;
  float motor2;
  float motor3;
} robot1;

// Function to compute the inverse kinematics given the desired position
referencemotorangle Inverse_kinematics(float delta, float psi, float r0);



// Encoder readings (volatile and copies)
float ctr2deg = 360.0 / 1500.;
float ctr2rad = 6.283185 / 1500.;
float deg2ctr = 1500. / (360.0 * 18.);
long encoder_count_A_prev = 0;
long encoder_count_B_prev = 0;
long encoder_count_C_prev = 0;

float encoder_count_A, encoder_count_B, encoder_count_C;

//*****************************************************************************************************

float errA, refA, P_A, I_A, D_A, PID_A, pwmA;
float errB, refB, P_B, I_B, D_B, PID_B, pwmB;
float errC, refC, P_C, I_C, D_C, PID_C, pwmC;
int dir_A, dir_B, dir_C, _dir_A, _dir_B, _dir_C;
float kP = 7.5;

// MOTOR
//*****************************************************************************************************
// Motor pin definition
int MA1 = 4; // Pin for the motor A - 4 for Arduino  - 4 for Teensy
int MA2 = 5; // Pin for the motor A - 5 for Arduino - 5 for Teensy
int MB1 = 3; // Pin for the motor B - 8 for Arduino - 3 for Teensy
int MB2 = 2; // Pin for the motor B - 9 for Arduino - 2 for Teensy
int MC1 = 33; // Pin for the motor C - 5 for Arduino
int MC2 = 12; // Pin for the motor C - 7 for Arduino

// Variables to control the timing
//*****************************************************************************************************
float start_time = 0;     // Time boot arduino
float t_now, t_prev_iter;          // Time in the current iteration


float dt_iter = 0.000;         // Iteration loop time (read)
//*****************************************************************************************************
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

float dataNumber = 0;             // new for this version
//*****************************************************************************************************



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
  // Serial comunication
  //*****************************************************************************************************
  Serial.begin(115200);

  // LED CONTROL
  pinMode(13, OUTPUT);  // LED on pin D13

  // declare pin of the motor to be an output
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(MC1, OUTPUT);
  pinMode(MC2, OUTPUT);
  pinMode(analogPin, INPUT);

  // Timming control of the program
  start_time = millis();
  t_now = millis() - start_time;
  t_prev_iter = t_now;
}


// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop()
{

  val = analogRead(analogPin);
  recvWithEndMarker();


  t_now = micros();
  dt_iter = (t_now - t_prev_iter);
  if (dt_iter >= 1000000) {
    t_prev_iter = t_now;

    // Encocer Readings
    encoder_count_A = myEnc_A.read() / 4.3333;
    encoder_count_B = myEnc_B.read() / 4.3333;
    encoder_count_C = myEnc_C.read() / 4.3333;
    
    Serial.print(encoder_count_A); Serial.print("\t");
    Serial.print(encoder_count_B); Serial.print("\t");
    Serial.print(encoder_count_C); Serial.print("\t");
    
    if (newData == true) {
        dataNumber = 0;             // new for this version
        dataNumber = atof(receivedChars);   // new for this version
//        Serial.print("This just in ... ");
//        Serial.println(dataNumber);     // new for this version
        newData = false;
    }

    //if (dataNumber>0) robot1=Inverse_kinematics(0., abs(dataNumber), 0.04);
    //else robot1=Inverse_kinematics(180., abs(dataNumber), 0.04);
    robot1=Inverse_kinematics(0., -0.0, 0.04); // first phi of cylinder: 0-360, second heading: 0-25 degrees, third height: 0.04-0.1
    
    refA = robot1.motor1;
    refB = robot1.motor2;
    refC = robot1.motor3;

    Serial.print(refA); Serial.print("\t");
    Serial.print(refB); Serial.print("\t");
    Serial.print(refC); Serial.print("\n");

    
    errA = refA - encoder_count_A;
    P_A = errA * kP;
    PID_A = P_A + I_A + D_A;

    
    errB = refB - encoder_count_B;
    P_B = errB * kP;
    PID_B = P_B + I_B + D_B;

    
    errC = refC - encoder_count_C;
    P_C = errC * kP;
    PID_C = P_C + I_C + D_C;

    pwmA = constrain(PID_A, -255, 255);
    pwmB = constrain(PID_B, -255, 255);
    pwmC = constrain(PID_C, -255, 255);

    if (pwmA > 0) dir_A = 1;
    else dir_A = 0;
    if (pwmB > 0) dir_B = 1;
    else dir_B = 0;
    if (pwmC > 0) dir_C = 1;
    else dir_C = 0;

    if (dir_A == _dir_A) {
      if (pwmA < 0)
      {
        analogWrite(MA1, 0);
        analogWrite(MA2, -map(pwmA, -255, 0, -255, -150));
      }
      else if (pwmA > 0)
      {
        analogWrite(MA2, 0);
        analogWrite(MA1, map(pwmA, 0, 255, 150, 255));
      }
      else {
        analogWrite(MA2, 0);
        analogWrite(MA1, 0);
      }
    }
    else {
      analogWrite(MA2, 0);
      analogWrite(MA1, 0);
    }

    if (dir_B == _dir_B) {
      if (pwmB < 0)
      {
        analogWrite(MB1, 0);
        analogWrite(MB2, -map(pwmB, -255, 0, -255, -150));
      }
      else if (pwmB > 0)
      {
        analogWrite(MB2, 0);
        analogWrite(MB1, map(pwmB, 0, 255, 150, 255));
      }
      else {
        analogWrite(MB2, 0);
        analogWrite(MB1, 0);
      }
    }
    else {
      analogWrite(MB2, 0);
      analogWrite(MB1, 0);
    }

    if (dir_C == _dir_C) {
      if (pwmC < 0)
      {
        analogWrite(MC1, 0);
        analogWrite(MC2, -map(pwmC, -255, 0, -255, -150));
      }
      else if (pwmC > 0)
      {
        analogWrite(MC2, 0);
        analogWrite(MC1, map(pwmC, 0, 255, 150, 255));
      }
      else {
        analogWrite(MC2, 0);
        analogWrite(MC1, 0);
      }
    }
    else {
      analogWrite(MC2, 0);
      analogWrite(MC1, 0);
    }

    _dir_A = dir_A;
    _dir_B = dir_B;
    _dir_C = dir_C;

    //  Serial.print(pwmA);Serial.print("\t");
    //  Serial.print(encoder_count_B);Serial.print("\t");
    
  }


}



/////////////////////////////////////////////////////////////////
////////////////////////////Functions////////////////////////////
/////////////////////////////////////////////////////////////////

referencemotorangle Inverse_kinematics(float delta, float psi, float r0) {
  struct referencemotorangle refangle;
  double eps = 1e-8;
  // moudle parameters
  double r = 0.04406;
  double l = 0.0594;
  // the center location of the base

  // the location of the waterbomb center bi
  double theta[3] = {0, 2 * M_PI / 3, 4 * M_PI / 3};
  // convert the deg to rad
  delta = delta / 180 * M_PI;
  psi = psi / 180 * M_PI;
  // transfer psi from top plnane to the mid plane
  psi = psi / 2;
  // calculate the distance to from the centre of the base to the top
  r0 = r0 / sin(M_PI / 2 - psi);
  // angle limits
  double lower_limits[3] = {0, 0, 0};
  double upper_limits[3] = {M_PI_2, M_PI_2, M_PI_2};
  float sols[3];
  double phi[2];
  phi[0] = 0;
  phi[1] = 0;
  // derive the second order polynomial equation for each leg
  for (int i = 0; i < 3; i++) {
    double a = (r - l) * (sin(psi) * cos(delta - theta[i])) - (r0 / 2);
    double b = 2 * l * cos(psi);
    double c = (r + l) * (sin(psi) * cos(delta - theta[i])) - (r0 / 2);
    double root = b * b - 4 * a * c;
    if (root >= 0) {
      phi[0] = (-b + sqrt(root)) / (2 * a);
      phi[1] = (-b + sqrt(root)) / (2 * a);
    } else {
      Serial.print("no real solution for current reference workpoint\n");
    }
    for (int j = 0; j < 2; j++) {
      phi[j] = 2 * atan2(phi[j], 1);
      if ((phi[j] > lower_limits[i] - eps) and
          (phi[j] < upper_limits[i] + eps)) {
        sols[i] = phi[j];
        sols[i] = sols[i] * 180 / M_PI;
        // Serial.print(sols[i]);
        // Serial.print("\n");
      }
    }
  }
  refangle.motor1 = sols[0];
  refangle.motor2 = sols[1];
  refangle.motor3 = sols[2];
  return refangle;
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}