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
#include <ArduinoEigenDense.h>


int analogPin = A3;
float val = 0;
// ENCODER
//*****************************************************************************************************
#define ENCODER_USE_INTERRUPTS  // Enable interrupts in arduino mega
#include "Encoder.h"  
Encoder myEnc_A(40, 41);         // 18 and 19 on Arduino MEGA, teeny 44, 45 not working
Encoder myEnc_B(42, 43);         // 20 and 21 on Arduino MEGA
Encoder myEnc_C(44, 45);          // Physical pins for the encoders, 2 and 3 for Arduino MEGA



struct referencemotorangle {
  float motor1;
  float motor2;
  float motor3;
} robot1;

// Function to compute the inverse kinematics given the desired position
referencemotorangle Inverse_kinematics(float r0, float roll, float pitch);



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
//7,.5

// MOTOR
//*****************************************************************************************************
// Motor pin definition
int MA1 = 36; // Pin for the motor A - 4 for Arduino  - 4 for Teensy
int MA2 = 37; // Pin for the motor A - 5 for Arduino - 5 for Teensy
int MB1 = 38; // Pin for the motor B - 8 for Arduino - 3 for Teensy
int MB2 = 39; // Pin for the motor B - 9 for Arduino - 2 for Teensy
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

// Forward kinematics parameters
double r = 0.04406; // in meters
double l = 0.0594; // in meters

Eigen::Vector3d vector_angle;
Eigen::Vector3d vector_angle_desired;

Eigen::Vector3d  results;
Eigen::Vector3d  desired;

String data_teensy;
String arduino_msg;

float initial_pos[3];
float desired_pos[3];
float* arduino_value;
int initialisation =0;



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


  initial_pos[0] = 19.68; // Initial height
  initial_pos[1] = 19.68;  // Initial roll
  initial_pos[2] = 19.68;  // Initial pitch

  // Copy values from initial_pos to desired_pos
  for (int i = 0; i < 3; i++) {
    desired_pos[i] = initial_pos[i];
  }

}


// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop()
{ 
  
  t_now = micros();
  dt_iter = (t_now - t_prev_iter);
  if (dt_iter >= 1000) {
    
    t_prev_iter = t_now;

    // Encocer Readings
    encoder_count_A = myEnc_A.read() / 4.3333;
    encoder_count_B = myEnc_B.read() / 4.3333;
    encoder_count_C = myEnc_C.read() / 4.3333;


    vector_angle  << encoder_count_A, encoder_count_B, encoder_count_C;
    results = forwardkinematics(vector_angle, r, l);

    if (initialisation == 0 && encoder_count_A >= 5 && encoder_count_B >= 5 && encoder_count_C >= 5 ){
        desired_pos[0] = encoder_count_A;
        desired_pos[1] = encoder_count_B;
        desired_pos[2] = encoder_count_C;
        initialisation = 1;
      
    }

    // Arduino reading if there is any message send by ROS2

    if (Serial.available() > 0) {

      // If there is something change the desired_position to the position send by ROS2
      if (Serial.find("<") == true) {
        data_teensy = Serial.readStringUntil('>'); // Read until '>' character
        arduino_msg = "<" + data_teensy + ">";
        arduino_value = extractValues(arduino_msg);
        desired_pos[0] = arduino_value[0];
        desired_pos[1] = arduino_value[1];
        desired_pos[2] = arduino_value[2];

      }
    }


    vector_angle_desired << refA, refB, refC;
    desired = forwardkinematics(vector_angle_desired, r, l);

    // Assign the reference position to the desired position from ROS2

    refA = desired_pos[0];
    refB = desired_pos[1];
    refC = desired_pos[2];

    // PID Implementation for correction on position
    
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

    // Output to be read by ROS2 for communcation
    String output_data = "<" + String(encoder_count_A) + ";" + String(encoder_count_B) + ";" + String(encoder_count_C) + ";" + String(refA) + ";" + String(refB) + ";" + String(refC) + ">";


    Serial.println(output_data);
 

    
  }
 

}


/////////////////////////////////////////////////////////////////
////////////////////////////Functions////////////////////////////
/////////////////////////////////////////////////////////////////

referencemotorangle Inverse_kinematics(float r0, float roll, float pitch) {
  struct referencemotorangle refangle;
  double eps = 1e-8;
  // moudle parameters
  double r = 0.04406;
  double l = 0.0594;
  // the center location of the base
  
  // the location of the waterbomb center bi
  double theta[3] = {0, 2 * M_PI / 3, 4 * M_PI / 3};
  // convert the deg to rad
  roll = roll / 180 * M_PI;
  pitch = pitch / 180 * M_PI;
  // transfer psi from top plnane to the mid plane
  pitch = pitch / 2;
  // calculate the distance to from the centre of the base to the top
  r0 = r0 / sin(M_PI / 2 - pitch);
  // angle limits
  double lower_limits[3] = {0, 0, 0};
  double upper_limits[3] = {M_PI_2, M_PI_2, M_PI_2};
  float sols[3];
  double phi[2];
  phi[0] = 0;
  phi[1] = 0;
  // derive the second order polynomial equation for each leg
  for (int i = 0; i < 3; i++) {
    double a = (r - l) * (sin(pitch) * cos(roll - theta[i])) - (r0 / 2);
    double b = 2 * l * cos(pitch);
    double c = (r + l) * (sin(pitch) * cos(roll - theta[i])) - (r0 / 2);
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

Eigen::VectorXd forwardkinematics(Eigen::VectorXd theta, double b, double l) {
  Eigen::VectorXd result(3);
  // Transform theta from degree to rad
  theta(0) = theta(0)*M_PI/180;
  theta(1) = theta(1)*M_PI/180;
  theta(2) = theta(2)*M_PI/180;

  // configurations for legs
  double phi1 = M_PI / 6;
  double phi2 = 5 * M_PI / 6;
  double phi3 = 3 * M_PI / 2;
  // waterbomb center position
  Eigen::Vector3d b1, b2, b3;
  b1 << cos(phi1) * (b + l * cos(theta(0))),
      sin(phi1) * (b + l * cos(theta(0))), l * sin(theta(0));
  b2 << cos(phi2) * (b + l * cos(theta(1))),
      sin(phi2) * (b + l * cos(theta(1))), l * sin(theta(1));
  b3 << cos(phi3) * (b + l * cos(theta(2))),
      sin(phi3) * (b + l * cos(theta(2))), l * sin(theta(2));
  // calculate the virtual plane's norm vector
  Eigen::Vector3d N = (b1 - b2).cross(b1 - b3);
  double d = (b1 - Eigen::Vector3d::Zero()).dot(N) / N.norm();
  Eigen::Vector3d pos = 2 * d * N / N.norm();
  /*Calculate the rotation angle for the top plane's norm vector*/
  N = N / N.norm();
  double alpha = atan2(N(1), N(0)) + M_PI;                  // azimuthal angle
  double beta = 2 * asin(sqrt(N(0) * N(0) + N(1) * N(1)));  // polar angle

  // calculate the roll and pitch angle ( in xyz-euler)  from  polar angle and
  // azimuthal angle
  double roll, pitch;
  pitch = asin(cos(alpha) * sin(beta));
  roll = asin(sin(alpha) * sin(beta) / -cos(pitch));

  // Transform pitch and roll from rad to degree
  pitch = pitch*180/M_PI;
  roll = roll*180/M_PI;

  result << pos(2), roll, pitch;

  return result;
}


float* extractValues(String arduino_msg) {
    static float values[3]; // Array to store float values

    // Remove the angle brackets from the string
    arduino_msg = arduino_msg.substring(1, arduino_msg.length() - 1);

    // Find the indices of semicolons
    int firstIndex = arduino_msg.indexOf(';');
    int secondIndex = arduino_msg.indexOf(';', firstIndex + 1);

    // Extract and convert the substrings between semicolons to floats
    values[0] = arduino_msg.substring(0, firstIndex).toFloat();
    values[1] = arduino_msg.substring(firstIndex + 1, secondIndex).toFloat();
    values[2] = arduino_msg.substring(secondIndex + 1).toFloat();

    // Return the array of float values
    return values;
}
