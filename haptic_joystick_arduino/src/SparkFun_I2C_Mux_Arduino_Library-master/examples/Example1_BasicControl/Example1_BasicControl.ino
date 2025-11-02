/*
  Use the Qwiic Mux to access multiple I2C devices on seperate busses.
  By: Nathan Seidle @ SparkFun Electronics
  Date: May 17th, 2020
  License: This code is public domain but you buy me a beer if you use this
  and we meet someday (Beerware license).

  Some I2C devices respond to only one I2C address. This can be a problem
  when you want to hook multiple of a device to the I2C bus. An I2C Mux
  solves this issue by allowing you to change the 'channel' or port that
  the master is talking to.

  This example shows how to connect to different ports.
  The TCA9548A is a mux. This means when you enableMuxPort(2) then the SDA and SCL lines of the master (Arduino)
  are connected to port 2. Whatever I2C traffic you do, such as distanceSensor.startRanging() will be communicated to whatever
  sensor you have on port 2.

  Hardware Connections:
  Attach the Qwiic Mux Shield to your RedBoard or Uno.
  Plug a device into port 0 or 1
  Serial.print it out at 115200 baud to serial monitor.

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14685
*/

#include <Wire.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802

QWIICMUX myMux;
NAU7802 load_sensor_1;
NAU7802 load_sensor_2;
NAU7802 load_sensor_3;

String output;

void setup() {

    Serial.begin(115200);
    Serial.println();
    Serial.println("Qwiic Mux Shield Read Example");

    Wire.begin();

    if (myMux.begin() == false) {
        Serial.println("Mux not detected. Freezing...");
        while (1);
    }
    Serial.println("Mux detected");

    myMux.setPort(1); // Connect master to port labeled '2' on the mux for the Qwiic Scale

    // Initialize communication with the Qwiic Scale
    if (!load_sensor_1.begin()) {
        Serial.println("Sensor 1 not detected. Freezing...");
        while (1);
    }
    Serial.println("Sensor 1 detected");


    myMux.setPort(2); // Connect master to port labeled '2' on the mux for the Qwiic Scale

    // Initialize communication with the Qwiic Scale
    if (!load_sensor_2.begin()) {
        Serial.println("Sensor 2 not detected. Freezing...");
        while (1);
    }
    Serial.println("Sensor 2 detected");

    myMux.setPort(3); // Connect master to port labeled '2' on the mux for the Qwiic Scale

    // Initialize communication with the Qwiic Scale
    if (!load_sensor_3.begin()) {
        Serial.println("Sensor 3 not detected. Freezing...");
        while (1);
    }
    Serial.println("Sensor 3 detected");
    


    Serial.println("Begin scanning for I2C devices");
}

void loop(){


    myMux.setPort(1); // Connect master to port labeled '2' on the mux for the Qwiic Scale
    int32_t currentReading_1 = load_sensor_1.getReading();
    myMux.setPort(2); // Connect master to port labeled '2' on the mux for the Qwiic Scale
    int32_t currentReading_2 = load_sensor_1.getReading();
    myMux.setPort(3); // Connect master to port labeled '2' on the mux for the Qwiic Scale
    int32_t currentReading_3 = load_sensor_1.getReading();

    output = "<" + String(currentReading_1) + ";" + String(currentReading_2) + ";" + String(currentReading_3) + ">";
    Serial.println(output);
    delay(500);

  
}
