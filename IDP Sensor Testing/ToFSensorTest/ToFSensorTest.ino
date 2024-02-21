#include "Arduino.h"
#include "Wire.h"
  #include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X sensor;
void setup() {
 //initialize serial communication at 9600 bits per second:
 Serial.begin(115200);
 //join i2c bus (address optional for master)
 Wire.begin();
 //Set I2C sub-device address
 sensor.begin(0x50);
 //Set to Back-to-back mode and high precision mode
 sensor.setMode(sensor.eContinuous,sensor.eHigh);
 //Laser rangefinder begins to work
 sensor.start();
}



void loop() {
  // Get the distance in meters
  float distance = sensor.getDistance();

  // Convert distance to centimeters (1 meter = 100 centimeters)
  float distance_cm = distance * 100;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  // Check if the distance is smaller than 1 cm
  if (distance_cm < 1.0) {
    // Command to stop the robot
    robotStop();
    Serial.println("Robot stopped due to close obstacle.");
  }

  // The delay is added to demonstrate the effect, and it will not affect the measurement accuracy
  delay(500);
}
