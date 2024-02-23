// -----MOTOR INITILIASATION-------
#include <Adafruit_MotorShield.h>

//initalise motorshield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// initalise motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);//M4 pin
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);//M3 pin


//initalise motor variables
int motorSpeed = 250;
int turnSteeringMotorSpeed = 150;//CALIBRATE
int timeLength = 200; // define turning time
bool isMoving = false; // Tracks whether the robot is moving

// ----------------------------------

// -----LED INITILIASATION-------

bool LED = false;

/*
//initalise Led
#define movingLed 1
#define BLUE_LED_PIN 11 // Replace x with the actual pin number for the blue LED
*/

// ----------------------------------

// -----Line Sensor INITILIASATION-------

int left_line_sensor_pin=2; // left line sensor 
int right_line_sensor_pin=3; // right line sensor
int front_line_sensor_pin =4; // sensor on line at front 
//int back_line_sensor_pin =5; // sensor on line at back 

// ----------------------------------

// -----Junction INITILIASATION-------

bool at_T_junction = false;// can only go left or right
bool at_right_T_junction = false;// can only go right or forward
bool at_left_T_junction = false;// can only go left or forward
bool resteer = true;// needs to resteer onto line --> either curved track or vehichle oriented wrong. BACK wheel drive so easier to self correct --> back doesn't move much even if front swings.
bool stop_moving = false;

// ----------------------------------



void setup() {
  // Start the motor shield
  AFMS.begin();
  // Set initial motor speed to 0
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

  // Initial direction
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  

  pinMode(left_line_sensor_pin, INPUT); // initialize Right sensor as an inut
  pinMode(right_line_sensor_pin, INPUT); // initialize Left sensor as as input
  pinMode(front_line_sensor_pin, INPUT); // initialize front sensor as an inut
  //pinMode(back_line_sensor_pin, INPUT); // initialize back sensor as as input
  attachInterrupt(digitalPinToInterrupt(front_line_sensor_pin), steer, FALLING);// if front sensor not on line anymore either: T-junction, or needs to resteer onto line

  // Initialize Serial Communication at 9600 baud rate
  Serial.begin(9600);

  delay(100); // Ensure serial communication is fully set up

  // Additional setup as needed
}


void loop() {

  int left_line_value = digitalRead(left_line_sensor_pin); // read left input value
  int right_line_value = digitalRead(right_line_sensor_pin); // read right input value
  int front_line_value = digitalRead(front_line_sensor_pin); // read front input value
  //int back_line_value = digitalRead(back_line_sensor_pin); // read back input value
  //Serial.println(back_line_value);
  delay(100);

  if (front_line_value == 1){// if front on line

    //
    /*if(left_line_value==0 && right_line_value==0 && back_line_value==1){ //FORWARD definetly oriented
      Serial.print("FORWARD FULL"); 
    }
    if(left_line_value==0 && right_line_value==0 && back_line_value==0){ //FORWARD slow as definetly veering off line
      Serial.print("FORWARD SLOW"); 
    }*/

    if(left_line_value==0 && right_line_value==0){ //FORWARD if left black right black and front white
      Serial.print("FORWARD");
      moveForwardTillJunction();
    }
    else if(left_line_value==1 && right_line_value==1){ //LEFT possible RIGHT possible junction
      at_T_junction = true;
      Serial.print("at END of T-JUNCTION");
      Serial.print("LEFT or RIGHT ... picking left");

      turnLeft();
      

      // trigger junction function

    }
    else if(left_line_value==1 && right_line_value==0){ //LEFT possible
      at_left_T_junction = true;
      Serial.print("LEFT or FORWARD"); 

      turnLeft();

      // trigger junction function

    }
    else if(left_line_value==0 && right_line_value==1){ //RIGHT possible
      at_right_T_junction = true;
      Serial.print("RIGHT or FORWARD"); 

      turnRight();

      // trigger junction function

    }
    
  }
  
  
}


void flashBlueLED() {
  if (isMoving) {
    unsigned long currentMillis = millis();
    static unsigned long previousMillis = 0;
    const long interval = 250; // LED on/off interval set to 250ms for 2Hz flash rate

    if (currentMillis - previousMillis >= interval) {
      // save the last time we blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      /*int ledState = digitalRead(BLUE_LED_PIN); 
      digitalWrite(BLUE_LED_PIN, !ledState); */
      LED != LED;
      Serial.print("LED");
      Serial.println(LED);

    }
  } 
  else {
    //digitalWrite(BLUE_LED_PIN, LOW); 
    Serial.println("LED OFF");
  }
  
}



// DECLARE FUNCTIONS TO MOVE --


void moveForwardTillJunction() {

  Serial.println("Moving forwards ...");
  int left_line_value = digitalRead(left_line_sensor_pin); // read left input value
  int right_line_value = digitalRead(right_line_sensor_pin); // read right input value

  isMoving = true; // Start moving
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);

  while (digitalRead(left_line_sensor_pin) == 0 && digitalRead(right_line_sensor_pin) == 0) {//while
      flashBlueLED();
  }
  isMoving = false;
  stopMoving();
  delay(1000);
}

void moveBackwardTillJunction() {

  Serial.println("Moving backwards ...");
  isMoving = true;
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  while (digitalRead(left_line_sensor_pin) == 0 && digitalRead(right_line_sensor_pin) == 0) {//while
      flashBlueLED();
  }
  isMoving = false;
  stopMoving();
  delay(1000);
}

void stopMoving() {
  Serial.println("Stopping motion ...");
  isMoving = false;
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  delay(1000);
}

void turnRight() {
  Serial.println("Turning right ...");
  detachInterrupt(digitalPinToInterrupt(front_line_sensor_pin));
  isMoving = true;
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  while (digitalRead(right_line_sensor_pin) == 0) {//while right not on line
    flashBlueLED();
  }
  while (digitalRead(front_line_sensor_pin) == 0) {
      flashBlueLED();
  }
  isMoving = false;
  stopMoving();
  attachInterrupt(digitalPinToInterrupt(front_line_sensor_pin), steer, FALLING);// Keep checking for veering off line 
  
  delay(1000);
  
}

void turnLeft() {
  Serial.println("Turning left ...");
  detachInterrupt(digitalPinToInterrupt(front_line_sensor_pin));//
  isMoving = true;

  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);

  while (digitalRead(left_line_sensor_pin) == 0) {//while left not on line
    flashBlueLED();
  }
  while (digitalRead(front_line_sensor_pin) == 0) {
      flashBlueLED();
  }
  isMoving = false;
  stopMoving();
  attachInterrupt(digitalPinToInterrupt(front_line_sensor_pin), steer, FALLING);// Keep checking for veering off line 
  delay(1000);
  
}

void turn180() {//AT GREEN ZONE
  Serial.println("Turning left 180 deg ...");
  isMoving = true;
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  while (digitalRead(left_line_sensor_pin) == 0) {//while left not on line
    flashBlueLED();
  }
  while (digitalRead(front_line_sensor_pin) == 0) {
      flashBlueLED();
  }
  while (digitalRead(left_line_sensor_pin) == 0) {//while left not on line
    flashBlueLED();
  }
  while (digitalRead(front_line_sensor_pin) == 0) {
      flashBlueLED();
  }
  isMoving = false;
  stopMoving();
  delay(1000);
}

// stationery turn right 
void turn180_right () {//AT RED zone
  Serial.println("Turning left 180 deg ...");

  isMoving = true;
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  while (digitalRead(right_line_sensor_pin) == 0) {//while right not on line
    flashBlueLED();
  }
  while