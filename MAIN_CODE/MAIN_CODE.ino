// -----MOTOR INITILIASATION-------
#include <Adafruit_MotorShield.h>
//initalise motorshield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// initalise motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);//M3 pin
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);//M4 pin
//initalise motor variables
int motorSpeed = 200;//200
int reverseMotorSpeed = 200; //(may need seperate speed callib for reversing)
int steeringMotorSpeed = 180;//180
int turningMotorSpeed = 150;//180
int slowMotorSpeed = 150;//CALIBRATE
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
int right_line_sensor_pin=13; // right line sensor
int front_left_line_sensor_pin =4; // sensor on line at front
int front_right_line_sensor_pin =11; // sensor on line at back
// ----------------------------------
// -----Junction INITILIASATION-------
bool at_T_junction = false;// can only go left or right
bool at_right_T_junction = false;// can only go right or forward
bool at_left_T_junction = false;// can only go left or forward

static unsigned long totalMillis = 50;
unsigned long startMillis = 0;
bool junction_detected = false;

bool turn_left = false;
bool turn_right = false;
bool started_turn_from_left_on_line = false;
bool started_turn_from_right_on_line = false;
bool check_end_turn = false;

// ----------------------------------


// ----------------------- NAVIGATION --------------------------------

// Define routes 
String pick_up_1[3] = {"Left", "Right", "Block_Detection"};// START to pickup 1
String drop_off_1_green[5] = {"Reverse", "Left", "Left", "Drop_Block", "Turn_180_Left"};// pick up 1 to green
String drop_off_1_red[6] = {"Reverse", "Right", "Forward_FR", "Right", "Drop_Block", "Turn_180_Right"};//pick up 1 to red
String pick_up_2_green[6] = {"Turn_180_Left", "Forward_RF", "Right", "Forward_FL", "Right", "Block_Detection"};// green to 2
String pick_up_2_red[4] = {"Forward_FL", "Left", "Left", "Block_Detection"};//red to 2
String drop_off_2_green[7] = {"Reverse", "Right", "Forward_FR", "Left", "Forward_FL", "Drop_Block", "Turn_180_Left"};// 2 to green
String drop_off_2_red[6] = {"Reverse", "Left", "Right", "Forward_FR", "Drop_Block", "Turn_180_Right"};// 2 to red
String pick_up_3_green[5] = {"Forward_FR", "Right", "Left", "Left", "Block_Detection"};// green to 3
String pick_up_3_red[6] = {"Forward_FL", "Left", "Forward_FL", "Right", "Left", "Block_Detection"};// red to 3
String drop_off_3_green[7] = {"Reverse", "Left", "Right", "Left", "Forward_FL", "Drop_Block", "Turn_180_Left"};// 3 to green
String drop_off_3_red[8] = {"Reverse", "Left", "Left", "Forward_FR", "Right", "Forward_FR", "Drop_Block", "Turn_180_Right"};// 3 to red
String pick_up_4_green[12] = {"Forward_FR", "Forward_FR", "Forward_FR", "Right", "Block_Detection", "Forward_FR", "Right", "Left", "Forward_FL", "Right", "Right", "Block_Detection"};// green to 4
String pick_up_4_red[12] = {"Forward_FL", "Forward_FL", "Left", "Block_Detection", "Forward_FL", "Left", "Forward_FL", "Right", "Forward_FL", "Right", "Right", "Block_Detection"};// red to 4
String drop_off_4_green[7] = {"Reverse", "Right", "Forward_FL", "Forward_FL", "Forward_FL", "Drop_Block", "Turn_180_Left"};// 4 to green
String drop_off_4_red[6] = {"Reverse", "Left", "Forward_FR", "Forward_FR", "Drop_Block", "Turn_180_Right"};// 4 to red
String return_to_start_green[4] = {"Right", "Forward_FL", "Left", "Reverse"};// green to Start
String return_to_start_red[3] = {"Left", "Right", "Reverse"};// red to Start

// Array of pointers to each route array for easier iteration
String *routes[17] = {pick_up_1, drop_off_1_green, drop_off_1_red, pick_up_2_green, pick_up_2_red, drop_off_2_green, drop_off_2_red, pick_up_3_green, pick_up_3_red, drop_off_3_green, drop_off_3_red, pick_up_4_green, pick_up_4_red, drop_off_4_green, drop_off_4_red, return_to_start_green, return_to_start_red};
int routeSizes[17] = {3, 5, 6, 6, 4, 7, 6, 5, 6, 7, 8, 12, 12, 7, 6, 4, 3};

//pick_up_1, drop_off_1, pick_up_2, drop_off_2, pick_up_3, drop_off_3, pick_up_4, drop_off_4, return_to_start
bool tasks_completed[9] = {false, false, false, false, false, false, false, false, false};
const bool all_tasks_completed[9] = {true, true, true, true, true, true, true, true, true};


bool left = false;
bool right = false;
bool reverse = false;
bool left_180 = false;//180 turn left
bool right_180 = false;//180 turn right
bool detect_wall = false;
bool detect_block = false;

bool set_next_action = true;// set to true after turn, block pick up, block drop off, when reverse is set to false from true (i.e., in the turn code)
bool insideRoute = false;//define if within a route
int currentRoute = 0; // Tracks the current route
int indexInsideRoute = 0; //tracks within route

// ---------------------------------------------------------------------


// --------------------------------- Grabber -----------------------------------

#include <Wire.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"
#include <Servo.h>;

// ---------------------------------------------------------------

// ----------- Initialise TOF SENSOR VARS --------------------

DFRobot_VL53L0X sensor;//tof sensor
//MINIMUM around 2 cm!!!

//// For block pick up
//TO CALLIBRATE
float grabber_distance = 2.5;//distance for grabber mechanism to pick up MIN 2.0
float detection_offset_block = 3.0;
////

//// For block drop off
//TO CALLIBRATE
float wall_distance = 31.0;//where grabber will tigger release mechanism
float detection_offset_wall = 8.0;

// --------------------------------------------------------------

// ----------- Initialise COLOUR SENSOR VARS --------------------

#define IR 13 //colour sensor

//COLOUR SENSOR DISTANCE CALLIBRATED
float detection_distance = 8.0;//min distance colour sensor callibrated to detect black vs red at (can set with screwdriver)
bool colour_read = false;//only if within distance should colour value be of significance
bool red = true;//colour red or (if not) black

// --------------------------------------------------------------


// ----------- Initialise SERVOS VARS --------------------

Servo beak_servo;
Servo head_servo;


int open_pos_beak_servo = 30; 
int closed_pos_beak_servo = 5; 
int up_pos_head_servo = 90; 
int down_pos_head_servo = 150; 
int down_pos_lifted_head_servo = 140;

int beak_pos = 0;
int head_pos = 0;

// --------------------------------------------------------------


// ----------- Initialise TEST VARS --------------------

bool debug_grabber = false;//to initialise grabbers --> delay introduced before main loop (i.e 100s)


// --------------------------------------------------------------



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
  pinMode(front_left_line_sensor_pin, INPUT); // initialize front sensor as an inut
  pinMode(front_right_line_sensor_pin, INPUT); // initialize back sensor as as input
  // Initialize Serial Communication at 9600 baud rate
  Serial.begin(9600);
  Serial.println("Start");
  delay(100); // Ensure serial communication is fully set up
  // Additional setup as needed

  pinMode(IR,INPUT);//colour sensor

  //TOF sensor

  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous,sensor.eHigh);
  //Laser rangefinder begins to work
  sensor.start();


  beak_servo.write(closed_pos_beak_servo);//start closed
  beak_servo.attach(8);// callibrate pin.
  beak_pos = closed_pos_beak_servo;

  
  head_servo.write(up_pos_head_servo);//start up ensure in up already else will jerk at begining
  head_servo.attach(4);// callibrate pin
  head_pos = up_pos_head_servo;

  delay(1000);
  Serial.begin(9600);
  Serial.println("test");

  upHead();

  if (debug_grabber == true){
    openBeak();
    downHead();
    closeBeak();
    downLiftedHead();
    upHead();
    delay(100000);
  }
  Serial.println("finished set up");
}
void loop() {
  int left_line_value = digitalRead(left_line_sensor_pin); // read left input value
  int right_line_value = digitalRead(right_line_sensor_pin); // read right input value
  int front_left_line_value = digitalRead(front_left_line_sensor_pin); // read front input value
  int front_right_line_value = digitalRead(front_right_line_sensor_pin); // read front input value
  delay(25);
  
  //// NAVIGATION CODE GOES HERE
  if (insideRoute == false){
    
    // Execute routes sequentially, checking completion
    if (tasks_completed == all_tasks_completed) {
      //STOP MOVING AND DELAY
      Serial.println("All routes completed.");
      // trigger reverseFORSETTIME function
    } 
    else {
      //try this code
      int index = currentRoute/2;
      tasks_completed[index] = true;

      //else this code
      /*
      if (currentRoute == 1){
        tasks_completed[0] = true;
      }
      else if (currentRoute == 2 || currentRoute == 3){
        tasks_completed[1] = true;
      }
      else if (currentRoute == 4 || currentRoute == 5){
        tasks_completed[2] = true;
      }
      else if (currentRoute == 6 || currentRoute == 7){
        tasks_completed[3] = true;
      }
      else if (currentRoute == 8 || currentRoute == 9){
        tasks_completed[4] = true;
      }
      else if (currentRoute == 10 || currentRoute == 11){
        tasks_completed[5] = true;
      }
      else if (currentRoute == 12 || currentRoute == 13){
        tasks_completed[6] = true;
      }
      else if (currentRoute == 14 || currentRoute == 15){
        tasks_completed[7] = true;
      }
      else if (currentRoute == 16 || currentRoute == 17){
        tasks_completed[8] = true;
      }
      */
      currentRoute ++;
    }
  }
  if (insideRoute == true && set_next_action == true){
    set_next_action = false;
    
    
    String action = routes[currentRoute][indexInsideRoute];

    if (action == "Left"){
      left = true;
    }
    else if (action == "Right"){
      right = true;
    }
    else if (action == "Forward_FL"){
      left = false;//should just skip junction
    }
    else if (action == "Forward_FR"){
      right = false;//should just skip junction
    }
    else if (action == "Reverse"){
      reverse = true;
    }
    else if (action == "Turn_180_Left"){
      left_180 = true;
    }
    else if (action == "Turn_180_Right"){
      right_180 = true;
    }
    else if (action == "Drop_Block"){
      detect_wall = true;
    }
    else if (action == "Block_Detection"){
      detect_block = true;
    }




    if (indexInsideRoute + 1 == routeSizes[currentRoute]){//on last task of route
      insideRoute = false;
      Serial.println("Finished route");
    }
    indexInsideRoute ++;
    
  }
  ////

  //// actual turning (180 turning set if needed and no forward or backward motion as done on line not on junction)
  if (turn_left == true || left_180 == true){
    //180 degree turn or reverse till junction need on spot.
    if (left_180 == true || reverse == true){
      turnLeftOnSpot();
    }
    else{
      turnLeft();
    }
    
    if (started_turn_from_left_on_line == false && started_turn_from_right_on_line == false && check_end_turn == false){
      if (digitalRead(right_line_sensor_pin) == 1){// also should work for end t-junction
        started_turn_from_right_on_line = true;
        check_end_turn = true;
      }
      else if (digitalRead(left_line_sensor_pin) == 1){
        started_turn_from_left_on_line = true;
      }
      
    }

    //// left turn left on line (needs right to cross line)
    if (started_turn_from_left_on_line == true && check_end_turn == false){
      if (digitalRead(right_line_sensor_pin) == 1){
        check_end_turn = true;
      }
    }
    ////

    //// when both right sensors off line, continue line following
    if (check_end_turn == true){
      if (digitalRead(right_line_sensor_pin) == 0 && digitalRead(front_right_line_sensor_pin) == 1){
        turn_left = false;
        left_180 = false;
        started_turn_from_left_on_line = false;
        started_turn_from_right_on_line = false;
        check_end_turn = false;
        set_next_action = true;
        reverse = false;
        moveForward();//always move forward after turn by default
      }
    }
    ////
  }
  if (turn_right == true || right_180 == true){
    //180 degree turn or reverse till junction need on spot.
    if (right_180 == true  || reverse == true){
      turnRightOnSpot();
    }
    else{
      turnRight();
    }

    if (started_turn_from_left_on_line == false && started_turn_from_right_on_line == false && check_end_turn == false){
      if (digitalRead(left_line_sensor_pin) == 1){// also should encompass t-junction
        started_turn_from_left_on_line = true;
        check_end_turn = true;
      }
      else if (digitalRead(right_line_sensor_pin) == 1){
        started_turn_from_right_on_line = true;
      }
      
    }

    //// right turn right on line (needs left to cross line)
    if (started_turn_from_right_on_line == true && check_end_turn == false){
      if (digitalRead(left_line_sensor_pin) == 1){
        check_end_turn = true;
      }
    }
    ////

    //// when both left sensors off line, continue line following
    if (check_end_turn == true){
      if (digitalRead(left_line_sensor_pin) == 0 && digitalRead(front_left_line_sensor_pin) == 1){
        turn_right = false;
        right_180 = false;
        started_turn_from_left_on_line = false;
        started_turn_from_right_on_line = false;
        check_end_turn = false;
        set_next_action = true;
        reverse = false;
        moveForward();//always move forward after turn by default
      }
    }
    ////
  }


  //// grabber
  if (detect_wall == true || detect_block == true){
    grabber();
  }

  ////


  
  //e.g., for left detected if passing junction, left veer will happen but then right will cancel, till it leaves junction (hopefully enough for it to not veer off)
  if (front_left_line_value == 1 && front_right_line_value == 0 && turn_left == false && turn_right == false){
    if (reverse == true){
      rightABit();// correct in opposite way 
    }
    else{
      leftABit();
    }
  }
  if (front_left_line_value == 0 && front_right_line_value == 1 && turn_left == false && turn_right == false){
    if (reverse == true){
      leftABit();//correct in opposite way
    }
    else{
      rightABit();
    }
  }



  

  
  //// sets turning direction
  if (junction_detected == true){//
    turn_left = left;
    turn_right = right;
    left = false;
    right = false;
    junction_detected = false;
  }
  ////


  if(front_left_line_value==0 && front_right_line_value==0 && turn_left == false && turn_right == false){ //FORWARD if correct at start
    
    //Serial.println("FORWARD");
    if (reverse == true){
      moveBackward();
    }
    else{
      moveForward();
    }
    
  }
  
  //// junctions dependent on route --> i.e., left and right predetermined, junction detected is the purpose of this code
  if (turn_left == false && turn_right == false && junction_detected == false){
    if ((left_line_value==1 && right_line_value==1) || (left_line_value==1 && right_line_value==0) || (left_line_value==0 && right_line_value==1)){
      junction_detected = true;


      if (reverse == true){
        set_next_action = true;
      }


    }
    else {
      junction_detected = false;
    }
  }
  ////
}




void grabber() {
  // Get the distance in meters
  float distance = sensor.getDistance()/10;// cm
  

  int colour_detected = digitalRead(IR);
  delay(400);

  

  // NORMAL FORWARD MOTION


  if (detect_block == true){

    distance = distance - detection_offset_block;//offset in sensor (in built dont change)
    if (distance <= detection_distance){
      colour_read = true;
      if(colour_detected == 1){
        red = true;
      }
      else if (colour_detected == 0){
        red = false;
      }
      if(red == true && colour_read == true){
        Serial.println("RED");
      }
      else if (red == false && colour_read == true){
        Serial.println("BLACK");
      }
      delay(300);
    }
    
    

    // Check if the distance is smaller than grabber distance
    if (distance < grabber_distance) {
      Serial.println("In range to pick up.");

      stopMoving();

      ////TEST
      openBeak();
      downHead();
      closeBeak();
      downLiftedHead();
      ////TEST

      delay(1000);
      detect_block = false;
      set_next_action = true;
    }
  }

  else if (detect_wall){
    distance = distance - detection_offset_wall;//offset in sensor (based on if sensor 100% vertical)
    // Check if the distance is smaller than dropping off distance
    if (distance < wall_distance) {
      Serial.println("In range to drop off.");

      stopMoving();

      ////TEST
      downHead();


      moveForward();
      delay(1000);
      stopMoving();

      openBeak();
      upHead();
      ////TEST
      delay(1000);

      detect_wall = false;
      set_next_action = true;
    }
  }

  Serial.println("distance");
  Serial.println(distance);
  

  //
}

void openBeak() { 
  delay(2000);
  int pos;
  
  for (pos = beak_pos; pos <= open_pos_beak_servo; pos += 1) { // goes from 13 degrees to 0 degrees 
    beak_servo.write(pos); // tell servo to go to position
    delay(15); // waits 15 ms for the servo to reach the position 
  }
  beak_pos = open_pos_beak_servo;
}


void closeBeak() { 
  delay(2000);
  int pos;

  for (pos = beak_pos; pos >= closed_pos_beak_servo; pos -= 1) { // goes from 13 degrees to 0 degrees 
    beak_servo.write(pos); // tell servo to go to position
    delay(50); // waits 15 ms for the servo to reach the position 
  }
  beak_pos = closed_pos_beak_servo;
}

void downHead() { 
  delay(2000);
  int pos;

  for (pos = head_pos; pos <= down_pos_head_servo; pos += 1) { // goes from 13 degrees to 0 degrees 
    head_servo.write(pos); // tell servo to go to position
    delay(50); // waits 15 ms for the servo to reach the position 
  }
  head_pos = down_pos_head_servo;
}

void downLiftedHead() { 
  delay(2000);
  int pos;

  if (head_pos == down_pos_head_servo) {
    for (pos = head_pos; pos >= down_pos_lifted_head_servo; pos -= 1) { // goes from 13 degrees to 0 degrees 
      head_servo.write(pos); // tell servo to go to position
      delay(50); // waits 15 ms for the servo to reach the position 
    }
  }
  else if (head_pos == up_pos_head_servo) {
    for (pos = head_pos; pos <= down_pos_lifted_head_servo; pos += 1) { // goes from 13 degrees to 0 degrees 
      head_servo.write(pos); // tell servo to go to position
      delay(50); // waits 15 ms for the servo to reach the position 
    }
  }
  head_pos = down_pos_lifted_head_servo;
}

void upHead() { 
  delay(2000);
  int pos;
  
  for (pos = head_pos; pos >= up_pos_head_servo; pos -= 1) { // goes from 13 degrees to 0 degrees 
    head_servo.write(pos); // tell servo to go to position
    delay(50); // waits 15 ms for the servo to reach the position 
  }
  head_pos = up_pos_head_servo;
}



// DECLARE FUNCTIONS TO MOVE --
void moveForward() {
  //Serial.println("Moving forwards ...");
  isMoving = true; // Start moving
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  //leftMotor->run(FORWARD);
  //rightMotor->run(FORWARD);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void moveForwardSlowly() {
  Serial.println("Moving forwards slowly ...");
  isMoving = true; // Start moving
  leftMotor->setSpeed(slowMotorSpeed);
  rightMotor->setSpeed(slowMotorSpeed);
  //leftMotor->run(FORWARD);
  //rightMotor->run(FORWARD);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void stopMoving() {
  Serial.println("Stopping motion ...");
  isMoving = false;
  left = false;
  right = false;
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  delay(1000);
}

void moveBackward() {
  //Serial.println("Moving forwards ...");
  isMoving = true; // Start moving
  leftMotor->setSpeed(reverseMotorSpeed);
  rightMotor->setSpeed(reverseMotorSpeed);
  //leftMotor->run(FORWARD);
  //rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void turnRight() {
  Serial.println("Turning right ...");
  isMoving = true;
  leftMotor->setSpeed(turningMotorSpeed);
  rightMotor->setSpeed(turningMotorSpeed/2);
  // leftMotor->run(FORWARD);
  // rightMotor->run(BACKWARD);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
}
void turnLeft() {
  //Serial.println("Turning left ...");
  isMoving = true;
  leftMotor->setSpeed(turningMotorSpeed/2);
  rightMotor->setSpeed(turningMotorSpeed);
  // leftMotor->run(BACKWARD);
  // rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}
void turnRightOnSpot() {//useful after reversing or for 180 -- test
  Serial.println("Turning right ON SPOT ...");
  isMoving = true;
  leftMotor->setSpeed(turningMotorSpeed);
  rightMotor->setSpeed(turningMotorSpeed);
  // leftMotor->run(FORWARD);
  // rightMotor->run(BACKWARD);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
}
void turnLeftOnSpot() {
  Serial.println("Turning left ON SPOT ...");
  isMoving = true;
  leftMotor->setSpeed(turningMotorSpeed);
  rightMotor->setSpeed(turningMotorSpeed);
  // leftMotor->run(BACKWARD);
  // rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}
void leftABit(){
  //Serial.println("left a bit");
  isMoving = true;
  leftMotor->setSpeed(steeringMotorSpeed/5);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(steeringMotorSpeed);
  rightMotor->run(BACKWARD);
}
void rightABit(){
  //Serial.println("right a bit");
  isMoving = true;
  rightMotor->setSpeed(steeringMotorSpeed/5);
  rightMotor->run(BACKWARD);
  leftMotor->setSpeed(steeringMotorSpeed+30);// might be good removing + 30
  leftMotor->run(BACKWARD);
}



