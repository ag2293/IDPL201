/*
IDP
David Paterson
Line Sensor Module Example Code V1
Move the line sensor across the black and white line, monitor on serial
*/
int rightlinesensorPin = 3; // Connect sensor to input pin 3
void setup() {
 Serial.begin(9600); // Init the serial port
 pinMode(rightlinesensorPin, INPUT); // declare Micro switch as input
}
void loop(){
 int valRight = digitalRead(rightlinesensorPin); // read right input value
 Serial.println(valRight);
 delay(100);
}



// Algorithm Logic
//define pins for line sensors
#define leftLineSensor A0
#define centreLineSensor A1
#define rightLineSensor A2

// setting float value, nvm
float leftLineValue = 0;
float centreLineValue = 0 ;
float rightLineValue = 0;

float leftLineValueTolerance = xxx; // needs testing calibration
float centreLineValueTolerance = xxx; // needs testing calibration
float rightLineValueTolerance = xxx; // needs testing calibration

void straight_line_sensor (int timeLength){
      // below a value, detects a line (if cannot detect, then above above value)
      // if the value detected is greater than 111, we know the left sensor does not detect the line
      // if the value detected is smaller than 222, then we know the left sensor detects the line   
      // if we raise the value from 111 to 222, then the left sensor is more sensitive in detecting the line (more chance to detect a line)
      // in general, if we raise the tolerance value, it is easier for the line sensor to reach a value below the tolerance, hence easier to trigger turn direction function

      // too right
      if (leftLineValue < leftLineValueTolerance && rightLineValue > rightLineValueTolerance) {
        turnLeft(timeLength); // turn left
        }
        
      // too left
      else if (rightLineValue < rightLineValueTolerance && leftLineValue > leftLineValueTolerance){
      turnRight(timeLength); // turn right
      }
      
      // just centre
      else if (leftLineValue > leftLineValueTolerance && rightLineValue > rightLineValueTolerance) {
        moveForwards(timeLength); // nothing happens
      }
      else {
        moveForwards(timeLength);
    }
  
}
