int button_pin = 11;
bool start = false;

int RED_LED = 6;
int GREEN_LED = 9;
int BLUE_LED = 7;

bool LED = false;

int loopCounter = 0;



//---------------------------------
void(* resetFunc) (void) = 0; //  claim the position --> ISSUE


void IRS(){
  Serial.println("RESET");//have to reopen serial comms to see.
  resetFunc();
}

void setup() {
  pinMode(button_pin, INPUT_PULLDOWN); // initialize front right sensor as as input (will still 0 for nothing 1 for on.)


  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  // Initialize Serial Communication at 9600 baud rate
  Serial.begin(9600);
  delay(1000); // Ensure serial communication is fully set up
  while (!start) {
    start=digitalRead(button_pin);
    Serial.println("Waiting to start");
  }
  Serial.println("Start");
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(button_pin), IRS, CHANGE);
}


void loop() {
  delay(25);
  // (The rest of your existing loop code goes here)
  

  // LED control logic
  if(loopCounter >= 10) { // Toggle LED every 10 iterations
    LED = !LED; // Toggle the LED state
    if (LED == true){
      digitalWrite(BLUE_LED, LOW);
    }
    else{
      digitalWrite(BLUE_LED, HIGH);
    }
    loopCounter = 0; // Reset counter
  } else {
    loopCounter++; // Increment counter
  }
  
}

