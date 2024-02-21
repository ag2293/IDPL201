/*
 IDP
 David Paterson
 Push Button Module Example V1
 When you push the digital button the Led 2 will turn off otherwise the LED turns on.
*/
int inputPin = 3; // Connect sensor to input pin 3
int toggled = 0;
void setup() {
  Serial.begin(9600); // Init the serial port
 pinMode(inputPin, INPUT); // declare pushbutton as input
}
void loop(){
 int val = digitalRead(inputPin); // read input value
 if(val == 1 and toggled==0){
  toggled = 1;
  Serial.println("Pressed");
  }
 else if(val==0){
  toggled=0;

 }

}
