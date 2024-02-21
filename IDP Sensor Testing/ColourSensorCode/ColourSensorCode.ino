#include <Wire.h>

#define IR 4

void setup() {
  Serial.begin(9600);
  Serial.print("switch: ");
  pinMode(IR,INPUT);
}

void loop() {
  Serial.println("-------------------------RF-----------------------");
  if(digitalRead(IR) == 1)
    Serial.println("Colour: RED");
  else
    Serial.println("Colour: BLACK ");
  Serial.println();
}   
