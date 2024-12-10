#include<stdlib.h>

// Keep track of the number of right wheel pulses
volatile long count = 0;
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 // One-second interval for measurements
int interval = 1000;
// number of pulses per revolution
//#define N 552.3936/
//Interrupt Pins
#define readA digitalRead(0)
#define readB digitalRead(4)
//INPUT
int pwm_value=0;

// True = Forward; False = Reverse
boolean Direction_right = true;
void setup() {
  Serial.begin(115200);
  pinMode(0,INPUT_PULLUP);
  pinMode(4,INPUT);
  pinMode(14,OUTPUT);
  pinMode(12,OUTPUT);


     
 
 
  digitalWrite(14,HIGH);
  analogWrite(12,255);
 
  attachInterrupt(digitalPinToInterrupt(0), right_wheel_pulse,CHANGE);
  attachInterrupt(digitalPinToInterrupt(4),right_wheel_pulse_b,CHANGE);
 

}
 
void loop() {
 
  // put your main code here, to runrepeatedly:
  // Record the time
 
  currentMillis = millis();
 
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;

    float ang_velocity=(60*count)/N;
   
   
    Serial.println("the angular velocity is");
    Serial.println(ang_velocity);/
//    Serial.println(count);
    count = 0;
   
  }

}
// Increment the number of pulses by 1
   
  // Read the value for the encoder for the right wheel
  ICACHE_RAM_ATTR void right_wheel_pulse() {
  if(readB != readA) {
    count ++;
  } else {
    count --;
  }
}
 
ICACHE_RAM_ATTR void right_wheel_pulse_b() {
  if (readA == readB) {
    count ++;
  } else {
    count --;
  }
}