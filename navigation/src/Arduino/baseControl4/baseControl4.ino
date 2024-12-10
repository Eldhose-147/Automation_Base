#define ROSSERIAL_ARDUINO_TCP
#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
 
const char* ssid     = "NOWHERE";
const char* password = "123abcd1234";
// Set the rosserial socket server IP address
IPAddress server(192,168,200,145);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
std_msgs::Float64 str_msg;
ros::Publisher rpm_value4("rpm_value4", &str_msg);

int PWM;
//float dir;/


void rotate(const geometry_msgs::Vector3 &msg){
 
PWM = msg.x;

}



ros::Subscriber<geometry_msgs::Vector3> sub("keyboard_message4", &rotate);

//ENCODER DETAILS

// Keep track of the number of right wheel pulses
volatile long count = 0;
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 // One-second interval for measurements
int interval = 1000;
// number of pulses per revolution
#define N 552.3936
//Interrupt Pins
#define readA digitalRead(5)
#define readB digitalRead(4)
//INPUT
int pwm_value=0;

// True = Forward; False = Reverse
boolean Direction_right = true;





void setup() {
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
 
 nh.initNode();
 nh.subscribe(sub);
 nh.advertise(rpm_value4);

 Serial.println("node is initialised");
 
 
 
 pinMode(5,INPUT_PULLUP); //Encoder pin
 pinMode(4,INPUT); // Encoder Pin
 
 pinMode(13,OUTPUT);
 pinMode(15,OUTPUT);

 
 
attachInterrupt(digitalPinToInterrupt(5), right_wheel_pulse,CHANGE);
attachInterrupt(digitalPinToInterrupt(4),right_wheel_pulse_b,CHANGE);
 
 
 
  // put your setup code here, to run once:

}

void loop() {

   // Record the time
 
  currentMillis = millis();
 
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;

    float ang_velocity=(((60*count)*0.00182175)/60 )*2*3.14;
//   float ang_velocity=(count/7)*2*3.14/19.2;
   
    Serial.println("the angular velocity is");
    
//    Serial.println(ang_velocity);

    str_msg.data = ang_velocity;
// Publishing rpm to ros

    rpm_value4.publish( &str_msg );
//    Serial.println(count);
    
   count = 0;
  }
 analogWrite(15,abs(PWM)*1.5);
 digitalWrite(13,PWM<0);
 
 
 nh.spinOnce();
 delay(1);
 
 
 
  // put your main code here, to run repeatedly:

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
