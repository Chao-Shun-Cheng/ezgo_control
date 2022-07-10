/* 
 *  Testing Code for E-Z-GO Glof Car 
 *  This Package is able to control throttle, brake and light.
 *  Besides, it has two control mode, manual mode, autonumous mode.
 */ 
#include "DAC8562.h"

/*for feedback data in ros*/
#include <ros.h>
#include <string.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <string.h>

/*
 * modePin    : This is mode change pin. (digital input)
 * SSPin      : This pin controls DAC8562 
 * accPin     : This is accelerate pin.  (PWM output)
 * directPin  : This is direction pin    (digital output)
 * Clock      : Pin 13
 * Din        : Pin 11
 * diameter   : 18 inch tire 
 * gear_ratio : gear ratio is 16.99
 * REF_POWER  : This power is for referencing by DAC8562
 * lightPin   : This pin controls front and tail lights.
 */
#define modePin 7 
#define SSPin 10    
#define interruptPin 2  
#define directPin 4
#define diameter 0.4572
#define pi 3.14159
#define gear_ratio 16.99
#define REF_POWER 3.34
#define lightPin 8

/*
 * mode --> 0 : manual mode, 1 : autonomous mode
 * data --> {acc, dec, direction, light}
 *      acc : 0 ~ 5 V 
 *      dec : 0 ~ 5 V
 *      direction : 0 : forward, 1 : reverse
 *      light : 0 : off, 1 : on
 * distance --> distance during moter running a cycle
 */
int mode = 0;         
volatile float data[4] = {0, 0, 0, 0}; 
volatile float vel = 0; 
float distance = diameter * pi / gear_ratio * 1000 * 3.6; 
double previousTime = 0;
DAC8562 dac = DAC8562(SSPin, REF_POWER);

/*for feedback data in ros*/
String result = "0000";
ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("feedback", &str_msg);
/*result --> parameters to return to ros*/


void setup() {
    Serial.begin(57600);   
    dac.begin(); 
    attachInterrupt(digitalPinToInterrupt(interruptPin), calVel, RISING);
    pinMode(directPin, OUTPUT);  
    pinMode(modePin, INPUT);  
    pinMode(interruptPin, INPUT_PULLUP);
    pinMode(lightPin, OUTPUT);

    nh.initNode();        
    nh.advertise(chatter);
}

void loop() {
    int mode = digitalRead(modePin);
    if (mode == HIGH) { 
        getData();
        showInfo();
    } else {
      Serial.println("manual mode");

      result ="manual mode";
      char x[60];
      result.toCharArray(x,60);
      str_msg.data= x;
      chatter.publish(&str_msg);
      nh.spinOnce();
      delay(50);
      
      return;
    }
    delay(100);
}

void getData() {
    int i = 0;
    char chr;
    if (Serial.available() > 0) {
        data[0] = data[1] = data[2] = data[3] = 0;
        
        while ((chr = Serial.read()) != '\n' && i < 4) {
            if (chr == ',') i++;
            if (chr >= '0' && chr <= '9') data[i] = data[i] * 10 + (chr - '0');
        }
        if (data[2] == 1) digitalWrite(directPin, HIGH);
        else digitalWrite(directPin, LOW);
        if (data[3] == 1) digitalWrite(lightPin, HIGH);
        else digitalWrite(lightPin, LOW);
        data[0] = data[0] / 100.0 * 5;
        data[1] = data[1] / 100.0 * 5;
        
        dac.writeA(data[0]);
        dac.writeB(data[1]);
    }
    return;
}

void showInfo() {
    Serial.println("----------------");
    Serial.println("autonomous mode");
    Serial.print("Throttle Voltage : ");
    Serial.println(data[0]);\
    Serial.print("Brake Voltage : ");
    Serial.println(data[1]);
    Serial.print("Direction : ");
    if (data[2] == 1) {
      Serial.println("Reverse");
    }
    else Serial.println("Forward");
    if (data[3] == 1) Serial.println("Lights ON");
    else Serial.println("Lights OFF");
    

    result ="autonomous mode";
    char x[60];
    result.toCharArray(x,60);
    str_msg.data= x;
    chatter.publish(&str_msg);
    nh.spinOnce();
    delay(50);
    return;

}
void calVel() {
    static int count = 0;
    count = count + 1;
    if (count <= 3) previousTime = millis();
    else if (count >= 64) {
        vel = distance / (millis() - previousTime);
        count = 0;
    }
    return;
}

/* block for change to ros data  
  char x[60];
  result.toCharArray(x,60);
  str_msg.data= x;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(50);
  return;
*/
