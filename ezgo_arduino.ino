/* 
 *  Testing Code for E-Z-GO Glof Car 
 *  This Package is able to control throttle and brake.
 *  Besides, it has two control mode, manual mode, autonumous mode.
 */

#include <Wire.h>   

/*
 * MCP4725    : This is for giving brake signal.
 * modePin    : This is mode change pin. (digital input)
 * accPin     : This is accelerate pin.  (PWM output)
 * diameter   : 18 inch tire 
 * gear_ratio : gear ratio is 16.99
 */
#define MCP4725 0x60
#define modePin 22     
#define accPin 7    
#define interruptPin 2  
#define directPin 5
#define diameter 0.4572
#define pi 3.14159
#define gear_ratio 16.99

/*
 * mode --> 0 : manual mode, 1 : autonomous mode
 * data --> {acc, dec, direction} : 1 : forward, 2 : reverse
 * distance --> forward distance during moter running a cycle
 */
byte buffer[3];     
int mode = 0;          
volatile int data[3] = {0, 0, 0}; 
volatile float vel = 0; 
float distance = diameter * pi / gear_ratio * 1000 * 3.6; 
double previousTime = 0;

void setup() {
    Serial.begin(9600);      
    Wire.begin();             // Begins the I2C communication
    pinMode(accPin, OUTPUT);
    pinMode(directPin, OUTPUT);  
    pinMode(modePin, INPUT);  
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), calVel, RISING);
}

void loop() {
    int mode = digitalRead(modePin);
    if (mode == HIGH) {
        if (data[2] == 2) digitalWrite(directPin, HIGH);
        else digitalWrite(directPin, LOW);
        getData();
        showInfo();
    } else {
      Serial.println("manual mode");
    }
    delay(200);
}

/*
 * This function is used for sending real analog signal.
 * Input range : 0 ~ 4096
 */
void brake(unsigned int input) {
    buffer[0] = 0b01000000;
    buffer[1] = input >> 4;              
    buffer[2] = input << 4;   
    Wire.beginTransmission(MCP4725);   
    Wire.write(buffer[0]);
    Wire.write(buffer[1]);
    Wire.write(buffer[2]);
    Wire.endTransmission();
    return;
}

void getData() {
    int i = 0;
    char chr;
    if (Serial.available() > 0) {
      data[0] = data[1] = 0;
        while ((chr = Serial.read()) != '\n' && i < 3) {
            if (chr == ',') i++;
            if (chr >= '0' && chr <= '9') data[i] = data[i] * 10 + (chr - '0');
        }
        analogWrite(accPin, data[0]);
        brake(data[1]); 
    }
    return;
}

void showInfo() {
    Serial.println("autonomous mode");
    Serial.print("Throttle Voltage : ");
    Serial.println(data[0]);
    Serial.print("Brake Voltage : ");
    Serial.println(data[1]);
    return;
}

void calVel() {
    static int count = 0;
    count = count + 1;
    if (count <= 3) previousTime = millis();
    else if (count >= 64) {
        vel = motor / (millis() - previousTime);
        count = 0;
    }
    return;
}

//int i = 0;
//char chr;
//if (Serial.available() > 0) {
//  data[0] = data[1] = 0;
//    while ((chr = Serial.read()) != '\n' && i < 2) {
//        if (chr == ',') i++;
//        if (chr >= '0' && chr <= '9') data[i] = data[i] * 10 + (chr - '0');
//    }
//    analogWrite(accPin, data[0]);
//    brake(data[1]); 
//}
