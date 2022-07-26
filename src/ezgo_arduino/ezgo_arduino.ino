/*
 *  Testing Code for E-Z-GO Glof Car
 *  This Package is able to control throttle, brake and light.
 *  Besides, it has two control mode, manual mode, autonumous mode.
 */
#include "DAC8562.h"

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
byte mode = 0;
volatile float vel = 0;
float distance = diameter * pi / gear_ratio * 1000 * 3.6;
double previousTime = 0;
DAC8562 dac = DAC8562(SSPin, REF_POWER);

byte writer_buf[5] = {0};
byte reader_buf[3] = {0};

void setup()
{
    pinMode(directPin, OUTPUT);
    pinMode(modePin, INPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    pinMode(lightPin, OUTPUT);
    Serial.begin(115200);
    Serial.setTimeout(50);
    dac.begin();
    attachInterrupt(digitalPinToInterrupt(interruptPin), calVel, RISING);
}

void loop()
{
    mode = digitalRead(modePin);
    if (mode == HIGH) {
        getData();
    }
    showInfo();
    delay(100);
}

void getData()
{
    if (Serial.readBytes(reader_buf, 3) == 3) {
        dac.writeA(((float) reader_buf[0]) / 255.0 * 5.0);
        dac.writeB(((float) reader_buf[1]) / 255.0 * 5.0);
        if (reader_buf[2] & 0x01) {
            digitalWrite(directPin, HIGH);
        } else {
            digitalWrite(directPin, LOW);
        }
    }
    return;
}

void showInfo()
{
    writer_buf[0] = reader_buf[0];
    writer_buf[1] = reader_buf[1];

    writer_buf[2] = (mode & 0x01) | ((reader_buf[2] << 2) & 0x04);
    // light signal test
    // writer_buf[2] = (mode & 0x01) | ((reader_buf[2] << 2) & 0x04) | ((reader_buf[2] << 1) & 0x02); 
    int current_vel = vel * 1000;
    writer_buf[3] = (current_vel & 0xff00) >> 8;
    writer_buf[4] = current_vel & 0x00ff;
    Serial.write(writer_buf, 5);
    return;
}

void calVel()
{
    static int count = 0;
    count = count + 1;
    if (count <= 3)
        previousTime = millis();
    else if (count >= 64) {
        vel = distance / (millis() - previousTime);
        count = 0;
    }
    return;
}
