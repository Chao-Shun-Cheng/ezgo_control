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
#define RX_LENGTH 7
#define TX_LENGTH 7

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
volatile float vel = 0;
float distance = diameter * pi / gear_ratio * 1000 * 3.6;
double previousTime = 0;
DAC8562 dac = DAC8562(SSPin, REF_POWER);

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
    delay(10);
}

void getData()
{
    static byte reader_buf[RX_LENGTH] = {0};
    if (Serial.readBytes(reader_buf, RX_LENGTH) == RX_LENGTH) {
        dac.writeA(((float) reader_buf[1]) / 255.0 * 5.0); // Throttle
        dac.writeB(((float) reader_buf[0]) / 255.0 * 5.0); // Brake
        switch (reader_buf[4]) {
            case 0:
                digitalWrite(directPin, LOW);
                break;
            case 1:
                digitalWrite(directPin, HIGH);
                break;
            case 2:
                digitalWrite(directPin, LOW);
                break;
            case 3:
                digitalWrite(directPin, LOW);
                break;
            default:
                break;
        }
    }
    return;
}

void showInfo()
{
    static byte writer_buf[TX_LENGTH] = {0};
    writer_buf[0] = reader_buf[0];
    writer_buf[1] = reader_buf[1];

    int current_vel = vel * 1000;
    writer_buf[4] = (current_vel & 0xff00) >> 8;
    writer_buf[5] = current_vel & 0x00ff;

    writer_buf[6] = ((mode & 0x1) << 5); // control mode
    writer_buf[6] |= reader_buf[4];      // shift

    Serial.write(writer_buf, TX_LENGTH);
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
