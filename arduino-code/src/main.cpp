// Product Description

// Specifications:
// Item: 80KG Digital Servo
// Storage Temperature: -30 ℃ ~ 80 ℃
// Operating Temperature: -15 ℃ ~ 70 ℃
// Working Voltage: 6-8.4V
// Size: 65 * 30 * 48mm
// Gear Ratio: 357
// Bearing: Double Bearing
// Steering Gear Line: 300 + - 5mm
// Motor: 3-pole (s)
// Waterproof Performance: IP67
// Working Voltage: 6V 4mA 0.24sec/60 ° 85kg cm / 4.1a
// 7.4V 5MA 0.21SEC/60° 98kg-cm/5.4A
// 8.4V 6MA 0.19SEC/60° 105kg-cm/6.5A
// Driving Mode: PWM
// Pulse Width Range: 500-2500 USEC
// Midpoint Position: 1500 usec
// Control Angle: 270 ° (When 500 ~ 2500 USEC)
// Control Accuracy: 3 USEC
// Control Frequency: 50-330hz
// Rotation Direction: Counter Clockwise (When 500 ~ 2500 USEC)

// Package Information:
// Package Size: 8.5*6.5*4cm
// Package Weight: 160g

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 120  // This is the 'minimum' pulse length count (out of 4096) (20000 / 4096) 102.4 TO 512
#define SERVOMAX 505  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int interpolateDegrees(int degrees, int range);

String input;

// our servo # counter
uint8_t servonum = 0;

// the number of servos, up to 8
uint8_t servocount = 3;

void setup()
{
    Serial.begin(115200);
    Serial.println("8 channel Servo test!");

    pwm.begin();
    /*
     * In theory the internal oscillator (clock) is 25MHz but it really isn't
     * that precise. You can 'calibrate' this by tweaking this number until
     * you get the PWM update frequency you're expecting!
     * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     * is used for calculating things like writeMicroseconds()
     * Analog servos run at ~50 Hz updates, It is importaint to use an
     * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
     *    the I2C PCA9685 chip you are setting the value for.
     * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
     *    expected value (50Hz for most ESCs)
     * Setting the value here is specific to each individual I2C PCA9685 chip and
     * affects the calculations for the PWM update frequency.
     * Failure to correctly set the int.osc value will cause unexpected PWM results
     */
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    pwm.setPWM(0, 0, interpolateDegrees(0, 270));
    pwm.setPWM(1, 0, interpolateDegrees(0, 270));
    pwm.setPWM(2, 0, interpolateDegrees(0, 180));
    pwm.setPWM(3, 0, interpolateDegrees(0, 180));

    delay(1000);
}

void loop()
{
    while (Serial.available())
    {
        input = Serial.readString();

        // pwm.setPWM(0, 0, interpolateDegrees(input.toInt(), 270));
        pwm.setPWM(0, 0, input.toInt());
    }

    // Serial.println("0");
    // pwm.setPWM(0, 0, SERVOMIN);
    // pwm.setPWM(1, 0, SERVOMIN);

    // pwm.setPWM(4, 0, SERVOMIN);
    // pwm.setPWM(5, 0, SERVOMIN);

    // delay(4000);

    // Serial.println("1");
    // pwm.setPWM(0, 0, SERVOMAX);
    // pwm.setPWM(1, 0, SERVOMAX);

    // pwm.setPWM(4, 0, SERVOMAX);
    // pwm.setPWM(5, 0, SERVOMAX);

    // delay(4000);

    // // Drive each servo one at a time using setPWM()
    // Serial.println(servonum);
    // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
    // {
    //     pwm.setPWM(servonum, 0, pulselen);
    // }

    // delay(1000);
    // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
    // {
    //     pwm.setPWM(servonum, 0, pulselen);
    // }

    // delay(3000);

    // servonum++;
    // if (servonum > servocount)
    //     servonum = 0;

    // Serial.println("Again");
    // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
    // {
    //     pwm.setPWM(0, 0, pulselen);
    //     pwm.setPWM(1, 0, pulselen);
    //     pwm.setPWM(2, 0, pulselen);
    //     pwm.setPWM(3, 0, pulselen);
    // }

    // delay(1000);
    // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
    // {
    //     pwm.setPWM(0, 0, pulselen);
    //     pwm.setPWM(1, 0, pulselen);
    //     pwm.setPWM(2, 0, pulselen);
    //     pwm.setPWM(3, 0, pulselen);
    // }

    // delay(3000);
}

int interpolateDegrees(int degrees, int range)
{
    return map(degrees, -(range / 2), range / 2, SERVOMIN, SERVOMAX);
}

// #include <Arduino.h>
// #include <Servo.h>

// #define SERVO_PIN 2

// void trapezoidalMove(int startPos, int endPos, float maxVelocity, float maxAcceleration);

// Servo servo;

// String input;

// int currentPos = 0;

// const int maxVelocity = 1000;
// const int maxAcceleration = 1000;

// void setup()
// {
//     Serial.begin(115200);

//     servo.attach(SERVO_PIN);

//     servo.write(0);
// }

// void loop()
// {
//     while (Serial.available())
//     {
//         input = Serial.readString();

//         trapezoidalMove(currentPos, input.toInt(), maxVelocity, maxAcceleration);

//         Serial.print(currentPos);
//         Serial.print(" to ");
//         Serial.println(input.toInt());

//         currentPos = input.toInt();
//     }
// }

// void trapezoidalMove(int startPos, int endPos, float maxVelocity, float maxAcceleration)
// {
//     int distance = abs(endPos - startPos);
//     float accelTime = maxVelocity / maxAcceleration;                     // Time to reach max velocity
//     float accelDistance = 0.5 * maxAcceleration * accelTime * accelTime; // Distance covered during acceleration

//     // If distance is too short for full-speed motion, adjust max velocity
//     if (2 * accelDistance > distance)
//     {
//         maxVelocity = sqrt(distance * maxAcceleration);
//         accelTime = maxVelocity / maxAcceleration;
//         accelDistance = 0.5 * maxAcceleration * accelTime * accelTime;
//     }

//     int position = startPos;
//     float velocity = 0;
//     int sign = (endPos > startPos) ? 1 : -1;
//     int timeStep = 10; // Time step in milliseconds

//     for (int t = 0; sign * position < sign * endPos; t += timeStep)
//     {
//         if (sign * position < sign * (startPos + accelDistance))
//         {
//             // Acceleration phase
//             velocity += maxAcceleration * (timeStep / 1000.0);
//         }
//         else if (sign * position > sign * (endPos - accelDistance))
//         {
//             // Deceleration phase
//             velocity -= maxAcceleration * (timeStep / 1000.0);
//         }
//         else
//         {
//             // Constant velocity phase
//             velocity = maxVelocity;
//         }

//         position += sign * velocity * (timeStep / 1000.0);
//         int servoPosition = constrain(position, 0, 180); // Ensure position is within servo range
//         servo.write(servoPosition);                      // Move servo smoothly

//         delay(timeStep); // Wait before next step
//     }
// }
