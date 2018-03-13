/**
 * FlagBot control
 * Vietnamese-German University
 * By Tung Le Vo
 * Line following robot with obstacles detection and servo for raising flag
 * Last Modified 9th March 2018
 * Connections:
 *     Left Motor -> Motor 1 pins (Shield)
 *     Right Motor -> Motor 2 pins (Shield)
 *     +5v -> Line Sensor -> Digital Pin 8 -> GND
 *     +5v -> Line Sensor -> Digital Pin 3 -> GND
 *     +5v -> Line Sensor -> Digital Pin 11 -> GND
 *     +5v -> Line Sensor -> Digital Pin 12 -> GND
 *     +5v -> Ultrasonic Sensor -> Digital Pin 8 (Trigger) -> Digital Pin 9 (Echo) -> GND
 */

// Custom struct for storing DC Motors direction and speed pins
struct DCMotor {
    unsigned int direction;
    unsigned int speed;
};

const unsigned int outerRightLineSensor = 11;
const unsigned int rightLineSensor = 2;
const unsigned int leftLineSensor = 3;
const unsigned int outerLeftLineSensor = 12;

const unsigned int ultrasonicSensorTrigger = 8;
const unsigned int ultrasonicSensorEcho = 9;

// Pins for MotorShield control
DCMotor leftMotor = {4, 5};
DCMotor rightMotor = {7, 6};

bool idOpen = false;


void setup() {
    // DC Motors Pins
    pinMode(leftMotor.direction, OUTPUT);
    pinMode(rightMotor.direction, OUTPUT);

    // Ultrasonic Sensor Pins
    pinMode(ultrasonicSensorEcho, INPUT);
    pinMode(ultrasonicSensorTrigger, OUTPUT);

    // Line Sensor Pins
    pinMode(outerRightLineSensor, INPUT);
    pinMode(rightLineSensor, INPUT);
    pinMode(outerLeftLineSensor, INPUT);
    pinMode(leftLineSensor, INPUT);

    Serial.begin(9600);
}


void loop() {
    if (getDistance(ultrasonicSensorTrigger, ultrasonicSensorEcho) <= 20){
        // Stop both motors
        analogWrite(leftMotor.speed, 0);
        analogWrite(rightMotor.speed, 0);
    }
    else {
        // These are true when the line sensors come across a black line
        bool detectLeft = !digitalRead(leftLineSensor);
        bool detectOuterLeft = !digitalRead(outerLeftLineSensor);
        bool detectRight = !digitalRead(rightLineSensor);
        bool detectOuterRight = !digitalRead(outerRightLineSensor);

        // TODO Add center line sensor for better control
        // TODO Improve robot movement
        if (!detectLeft && !detectOuterLeft && !detectRight && !detectOuterRight) {
            runForward();
        } else if (detectLeft || detectOuterLeft) {
            turnLeft();
        } else if (detectRight || detectOuterRight) {
            turnRight();
        }
    }
}
