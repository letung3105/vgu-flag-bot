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
const unsigned int centerLineSensor = 10;
const unsigned int leftLineSensor = 3;
const unsigned int outerLeftLineSensor = 12;

const unsigned int utrasonicSensorTrigger = 8;
const unsigned int utrasonicSensorEcho = 9;

// Pins for MotorShield control
DCMotor leftMotor = {4, 5};
DCMotor rightMotor = {7, 6};


void setup() {
    // DC Motors Pins
    pinMode(leftMotor.direction, OUTPUT);
    pinMode(rightMotor.direction, OUTPUT);

    // Ultrasonic Sensor Pins
    pinMode(utrasonicSensorEcho, INPUT);
    pinMode(utrasonicSensorTrigger, OUTPUT);

    // Line Sensor Pins
    pinMode(outerRightLineSensor, INPUT);
    pinMode(rightLineSensor, INPUT);
    pinMode(centerLineSensor, INPUT);
    pinMode(outerLeftLineSensor, INPUT);
    pinMode(leftLineSensor, INPUT);

    Serial.begin(9600);
}


void loop() {
    if (getDistance(utrasonicSensorTrigger, utrasonicSensorEcho) <= 20) {
        // Stop both motors
        analogWrite(leftMotor.speed, 0);
        analogWrite(rightMotor.speed, 0);
    }
    else {
        /*
            These are true when the line sensors come across a black line
         */
        bool detectLeft = !digitalRead(leftLineSensor);
        bool detectOuterLeft = !digitalRead(outerLeftSensorInput);
        bool detectRight = !digitalRead(rightLineSensor);
        bool detectOuterRight = !digitalRead(outerRightSensorInput);

        if (!detectLeft && !detectOuterLeft && !detectRight && !detectOuterRight) {
            runForward();
        } else if (detectLeft || detectOuterLeft) {
            turnLeft();
        } else if (detectRight || detectOuterRight) {
            turnRight();
        }
    }
}
