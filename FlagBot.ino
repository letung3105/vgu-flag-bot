/*
FlagBot control
Vietnamese-German University
By Tung Le Vo
Line following robot with obstacles detection and servo for raising flag
Last Modified 8 March 2018
Connections:
    Left Motor -> Motor 1 pins (Shield)
    Right Motor -> Motor 2 pins (Shield)
    Line Sensor -> Digital Pin 2
    Line Sensor -> Digital Pin 3
    Line Sensor -> Digital Pin 11
    Line Sensor -> Digital Pin 12
    Ultrasonic Sensor Trigger Pin -> Digital Pin 8
    Ultrasonic Sensor Echo Pin -> Digital Pin 9
*/


// Custom struct for controlling DC direction and speed
struct DCMotor {
    unsigned int direction;
    unsigned int speed;
};

const int rightSensorInput = 2;
const int leftSensorInput = 3;

const int outerRightSensorInput = 11;
const int outerLeftSensorInput = 12;

const int utrasonicTriggerPin = 8;
const int utrasonicEchoPin = 9;

// Pins for MotorShield control
DCMotor leftMotor = {4, 5};
DCMotor rightMotor = {7, 6};


void setup() {
    // Output pins setup
    pinMode(leftMotor.direction, OUTPUT);
    pinMode(rightMotor.direction, OUTPUT);
    pinMode(utrasonicTriggerPin, OUTPUT);

    // Input pins setup
    pinMode(utrasonicEchoPin, INPUT);
    pinMode(outerRightSensorInput, INPUT);
    pinMode(outerLeftSensorInput, INPUT);
    pinMode(rightSensorInput, INPUT);
    pinMode(leftSensorInput, INPUT);

    Serial.begin(9600);
}


void loop() {
    if (getDistance(utrasonicTriggerPin, utrasonicEchoPin) <= 20) {
        // Stop both motors
        analogWrite(leftMotor.speed, 0);
        analogWrite(rightMotor.speed, 0);
    }
    else {
        bool detectLeft = !digitalRead(leftSensorInput);
        bool detectOuterLeft = !digitalRead(outerLeftSensorInput);
        bool detectRight = !digitalRead(rightSensorInput);
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
