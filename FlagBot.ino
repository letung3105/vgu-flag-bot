/**
 * FlagBot control
 * Vietnamese-German University
 * By Tung Le Vo
 * Line following robot with obstacles detection and servo for raising flag
 * Last Modified 19th March 2018
 * Connections:
 *     Left Motor -> Motor 1 pins (Shield)
 *     Right Motor -> Motor 2 pins (Shield)
 *     +5v -> Line Sensor -> Digital Pin 8 -> GND
 *     +5v -> Line Sensor -> Digital Pin 3 -> GND
 *     +5v -> Line Sensor -> Digital Pin 11 -> GND
 *     +5v -> Line Sensor -> Digital Pin 12 -> GND
 *     +5v -> Ultrasonic Sensor -> Digital Pin 8 (Trigger) -> Digital Pin 9 (Echo) -> GND
 */
// TODO: Rewrite connections in header comments

#include<Servo.h>

#define FORWARD HIGH
#define BACKWARD LOW

// Storing DC motors direction and speed control pins
typedef struct {
    const uint8_t direction;
    const uint8_t speed;
} DCMotor;

// Ser for raising the flag
Servo flagServo;

// DC motors speed limits
const uint8_t LEFT_BASE_SPEED = 125;
const uint8_t RIGHT_BASE_SPEED = 125;
const uint8_t LEFT_MAX_SPEED = 140;
const uint8_t RIGHT_MAX_SPEED = 140;

const uint8_t DISTANCE_TO_OBSTACLE = 20;

// The line sensors are indexed from left to right
const uint8_t numLineSensors = 3;
const uint8_t lineSensorPins[numLineSensors] = {2, 3, 13};
uint8_t lineSensors[numLineSensors];

// Using line detecting sensors for wall detection
const uint8_t numWallSensors =  2;
const uint8_t wallSensorPins[numWallSensors] = {10, 11};

// Pins for the ultrasonic sensor
const uint8_t ultraSonicSensor = 9;

// Pins for MotorShield control
const DCMotor leftMotor = {4, 5};
const DCMotor rightMotor = {7, 6};
const uint8_t leftLED = 13;
const uint8_t rightLED = 0;

// Proportional value
const float Kp = 4;
const float Ki = 0;
const float Kd = 4;
float P = 0;
float I = 0;
float D = 0;

// Bot's current state
int8_t currentError = 0;
int8_t previousError = 0;


void setup() {
    // DC Motors Pins
    pinMode(leftMotor.direction, OUTPUT);
    pinMode(rightMotor.direction, OUTPUT);
    pinMode(leftMotor.speed, OUTPUT);
    pinMode(rightMotor.speed, OUTPUT);

    // LEDs pins
    // pinMode(leftLED, OUTPUT);
    // pinMode(rightLED, OUTPUT);

    // Flag Servo Pin
    flagServo.attach(12);
    flagServo.write(0);

    // Line Sensor Pins
    for (uint8_t i=0; i < numLineSensors; i++){
        pinMode(lineSensorPins[i], INPUT);
        lineSensors[i] = 0;
    }

    // Wall Sensor Pins
    for (uint8_t i=0; i < numWallSensors; i++){
        pinMode(wallSensorPins[i], INPUT);
    }

    Serial.begin(9600);

    while (detectObstacle(10)){
        Serial.println("Stoping at gate");
        // digitalWrite(leftLED, HIGH);
        // digitalWrite(rightLED, HIGH);
    }
    delay(1000);
}


void loop() {
    currentError = getError();
    if (currentError == 100){
        currentError = previousError;
    }
    // Serial.print("Error: ");
    // Serial.println(currentError);

    int16_t valuePID = getPID(currentError, previousError);
    // Serial.print("PID value: ");
    // Serial.println(valuePID);

    uint8_t leftMotorSpeed = constrain(LEFT_BASE_SPEED + valuePID, 0, LEFT_MAX_SPEED);
    uint8_t rightMotorSpeed = constrain(RIGHT_BASE_SPEED - valuePID, 0, RIGHT_MAX_SPEED);
    // Serial.print("Left Motor: ");
    // Serial.println(leftMotorSpeed);
    // Serial.print("Right Motor: ");
    // Serial.println(rightMotorSpeed);

    // if (leftMotorSpeed == rightMotorSpeed){
    //     Serial.println("FORWARD");
    // } else if (leftMotorSpeed < rightMotorSpeed){
    //     Serial.println("TURN LEFT");
    // } else if (leftMotorSpeed > rightMotorSpeed){
    //     Serial.println("TURN RIGHT");
    // }

    if (detectWall(DISTANCE_TO_OBSTACLE)){
        // Stop both motors
        analogWrite(leftMotor.speed, 0);
        analogWrite(rightMotor.speed, 0);

        delay(2000);
        if (detectWall(DISTANCE_TO_OBSTACLE)){
            // Serial.println("Wall detected");

            // digitalWrite(leftLED, HIGH);
            // digitalWrite(rightLED, HIGH);

            flagServo.write(90);
            delay(2000);
            turnAround();
        }
    } else {
        runLeftMotor(FORWARD, leftMotorSpeed);
        runRightMotor(FORWARD, rightMotorSpeed);
    }

    previousError = currentError;
    // delay(10);
}
