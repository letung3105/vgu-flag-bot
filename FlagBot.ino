/**
 * FlagBot control
 * Vietnamese-German University
 * By Tung Le Vo
 * Line following robot with obstacles detection and servo for raising flag
 * Last Modified 11th April 2018
 * Connections:
 *     Left Motor -> Motor 1 (Motor Shield)
 *          Control using pin D4, D5
 *     Right Motor -> Motor 2 (Motor Shield)
 *          Control using pin D6, D7
 *     +5V -> Line Sensor -> D10 -> GND
 *     +5V -> Line Sensor -> D2 -> GND
 *     +5V -> Line Sensor -> D3 -> GND
 *     +5V -> Line Sensor -> D13 -> GND
 *     +5V -> Line Sensor -> D11 -> GND
 *     +5V -> Ultrasonic Sensor -> D8 -> GND
 *     +5V -> Ultrasonic Sensor -> D9 -> GND
 *     +5V -> Servo -> D12 -> GND
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

// Servo object for raising the flag
Servo flagServo;

// DC motors PWM speed limits
// Normal: 150 - 160
// Faster: 170 - 180
const uint8_t LEFT_BASE_SPEED = 160;
const uint8_t RIGHT_BASE_SPEED = 160;
const uint8_t LEFT_MAX_SPEED = 170;
const uint8_t RIGHT_MAX_SPEED = 170;

// Offtrack mode adjusted speed
// Normal: 100-100
// Faster: 110-110

const uint8_t OFFTRACK_LEFT_BASE_SPEED = 110;
const uint8_t OFFTRACK_RIGHT_BASE_SPEED = 110;
const uint8_t OFFTRACK_LEFT_MAX_SPEED = 110;
const uint8_t OFFTRACK_RIGHT_MAX_SPEED = 110;

// PID values
// Normal: 50-0-60
// Faster: 30-0-40
const float Kp = 30;
const float Ki = 0;
const float Kd = 30;
float P = 0;
float I = 0;
float D = 0;

// Distance threshold
const uint8_t DISTANCE_TO_GATE = 50;
const uint8_t DISTANCE_TO_OBSTACLE = 10;
const uint8_t DISTANCE_TO_WALL = 25;

// The line sensors are indexed from left to right
const uint8_t numLineSensors = 5;
const uint8_t lineSensorPins[numLineSensors] = {10, 2, 3, 13, 11};
uint8_t lineSensors[numLineSensors];

// Pins for the ultrasonic sensor
const uint8_t leftUltraSonicSensor = 8;
const uint8_t rightUltraSonicSensor = 9;

// Pins for MotorShield control
const DCMotor leftMotor = {4, 5};
const DCMotor rightMotor = {7, 6};

// Line positions
int8_t currentError = 0;
int8_t previousError = 0;

// LEDs
const uint8_t leftLED = A0;
const uint8_t rightLED = A1;

// Helper varialbles
int16_t valuePID;
uint8_t leftMotorSpeed;
uint8_t rightMotorSpeed;

uint8_t adj_left_max;
uint8_t adj_right_max;
uint8_t adj_left_base;
uint8_t adj_right_base;

bool isRaiseFlag = false;


void setup() {
    // DC Motors Pins
    pinMode(leftMotor.direction, OUTPUT);
    pinMode(rightMotor.direction, OUTPUT);
    pinMode(leftMotor.speed, OUTPUT);
    pinMode(rightMotor.speed, OUTPUT);

    // Set servo control pin to D12
    flagServo.attach(12);
    flagServo.write(10);

    // Line Sensor Pins
    for (uint8_t i=0; i < numLineSensors; i++){
        pinMode(lineSensorPins[i], INPUT);
        lineSensors[i] = 0;
    }

    // LED pins
    pinMode(leftLED, OUTPUT);
    pinMode(rightLED, OUTPUT);

    Serial.begin(9600);

    digitalWrite(leftLED, HIGH);
    digitalWrite(rightLED, HIGH);

    delay(1000);
    // Keep looping while the gate is still closing
    while (detectObstacle(DISTANCE_TO_WALL)){
        Serial.println("Stoping at gate");
        delay(100);
    }
    delay(1000); // Wait for the gate to fully open
}


void loop() {
    currentError = getError();
    if (detectObstacle(DISTANCE_TO_GATE)){
        // Stop running if there's an obstacle
        analogWrite(leftMotor.speed, 0);
        analogWrite(rightMotor.speed, 0);
        digitalWrite(leftLED, HIGH);
        digitalWrite(rightLED, HIGH);
        delay(400);
        if (detectObstacle(DISTANCE_TO_WALL) && !isRaiseFlag){
            flagServo.write(80);
            delay(500);
            turnAround();
        }
    } else {
        // This is when no line sensor can detect the line
        if (currentError == 100){
            currentError = previousError;

            adj_left_base = OFFTRACK_LEFT_BASE_SPEED;
            adj_right_base = OFFTRACK_RIGHT_BASE_SPEED;
            adj_left_max = OFFTRACK_LEFT_MAX_SPEED;
            adj_right_max = OFFTRACK_RIGHT_MAX_SPEED;
        } else {
            adj_left_base = LEFT_BASE_SPEED;
            adj_right_base = RIGHT_BASE_SPEED;
            adj_left_max = LEFT_MAX_SPEED;
            adj_right_max = RIGHT_MAX_SPEED;
        }

        valuePID = getPID(currentError, previousError);
        leftMotorSpeed = constrain(adj_left_base + valuePID, 0, adj_left_max);
        rightMotorSpeed = constrain(adj_right_base - valuePID, 0, adj_right_max);
        //do {
            runLeftMotor(FORWARD, leftMotorSpeed);
            runRightMotor(FORWARD, rightMotorSpeed);
            digitalWrite(leftLED, leftMotorSpeed < rightMotorSpeed);
            digitalWrite(rightLED, leftMotorSpeed > rightMotorSpeed);
        //} while (digitalRead(lineSensorPins[1]) && digitalRead(lineSensorPins[2]) && (digitalRead(lineSensorPins[3])));

    }
    if (currentError != 100) previousError = currentError;
    // delay(10);
}
