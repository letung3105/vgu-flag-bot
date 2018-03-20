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


// Storing DC motors direction and speed control pins
typedef struct {
    const uint8_t direction;
    const uint8_t speed;
} DCMotor;

// Storing ultrasonic sensor trigger and echo pins
typedef struct {
    const uint8_t trigger;
    const uint8_t echo;
} UltrasonicSensor;

// DC motors speed limits
const uint8_t LEFT_BASE_SPEED = 200;
const uint8_t RIGHT_BASE_SPEED = 200;
const uint8_t LEFT_MAX_SPEED = 250;
const uint8_t RIGHT_MAX_SPEED = 250;

// The line sensors are indexed from left to right
const uint8_t numLineSensors = 5;
const uint8_t lineSensorPins[numLineSensors] = {2, 3, 5, 6, 7};
uint8_t lineSensors[numLineSensors];

// Pins for the ultrasonic sensor
const UltrasonicSensor ultrasonicSensor = {8, 9};

// Pins for MotorShield control
const DCMotor leftMotor = {12, 10};
const DCMotor rightMotor = {13, 11};

// Tracking current and previous err (postion of the FlagBot)
int8_t err = 0;
int8_t prev_err = 0;

// Parameters for PID control
int16_t P = 0, I = 0, D = 0;
const uint8_t Kp = 50;
const uint8_t Ki = 0;
const uint8_t Kd = 0;


void setup() {
    // DC Motors Pins
    pinMode(leftMotor.direction, OUTPUT);
    pinMode(rightMotor.direction, OUTPUT);

    // Ultrasonic Sensor Pins
    pinMode(ultrasonicSensor.echo, INPUT);
    pinMode(ultrasonicSensor.trigger, OUTPUT);

    // Line Sensor Pins
    for (int i=0; i < numLineSensors; i++){
        pinMode(lineSensorPins[i], INPUT);
    }

    Serial.begin(9600);
}


void loop() {
    Serial.println(getDistance(ultrasonicSensor));
    // if (getDistance(ultrasonicSensor) <= 20){
    //     // Stop both motors
    //     analogWrite(leftMotor.speed, 0);
    //     analogWrite(rightMotor.speed, 0);
    // }
    // else {
        for (int i=0; i < numLineSensors; i++){
            lineSensors[i] = digitalRead(lineSensorPins[i]);
        }

        err = getError();
        int16_t PIDval = calculatePID(err, prev_err);
        prev_err = err;

        uint8_t leftMotorSpeed = constrain(LEFT_BASE_SPEED + PIDval, 0, 255);
        uint8_t rightMotorSpeed = constrain(RIGHT_BASE_SPEED - PIDval, 0, 255);
        Serial.print("Left motor speed: ");
        Serial.println(leftMotorSpeed);
        Serial.print("Right motor speed: ");
        Serial.println(rightMotorSpeed);
        Serial.print("\n");
        runMotor(leftMotor, HIGH, leftMotorSpeed);
        runMotor(rightMotor, HIGH, rightMotorSpeed);
    delay(1000);
}
