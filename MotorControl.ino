/**
 * DFRobot L298Pv1.1 MotorShield Control Module
 * Vietnamese-German University
 * By Tung Le Vo
 * Control 2 DC motors through the shield using PWN speed control mode:
 * Last Modified 9th March 2018
 */

#define FORWARD HIGH
#define BACKWARD LOW

// Constants for motor's speed control
const unsigned int FULL_SPEED = 255;
const unsigned int HALF_SPEED = 127;
const unsigned int QUARTER_SPEED = 64;


/**
 * Running both motors forward at HALF_SPEED
 */
void runForward() {
    // Serial.println("FORWARD");
    digitalWrite(rightMotor.direction, FORWARD);
    digitalWrite(leftMotor.direction, FORWARD);
    analogWrite(rightMotor.speed, HALF_SPEED);
    analogWrite(leftMotor.speed, HALF_SPEED);
}


/**
 * Turn left by slow left motor speed to QUARTER_SPEED / 2
 */
void turnLeft() {
    // Serial.println("LEFT");
    digitalWrite(rightMotor.direction, FORWARD);
    digitalWrite(leftMotor.direction, FORWARD);
    analogWrite(rightMotor.speed, HALF_SPEED);
    analogWrite(leftMotor.speed, QUARTER_SPEED / 2);
}


/**
 * Turn right by slow right motor speed to QUARTER_SPEED / 2
 */
void turnRight() {
    // Serial.println("RIGHT");
    digitalWrite(rightMotor.direction, FORWARD);
    digitalWrite(leftMotor.direction, FORWARD);
    analogWrite(rightMotor.speed, QUARTER_SPEED / 2);
    analogWrite(leftMotor.speed, HALF_SPEED);
}
