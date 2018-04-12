/**
 * DFRobot L298Pv1.1 MotorShield Control Module
 * Vietnamese-German University
 * By Tung Le Vo
 * Control 2 DC motors through the shield using PWN speed control mode:
 * Last Modified 11th April 2018
 */


// Drive left motor
void runLeftMotor(bool direction, uint8_t speed){
    analogWrite(leftMotor.speed, speed);
    digitalWrite(leftMotor.direction, direction);
}


// Drive right motor
void runRightMotor(bool direction, uint8_t speed){
    analogWrite(rightMotor.speed, speed);
    digitalWrite(rightMotor.direction, direction);
}


// Produce error based on the position of the line
int8_t getError(){
    for (uint8_t i=0; i < numLineSensors; i++){
        lineSensors[i] = !digitalRead(lineSensorPins[i]);
    }
    if ((lineSensors[0] == 0) && (lineSensors[1] == 0) && (lineSensors[2] == 0) && (lineSensors[3] == 0) && (lineSensors[4] == 1)) return 4;
    if ((lineSensors[0] == 0) && (lineSensors[1] == 0) && (lineSensors[2] == 0) && (lineSensors[3] == 1) && (lineSensors[4] == 1)) return 3;
    if ((lineSensors[0] == 0) && (lineSensors[1] == 0) && (lineSensors[2] == 0) && (lineSensors[3] == 1) && (lineSensors[4] == 0)) return 2;
    if ((lineSensors[0] == 0) && (lineSensors[1] == 0) && (lineSensors[2] == 1) && (lineSensors[3] == 1) && (lineSensors[4] == 0)) return 1;
    if ((lineSensors[0] == 0) && (lineSensors[1] == 0) && (lineSensors[2] == 1) && (lineSensors[3] == 0) && (lineSensors[4] == 0)) return 0;
    if ((lineSensors[0] == 0) && (lineSensors[1] == 1) && (lineSensors[2] == 1) && (lineSensors[3] == 0) && (lineSensors[4] == 0)) return -1;
    if ((lineSensors[0] == 0) && (lineSensors[1] == 1) && (lineSensors[2] == 0) && (lineSensors[3] == 0) && (lineSensors[4] == 0)) return -2;
    if ((lineSensors[0] == 1) && (lineSensors[1] == 1) && (lineSensors[2] == 0) && (lineSensors[3] == 0) && (lineSensors[4] == 0)) return -3;
    if ((lineSensors[0] == 1) && (lineSensors[1] == 0) && (lineSensors[2] == 0) && (lineSensors[3] == 0) && (lineSensors[4] == 0)) return -4;
    if ((lineSensors[0] == 0) && (lineSensors[1] == 0) && (lineSensors[2] == 0) && (lineSensors[3] == 0) && (lineSensors[4] == 0)) return 100;

}


// Calculate the PID value
float getPID(int8_t error, int8_t previous_error){
    P = error;
    I = I + error;
    D = error - previous_error;
    int16_t value = (Kp * P) + (Ki * I) + (Kd * D);
    return value;
}


// Spin in place
void turnAround(){
    digitalWrite(leftLED, HIGH);
    digitalWrite(rightLED, LOW);
    runLeftMotor(BACKWARD, 100);
    runRightMotor(FORWARD, 100);
    delay(500);
    // Stop when 1 of the 3 middle sensors detects the line
    do {
        runLeftMotor(BACKWARD, 100);
        runRightMotor(FORWARD, 100);
    } while (digitalRead(lineSensorPins[1]) && digitalRead(lineSensorPins[2]) && (digitalRead(lineSensorPins[3])));
}
