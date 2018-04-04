/**
 * DFRobot L298Pv1.1 MotorShield Control Module
 * Vietnamese-German University
 * By Tung Le Vo
 * Control 2 DC motors through the shield using PWN speed control mode:
 * Last Modified 9th March 2018
 */


void runLeftMotor(bool direction, uint8_t speed){
    digitalWrite(leftMotor.direction, direction);
    analogWrite(leftMotor.speed, speed);
}


void runRightMotor(bool direction, uint8_t speed){
    digitalWrite(rightMotor.direction, direction);
    analogWrite(rightMotor.speed, speed);
}


int8_t getError(){
    if (digitalRead(lineSensorPins[0])
        && !digitalRead(lineSensorPins[1])
        && digitalRead(lineSensorPins[2]))
        return 0;
    else if (!digitalRead(lineSensorPins[0])
        && !digitalRead(lineSensorPins[1])
        && digitalRead(lineSensorPins[2]))
        return -10;
    else if (digitalRead(lineSensorPins[0])
        && !digitalRead(lineSensorPins[1])
        && !digitalRead(lineSensorPins[2]))
        return 10;
    else if (!digitalRead(lineSensorPins[0])
        && digitalRead(lineSensorPins[1])
        && digitalRead(lineSensorPins[2]))
        return -20;
    else if (digitalRead(lineSensorPins[0])
        && digitalRead(lineSensorPins[1])
        && !digitalRead(lineSensorPins[2]))
        return 20;
    else return 100;
}


float getPID(int8_t error, int8_t previous_error){
    P = error;
    I = I + error;
    D = error - previous_error;
    int16_t value = (Kp * P) + (Ki * I) + (Kd * D);
    return value;
}


void turnAround(){
    runLeftMotor(BACKWARD, LEFT_BASE_SPEED / 2);
    runRightMotor(FORWARD, RIGHT_BASE_SPEED / 2);
    delay(100);
    do {
        runLeftMotor(BACKWARD, LEFT_BASE_SPEED);
        runRightMotor(FORWARD, RIGHT_BASE_SPEED);
        Serial.println("TURNING AROUND");
    } while (digitalRead(lineSensorPins[1]));
}
