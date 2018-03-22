/**
 * DFRobot L298Pv1.1 MotorShield Control Module
 * Vietnamese-German University
 * By Tung Le Vo
 * Control 2 DC motors through the shield using PWN speed control mode:
 * Last Modified 9th March 2018
 */


void runMotor(DCMotor motor, bool direction, uint8_t speed){
    digitalWrite(motor.direction, direction);
    analogWrite(motor.speed, speed);
}


int16_t calculatePID(int8_t err, int8_t prev_err){
    P = err;
    I += err;
    D = err - prev_err;
    int16_t PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    return PIDvalue;
}


int8_t getError(){
    int8_t error = 0;
    if (lineSensors[0] && lineSensors[1] && !lineSensors[2] && lineSensors[3] && lineSensors[4]){
        error = 0;
    } else if (lineSensors[0] && lineSensors[1] && !lineSensors[2] && !lineSensors[3] && lineSensors[4]){
        error = 1;
    } else if (lineSensors[0] && !lineSensors[1] && !lineSensors[2] && lineSensors[3] && lineSensors[4]){
        error = -1;
    } else if (lineSensors[0] && lineSensors[1] && lineSensors[2] && !lineSensors[3] && lineSensors[4]){
        error = 2;
    } else if (lineSensors[0] && !lineSensors[1] && lineSensors[2] && lineSensors[3] && lineSensors[4]){
        error = -2;
    } else if (lineSensors[0] && lineSensors[1] && lineSensors[2] && !lineSensors[3] && !lineSensors[4]){
        error = 3;
    } else if (!lineSensors[0] && !lineSensors[1] && lineSensors[2] && lineSensors[3] && lineSensors[4]){
        error = -3;
    } else if (lineSensors[0] && lineSensors[1] && lineSensors[2] && lineSensors[3] && !lineSensors[4]){
        error = 4;
    } else if (!lineSensors[0] && lineSensors[1] && lineSensors[2] && lineSensors[3] && lineSensors[4]){
        error = -4;
    // } else if (lineSensors[0] && lineSensors[1] && lineSensors[2] && lineSensors[3] && lineSensors[4]){
    //     error = 10;
    }
    return error;
}
