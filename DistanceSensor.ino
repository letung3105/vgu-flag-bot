/**
 * Ultrasonic Sensor HC-SR04
 * Vietnamese-German University
 * By Tung Le Vo
 * Calculate distance to obstacles with HC-SR04
 * Last Modified 9th March 2018
 */


/**
 * Calculate the distance from the FlagBot to the nearest object
 * @param  ultrasonicSensor Storing trigger pin number and echo pin number
 * @return                  The distance in centimeters
 */
uint16_t getDistance(uint8_t mySensor) {
    // Sending out sound waves
    pinMode(mySensor, OUTPUT);
    digitalWrite(mySensor, LOW);
    delayMicroseconds(2);
    digitalWrite(mySensor, HIGH);
    delayMicroseconds(10);
    digitalWrite(mySensor, LOW);
    pinMode(mySensor, INPUT);
    uint16_t duration = pulseIn(mySensor, HIGH); // Return pulses duration (ms)
    uint16_t distance = (duration / 2) * 0.034; // Convert miliseconds to centimeters
    Serial.print("Distance: ");
    Serial.println(distance);
    return distance;
}


bool getFrontInf(){
    for (uint8_t i=0; i < numWallSensors; i++){
        if (digitalRead(wallSensorPins[i])) return true;
    }
    return false;
}


bool detectObstacle(uint8_t threshold){
    uint16_t distance = getDistance(ultraSonicSensor);
    if ((distance > 0 && distance <= threshold) || getFrontInf()){
        return true;
    }
    return false;
}


// bool detectWall(uint8_t threshold){
//     uint16_t distance = getDistance(ultraSonicSensor);
//     if ((distance > 0 && distance <= threshold) && getFrontInf()){
//         return true;
//     }
//     return false;
// }
