/**
 * Ultrasonic Sensor HC-SR04
 * Vietnamese-German University
 * By Tung Le Vo
 * Calculate distance to obstacles with HC-SR04
 * Last Modified 9th March 2018
 */


/**
 * Calculate distance from FlagBot to the nearest object
 * @param  ultrasonicSensorTrigger unsigned int, OUTPUT PIN
 * @param  ultrasonicSensorEcho    unsigned int, INPUT PIN
 * @return                         float
 */
float getDistance(unsigned int ultrasonicSensorTrigger, unsigned int ultrasonicSensorEcho) {
    // Sending out sound waves
    digitalWrite(ultrasonicSensorTrigger, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicSensorTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicSensorTrigger, LOW);

    unsigned long duration = pulseIn(ultrasonicSensorEcho, HIGH); // Return pulses duration (ms)
    float distance = duration/29/2; // Convert miliseconds to centimeters

    return distance;
}
