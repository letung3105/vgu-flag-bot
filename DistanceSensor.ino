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
float getDistance(UltrasonicSensor ultrasonicSensor) {
    // Sending out sound waves
    digitalWrite(ultrasonicSensor.trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicSensor.trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicSensor.trigger, LOW);

    uint16_t duration = pulseIn(ultrasonicSensor.echo, HIGH); // Return pulses duration (ms)
    float distance = duration/29/2; // Convert miliseconds to centimeters

    return distance;
}
